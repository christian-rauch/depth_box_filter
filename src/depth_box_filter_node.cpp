#include <cstdlib>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <depth_image_proc/depth_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <XmlRpcException.h>


class BoxFilter {
public:
    BoxFilter() : n_priv("~"), it(n), tf(tf_buffer) {
        if(!n_priv.getParam("base_frame", base_frame)) {
            ROS_WARN_STREAM("no base_frame");
            throw std::runtime_error("no base_frame");
        }

        // read plane parameters
        XmlRpc::XmlRpcValue planes_param;
        if(!n_priv.getParam("planes", planes_param)) {
            ROS_WARN_STREAM("no planes");
            throw std::runtime_error("no planes");
        }

        read_plane_param(planes_param);

        sub_image.subscribe(it, ros::names::remap("image"), 1);
        sub_info.subscribe(n, ros::names::remap("camera_info"), 1);

        pub_filtered = it.advertise("image_filtered", 1);
        pub_points = n.advertise<sensor_msgs::PointCloud2>("points_filtered", 1);

        sync = std::unique_ptr<message_filters::Synchronizer<ApproxSync>>(new message_filters::Synchronizer<ApproxSync>(ApproxSync(10), sub_info, sub_image));
        sync->registerCallback(boost::bind(&BoxFilter::cb, this, _1, _2));
    }

    void read_plane_param(XmlRpc::XmlRpcValue &planes_param) {
        if(planes_param.getType()!=XmlRpc::XmlRpcValue::TypeArray) {
            throw std::runtime_error("first level of 'planes' must be an array");
        }

        for(int i = 0; i<planes_param.size(); i++) {
            if(planes_param[i].getType()!=XmlRpc::XmlRpcValue::TypeArray) {
                throw std::runtime_error("second level of 'planes' must be an array");
            }

            if(planes_param[i].size()!=6) {
                throw std::runtime_error("vectors in 'planes' must have exactly 6 values (nx,ny,nz,px,py,pz)");
            }

            try {
                // read plane normal 'n' and point 'p'
                const Eigen::Vector3d n(planes_param[i][0], planes_param[i][1], planes_param[i][2]);
                const Eigen::Vector3d p(planes_param[i][3], planes_param[i][4], planes_param[i][5]);
                // convert from normal and point to plane equation: ax+by+cz+d=0
                // http://mathworld.wolfram.com/Plane.html
                planes_base.push_back(Eigen::Vector4d({n.x(), n.y(), n.z(), -n.dot(p)}).cast<float>());
            }
            catch (XmlRpc::XmlRpcException &e) {
                ROS_ERROR_STREAM("XmlRpc: " << e.getMessage());
                ROS_ERROR_STREAM("Numeric values need to be given explicitly as floating point.");
                throw;
            }
        }
    }

    ~BoxFilter() {
        sync.reset();
    }

    void cb(sensor_msgs::CameraInfoConstPtr ci, sensor_msgs::ImageConstPtr img_msg) {
        image_geometry::PinholeCameraModel camera_model;
        camera_model.fromCameraInfo(ci);

        sensor_msgs::PointCloud2::Ptr points_cam_msg(new sensor_msgs::PointCloud2);
        points_cam_msg->header = img_msg->header;
        points_cam_msg->height = img_msg->height;
        points_cam_msg->width  = img_msg->width;
        points_cam_msg->is_dense = false;
        points_cam_msg->is_bigendian = false;

        sensor_msgs::PointCloud2Modifier pcd_modifier(*points_cam_msg);
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        if (img_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            depth_image_proc::convert<uint16_t>(img_msg, points_cam_msg, camera_model);
        }
        else if (img_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            depth_image_proc::convert<float>(img_msg, points_cam_msg, camera_model);
        }

        const std::string camera_frame = img_msg->header.frame_id;

        // transformation from camera to base
        Eigen::Isometry3d T_cb;
        try {
            tf::transformMsgToEigen(
                        tf_buffer.lookupTransform(camera_frame, // target
                                                  base_frame,   // source
                                                  ros::Time(0)
                                                  ).transform, T_cb);
        }
        catch (const tf2::TransformException &e) {
            ROS_WARN_STREAM(e.what());
            return;
        }

        // transform planes from base to camera frame
        std::vector<Eigen::Vector4f> planes_cam;
        for(const Eigen::Vector4f &pb : planes_base) {
            // https://math.stackexchange.com/a/1377119
            const Eigen::Vector4f pc = T_cb.cast<float>().inverse().matrix().transpose() * pb;
            planes_cam.push_back(pc);
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*points_cam_msg, cloud);

        // filter points and depth values
        cv_bridge::CvImagePtr dimg = cv_bridge::toCvCopy(img_msg);
        for(int r=0; r<cloud.height; r++) {
            for(int c=0; c<cloud.width; c++) {
                for(const Eigen::Vector4f &p : planes_cam) {
                    // point-to-plane distance
                    // http://mathworld.wolfram.com/Point-PlaneDistance.html
                    const auto d = (p.head<3>().dot(cloud.at(c,r).getVector3fMap()) + p[3]) / p.head<3>().norm();
                    if(d>=0) {
                        // outside
                        dimg->image.at<uint16_t>(cv::Point(c,r)) = 0;
                        cloud.at(c,r) = pcl::PointXYZ();
                    }
                }
            }
        }

        pub_filtered.publish(dimg->toImageMsg());

        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(cloud, out);

        pub_points.publish(out);
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image> ApproxSync;

    ros::NodeHandle n;
    ros::NodeHandle n_priv;
    image_transport::ImageTransport it;
    image_transport::SubscriberFilter sub_image;
    ros::Publisher pub_points;
    image_transport::Publisher pub_filtered;

    std::unique_ptr<message_filters::Synchronizer<ApproxSync>> sync;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf;

    std::string base_frame;

    // plane parameters (a, b, c, d)
    std::vector<Eigen::Vector4f> planes_base;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth_box_filter");

    BoxFilter filter;

    ros::spin();

    return EXIT_SUCCESS;
}
