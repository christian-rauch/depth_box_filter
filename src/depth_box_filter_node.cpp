#include <ros/ros.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <depth_image_proc/depth_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
// TODO: remove 'pcl_conversions' header, it is required to silent warnings in Eigen
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <XmlRpcException.h>


class BoxFilter {
public:
    BoxFilter() : n_priv("~"), it(n), tf(tf_buffer) {
        if(!n_priv.getParam("base_frame", base_frame)) {
            ROS_ERROR_STREAM("no base_frame");
            throw std::runtime_error("no base_frame");
        }
        if(n_priv.getParam("camera_frame", camera_frame)) {
            ROS_INFO_STREAM("Will use frame '"<< camera_frame <<"' instead of camera frame");
        }

        const bool use_colour = n_priv.param<bool>("use_colour", false);

        // read plane parameters
        XmlRpc::XmlRpcValue planes_param;
        if(!n_priv.getParam("planes", planes_param)) {
            ROS_ERROR_STREAM("no planes");
            throw std::runtime_error("no planes");
        }

        read_plane_param(planes_param);

        sub_image_depth.subscribe(it, ros::names::remap("depth/image"), 1,
                                  image_transport::TransportHints("raw", ros::TransportHints(), n_priv, "depth/image_transport"));
        sub_info.subscribe(n, ros::names::remap("camera_info"), 1);

        pub_filtered = it.advertise("image_filtered", 1);
        pub_points = n.advertise<sensor_msgs::PointCloud2>("points_filtered", 1);

        points_cam_msg = boost::make_shared<sensor_msgs::PointCloud2>();

        if(use_colour) {
            sub_image_rgb.subscribe(it, ros::names::remap("rgb/image"), 1,
                                    image_transport::TransportHints("raw", ros::TransportHints(), n_priv, "rgb/image_transport"));
            sync_rgbd = std::make_unique<RegisteredSync>(SyncPolRGBD(10), sub_info, sub_image_depth, sub_image_rgb);
            sync_rgbd->registerCallback(boost::bind(&BoxFilter::cb, this, _1, _2, _3));
        }
        else {
            sync = std::make_unique<DepthSync>(SyncPolDepth(10), sub_info, sub_image_depth);
            sync->registerCallback(boost::bind(&BoxFilter::cb, this, _1, _2, nullptr));
        }
    }

    void read_plane_param(XmlRpc::XmlRpcValue &planes_param) {
        if(planes_param.getType()!=XmlRpc::XmlRpcValue::TypeArray) {
            throw std::runtime_error("first level of 'planes' must be an array");
        }

        for(size_t i = 0; i<planes_param.size(); i++) {
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
                planes_base.emplace_back(n.x(), n.y(), n.z(), -n.dot(p));
            }
            catch (XmlRpc::XmlRpcException &e) {
                ROS_ERROR_STREAM("XmlRpc: " << e.getMessage());
                ROS_ERROR_STREAM("Numeric values need to be given explicitly as floating point.");
                throw;
            }
            planes_cam.resize(planes_base.size());
        }
    }

    ~BoxFilter() {
        sync.reset();
        sync_rgbd.reset();
    }

    void cb(const sensor_msgs::CameraInfo::ConstPtr &ci, const sensor_msgs::Image::ConstPtr &depth_img_msg, const sensor_msgs::Image::ConstPtr &rgb_img_msg) {
        camera_model.fromCameraInfo(ci);

        points_cam_msg->header = depth_img_msg->header;
        points_cam_msg->height = depth_img_msg->height;
        points_cam_msg->width  = depth_img_msg->width;
        points_cam_msg->is_dense = uint8_t(false);
        points_cam_msg->is_bigendian = uint8_t(false);

        sensor_msgs::PointCloud2Modifier pcd_modifier(*points_cam_msg);
        if(!rgb_img_msg) {
            pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
        }
        else {
            pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        }

        if (depth_img_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || depth_img_msg->encoding == sensor_msgs::image_encodings::MONO16) {
            depth_image_proc::convert<uint16_t>(depth_img_msg, points_cam_msg, camera_model);
        }
        else if (depth_img_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            depth_image_proc::convert<float>(depth_img_msg, points_cam_msg, camera_model);
        }
        else {
            throw std::runtime_error("unknown image encoding: "+depth_img_msg->encoding);
        }

        if(rgb_img_msg) {
            std::shared_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_ch1;
            std::shared_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_ch2;
            std::shared_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_ch3;

            if(rgb_img_msg->encoding==sensor_msgs::image_encodings::RGB8) {
                iter_ch1 = std::make_shared<sensor_msgs::PointCloud2Iterator<uint8_t>>(*points_cam_msg, "r");
                iter_ch2 = std::make_shared<sensor_msgs::PointCloud2Iterator<uint8_t>>(*points_cam_msg, "g");
                iter_ch3 = std::make_shared<sensor_msgs::PointCloud2Iterator<uint8_t>>(*points_cam_msg, "b");
            }
            else if(rgb_img_msg->encoding==sensor_msgs::image_encodings::BGR8) {
                iter_ch1 = std::make_shared<sensor_msgs::PointCloud2Iterator<uint8_t>>(*points_cam_msg, "b");
                iter_ch2 = std::make_shared<sensor_msgs::PointCloud2Iterator<uint8_t>>(*points_cam_msg, "g");
                iter_ch3 = std::make_shared<sensor_msgs::PointCloud2Iterator<uint8_t>>(*points_cam_msg, "r");
            }
            else {
                ROS_ERROR_STREAM("Unsupported colour encoding: '"+rgb_img_msg->encoding+"'. ");
                throw std::runtime_error("Unsupported colour encoding: '"+rgb_img_msg->encoding+"'.");
            }

            for(uint i = 0; i<points_cam_msg->width*points_cam_msg->height; i++) {
                *(*iter_ch1) = rgb_img_msg->data[i*3+0]; ++(*iter_ch1);
                *(*iter_ch2) = rgb_img_msg->data[i*3+1]; ++(*iter_ch2);
                *(*iter_ch3) = rgb_img_msg->data[i*3+2]; ++(*iter_ch3);
            }
        }

        // transformation from camera to base
        Eigen::Isometry3d T_cb;
        const std::string camera_frame = this->camera_frame.empty() ? depth_img_msg->header.frame_id : this->camera_frame;
        try {
            tf::transformMsgToEigen(tf_buffer.lookupTransform(camera_frame, base_frame, ros::Time(0)).transform, T_cb);
        }
        catch (const tf2::TransformException &e) {
            ROS_DEBUG_STREAM(e.what());
            return;
        }

        // transform planes from base to camera frame
        for(uint i = 0; i<planes_base.size(); i++) {
            // https://math.stackexchange.com/a/1377119
            planes_cam[i] = T_cb.cast<float>().inverse().matrix().transpose() * planes_base[i];
        }

        sensor_msgs::PointCloud2Iterator<float> iter_x(*points_cam_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*points_cam_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*points_cam_msg, "z");
        cv::MatIterator_<uint16_t> it_dimg;

        dimg_filtered = cv_bridge::toCvCopy(depth_img_msg);
        for(it_dimg = dimg_filtered->image.begin<uint16_t>();
            iter_x != iter_x.end();
            ++iter_x, ++iter_y, ++iter_z, it_dimg++)
        {
            for(const Eigen::Vector4f &plane : planes_cam) {
                // check on which side of the plane the point is (positive/negative)
                // http://mathworld.wolfram.com/Point-PlaneDistance.html
                if((plane[0]*(*iter_x) + plane[1]*(*iter_y) + plane[2]*(*iter_z) + plane[3])>0) {
                    // outside, set depth and cooridinates to 0
                    (*it_dimg) = 0;
                    (*iter_x) = (*iter_y) = (*iter_z) = 0;
                    // skip check of remaining planes
                    break;
                }
            }
        }

        pub_filtered.publish(dimg_filtered->toImageMsg());
        pub_points.publish(points_cam_msg);

        pcd_modifier.clear();
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image> SyncPolDepth;
    typedef message_filters::Synchronizer<SyncPolDepth> DepthSync;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image> SyncPolRGBD;
    typedef message_filters::Synchronizer<SyncPolRGBD> RegisteredSync;

    ros::NodeHandle n;
    ros::NodeHandle n_priv;
    image_transport::ImageTransport it;
    image_transport::SubscriberFilter sub_image_depth;
    image_transport::SubscriberFilter sub_image_rgb;
    ros::Publisher pub_points;
    image_transport::Publisher pub_filtered;

    std::unique_ptr<DepthSync> sync;
    std::unique_ptr<RegisteredSync> sync_rgbd;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf;

    std::string base_frame;
    std::string camera_frame;

    // plane parameters (a, b, c, d)
    std::vector<Eigen::Vector4f> planes_base;
    std::vector<Eigen::Vector4f> planes_cam;

    image_geometry::PinholeCameraModel camera_model;
    sensor_msgs::PointCloud2::Ptr points_cam_msg;
    cv_bridge::CvImagePtr dimg_filtered;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth_box_filter");

    BoxFilter filter;

    ros::spin();

    return EXIT_SUCCESS;
}
