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

class BoxFilter {
public:
    BoxFilter() : it(n), n_priv("~"), tf(tf_buffer) {
        if(!n_priv.getParam("base_frame", base_frame)) {
            ROS_WARN_STREAM("no base_frame");
            throw std::runtime_error("no base_frame");
        }

        sub_image.subscribe(it, ros::names::remap("image"), 1);
        sub_info.subscribe(n, ros::names::remap("camera_info"), 1);

        pub_points = n.advertise<sensor_msgs::PointCloud2>("points_transformed", 1);

        sync = std::unique_ptr<message_filters::Synchronizer<ApproxSync>>(new message_filters::Synchronizer<ApproxSync>(ApproxSync(10), sub_info, sub_image));
        sync->registerCallback(boost::bind(&BoxFilter::cb, this, _1, _2));
    }

    void cb(sensor_msgs::CameraInfoConstPtr ci, sensor_msgs::ImageConstPtr img_msg) {
        image_geometry::PinholeCameraModel camera_model;
        camera_model.fromCameraInfo(ci);

        sensor_msgs::PointCloud2::Ptr points_cam(new sensor_msgs::PointCloud2);
        points_cam->header = img_msg->header;
        points_cam->height = img_msg->height;
        points_cam->width  = img_msg->width;
        points_cam->is_dense = false;
        points_cam->is_bigendian = false;

        sensor_msgs::PointCloud2Modifier pcd_modifier(*points_cam);
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        if (img_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            depth_image_proc::convert<uint16_t>(img_msg, points_cam, camera_model);
        }
        else if (img_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            depth_image_proc::convert<float>(img_msg, points_cam, camera_model);
        }

        const std::string camera_frame = img_msg->header.frame_id;

        geometry_msgs::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer.lookupTransform(base_frame, camera_frame, ros::Time(0));
        }
        catch (const tf2::TransformException &e) {
            ROS_WARN_STREAM(e.what());
        }

        // points in link frame
        sensor_msgs::PointCloud2 points_lnk;
        tf2::doTransform<sensor_msgs::PointCloud2>(*points_cam.get(), points_lnk, tf_msg);
        pub_points.publish(points_lnk);
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image> ApproxSync;

    ros::NodeHandle n;
    ros::NodeHandle n_priv;
    image_transport::ImageTransport it;
    image_transport::SubscriberFilter sub_image;
    ros::Publisher pub_points;

    std::unique_ptr<message_filters::Synchronizer<ApproxSync>> sync;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf;

    std::string base_frame;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth_box_filter");

    BoxFilter filter;

    ros::spin();

    return EXIT_SUCCESS;
}
