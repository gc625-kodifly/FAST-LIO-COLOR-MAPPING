#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

namespace camera_interface{

class ImagePublisher {
public:
    ImagePublisher() {
        // Initialize the node handle
        ros::NodeHandle nh;

        // Read parameters from the parameter server
        std::string rtsp_username, rtsp_password, rtsp_ip;
        nh.param<std::string>("rtsp_username", rtsp_username, "admin");
        nh.param<std::string>("rtsp_password", rtsp_password, "Kodifly2022");
        nh.param<std::string>("rtsp_ip", rtsp_ip, "192.168.1.107");

        // Create publisher
        publisher_ = nh.advertise<sensor_msgs::Image>("image_topic", 1);

        // Construct RTSP URL
        // std::string rtsp_url = "rtsp://" + rtsp_username + ":" + rtsp_password + "@" + rtsp_ip + ":554";
        // ROS_INFO("RTSP URL: %s", rtsp_url.c_str());
        std::string rtsp_url = "rtsp://admin:@192.168.1.10";
        // GStreamer pipeline
        std::string pipeline = "rtspsrc location=" + rtsp_url +
                               " latency=0 ! "
                               "queue ! "
                               "rtph264depay ! "
                               "h264parse ! "
                               "avdec_h264 ! "
                               "videoconvert ! "
                               "appsink";

        // Open video capture
        cap_.open(pipeline, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            ROS_ERROR("Failed to open the camera feed");
        }

        // Timer for publishing frames
        timer_ = nh.createTimer(ros::Duration(1.0 / 30), &ImagePublisher::publish_frame, this);
    }

    void publish_frame(const ros::TimerEvent&) {
        if (!cap_.isOpened()) {
            ROS_ERROR("Failed to open the camera feed");
            return;
        }

        cv::Mat frame;
        if (!cap_.read(frame)) {
            ROS_ERROR("Failed to read frame from the camera feed");
            return;
        }

        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = ros::Time::now();
        cv_image.encoding = "bgr8";
        cv_image.image = frame;

        sensor_msgs::ImagePtr msg = cv_image.toImageMsg();
        publisher_.publish(msg);
    }

private:
    ros::Publisher publisher_;
    ros::Timer timer_;
    cv::VideoCapture cap_;
};

} // namespace lrv_camera_interface

int main(int argc, char **argv) {
    ros::init(argc, argv, "ImagePublisher");
    camera_interface::ImagePublisher image_publisher;
    ros::spin();
    return 0;
}
