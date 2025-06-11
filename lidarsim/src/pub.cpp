#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <memory>
#include <chrono>

bool ctrl_c_pressed= false;
void ctrlc_handler(int) { ctrl_c_pressed= true; }

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("campub");   // 영상 pub 노드 : campub
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto mypub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);    // 영상 pub 토픽 : image/compressed
    
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
    rclcpp::WallRate loop_rate(10.0);

    // cv::VideoCapture cap(source, cv::CAP_GSTREAMER);
    cv::VideoCapture cap("/home/linux/ros2_ws/test.mp4");
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open video! Check if /dev/video0 exists and GStreamer plugins are installed.");
        rclcpp::shutdown();
        return -1;
    }
    cv::Mat frame;

    while (rclcpp::ok())
    {
        cap >> frame;
        if (frame.empty()) { RCLCPP_ERROR(node->get_logger(), "Frame empty!"); break;}
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
        mypub->publish(*msg);
        RCLCPP_INFO(node->get_logger(), "Published image: %dx%d", frame.cols, frame.rows);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}