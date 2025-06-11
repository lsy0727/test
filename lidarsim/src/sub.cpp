#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include "/home/linux/ros2_ws/src/lidardrive/include/lidardrive/vision.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;
  
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
    cv::Mat gray, thresh, stats, centroids, l_frame, r_frame;
    std::vector<cv::vector<int>> index(3);
    std::vector<cv::Mat> p_frame(2);

    lidar_preprocess(frame, gray, thresh, l_frame, r_frame);

    // lidar_preprocess(frame, gray, thresh);
    // findObjects(thresh, stats, centroids, index);
    // drawObjects(frame, stats, centroids, index);
    cv::imshow("frame",frame);
    cv::imshow("gray", gray);
    cv::imshow("thresh", thresh);
    cv::waitKey(1);
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");   // 영상 sub 노드 : camsub_wsl
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",qos_profile,fn);   // 영상 sub 토픽 : image/compressed
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

