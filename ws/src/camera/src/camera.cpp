#include "camera/camera.hpp"

Camera::Camera(const rclcpp::NodeOptions &node_options)
: Node("camera", node_options) {
    threshold_ = this->declare_parameter<double>("threshold");
    max_ = this->declare_parameter<double>("max");

    camera_frame_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", rclcpp::SensorDataQoS().keep_last(5),
        std::bind(&Camera::callback_camera_frame, this, std::placeholders::_1));
}

void Camera::callback_camera_frame(const sensor_msgs::msg::Image::SharedPtr msg){
  while(1){ RCLCPP_DEBUG(this->get_logger(), "My log message %f", threshold_); }
}