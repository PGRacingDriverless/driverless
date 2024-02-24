#ifndef CAMERA_NODE_HPP_
#define CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class Camera : public rclcpp::Node {
    public:
        Camera(const rclcpp::NodeOptions &node_options);

        void callback_camera_frame(const sensor_msgs::msg::Image::SharedPtr msg);

    private:

        /** ROS topic subscribers */
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_frame_subscriber_;

        double threshold_;
        double max_;

};

#endif // CAMERA_NODE_HPP_