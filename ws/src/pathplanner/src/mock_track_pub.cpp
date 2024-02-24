#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "path_planner/Cones.h"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    cones.GenerateTestingMap();

    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("topic", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    while (marker_publisher_->get_subscription_count() < 1) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_WARN_ONCE(this->get_logger(),
                       "Please create a subscriber to the marker");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    static int x1 = 0, x2 = 0, y1 = 0, y2 = 0;

    if (count_ >= cones.GetNumCones(Cone::TrackSide::INNER)) return;

    x1 = cones.GetCone(Cone::TrackSide::INNER, count_).GetX();
    y1 = cones.GetCone(Cone::TrackSide::INNER, count_).GetY();

    x2 = cones.GetCone(Cone::TrackSide::OUTER, count_).GetX();
    y2 = cones.GetCone(Cone::TrackSide::OUTER, count_).GetY();

    // auto message = std_msgs::msg::String();
    // message.data = "{" + std::to_string(x1) + ", " + std::to_string(y1) + "},
    // " + "{" + std::to_string(x2) + ", " + std::to_string(y2) + "}";

    auto message = geometry_msgs::msg::Point();
    message.x = x1;
    message.y = y1;
    message.z = Cone::TrackSide::INNER;

    count_++;

    RCLCPP_INFO(this->get_logger(), "Publishing inner cone: '%f %f %f'",
                message.x, message.y, message.z);
    publisher_->publish(message);

    message.x = x2;
    message.y = y2;
    message.z = Cone::TrackSide::OUTER;

    RCLCPP_INFO(this->get_logger(), "Publishing outer cone: '%f %f %f'",
                message.x, message.y, message.z);
    publisher_->publish(message);

    // Create a marker for the inner cone
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = this->now();
    marker.ns = "points";
    marker.id = count_;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x1;
    marker.pose.position.y = y1;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    // Set the color of the inner cone to yellow
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Publish the marker for the inner cone
    marker_publisher_->publish(marker);

    // Update the marker for the outer cone
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = this->now();
    marker.ns = "points";
    marker.id = 200 - count_;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x2;
    marker.pose.position.y = y2;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    // Set the color of the outer cone to blue
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Publish the marker for the outer cone
    marker_publisher_->publish(marker);
  }

  Cones cones;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      marker_publisher_;

  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}