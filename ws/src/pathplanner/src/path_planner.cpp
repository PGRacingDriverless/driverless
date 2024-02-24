#include <boost/math/interpolators/barycentric_rational.hpp>
#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "path_planner/Cones.h"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

class PathPlanner : public rclcpp::Node {
 public:
  PathPlanner() : Node("path_planner") {
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        "topic", 10, std::bind(&PathPlanner::topic_callback, this, _1));
    path_publisher_ =
        this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
  }

 private:
  void topic_callback(const geometry_msgs::msg::Point& msg) {
    if (msg.z == Cone::TrackSide::INNER) {
      cones.AddCone(msg.x, msg.y, Cone::TrackSide::INNER);
    } else if (msg.z == Cone::TrackSide::OUTER) {
      cones.AddCone(msg.x, msg.y, Cone::TrackSide::OUTER);
    }

    if (cones.GetNumCones(Cone::TrackSide::INNER) == 20 &&
        cones.GetNumCones(Cone::TrackSide::OUTER) == 20) {
      cones.CalculateWayPoints(cones);

      // Create a vector of x and y coordinates from the waypoints
      std::vector<double> x_coords, y_coords;
      for (wayPoint point : cones.wayPoints) {
        x_coords.push_back(point.x);
        y_coords.push_back(point.y);
      }

      // Tworzenie wektora t (czasu lub indeksu punktu)
      std::vector<double> t_coords;
      for (size_t i = 0; i < x_coords.size(); ++i) {
        t_coords.push_back(i);
      }

      // Tworzenie interpolatorów dla x i y
      boost::math::barycentric_rational<double> x_interpolator(
          t_coords.data(), x_coords.data(), t_coords.size());
      boost::math::barycentric_rational<double> y_interpolator(
          t_coords.data(), y_coords.data(), t_coords.size());

      // Generowanie ścieżki
      double t_start = t_coords.front();
      double t_end = t_coords.back();
      double t_step = (t_end - t_start) /
                      1000;  // Dostosuj mianownik, aby zmienić liczbę punktów

      nav_msgs::msg::Path path;
      path.header.stamp = this->now();
      path.header.frame_id = "/my_frame";  // Replace with your frame_id

      for (double t = t_start; t <= t_end; t += t_step) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "/my_frame";  // Zastąp swoim frame_id
        pose.pose.position.x = x_interpolator(t);
        pose.pose.position.y = y_interpolator(t);
        pose.pose.orientation.w = 1.0;  // Zastąp swoją rzeczywistą orientacją
        path.poses.push_back(pose);
      }

      // Publish the path
      path_publisher_->publish(path);

      RCLCPP_INFO(this->get_logger(), "Waypoints:");
      for (wayPoint point : cones.wayPoints) {
        RCLCPP_INFO(this->get_logger(), "Waypoint: '%f %f %f'", point.x,
                    point.y, point.heading);
      }
    }
  }

  Cones cones;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanner>());
  rclcpp::shutdown();
  return 0;
}
