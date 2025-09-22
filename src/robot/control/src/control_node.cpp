#include "control_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {current_path_ = msg;});
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {current_odom_ = msg;});
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(dt_ms), [this]() { controlLoop(); });
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

double ControlNode::extractYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

void ControlNode::controlLoop() {
  if (!current_path_ || !current_odom_) {
    RCLCPP_WARN(this->get_logger(), "No path and odometry found. waiting ...");
    return;
  }

  double distance = computeDistance(current_odom_->pose.pose.position, current_path_->poses.back().pose.position);
  if (distance < TOLERANCE) {
    RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping the robot.");
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  // Compute control commands here
  auto lookahead_point_opt = getLookaheadPoint();
  if (!lookahead_point_opt.has_value()) {
    if (distance < lookahead_distance_) {
      lookahead_point_opt = current_path_->poses.back();
    }
    else {
      RCLCPP_WARN(this->get_logger(), "No lookahead point found. Stopping the robot.");
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }
  }
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
