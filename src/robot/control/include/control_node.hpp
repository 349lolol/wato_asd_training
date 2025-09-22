#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>
#include <optional>
#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist computeVel(const geometry_msgs::msg::PoseStamped& target);
    std::optional<geometry_msgs::msg::PoseStamped> getLookaheadPoint();

    double computeDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
    double extractYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

    static constexpr double LOOKAHEAD_DISTANCE = 0.6; // meters
    static constexpr double TOLERANCE = 0.1;
    static constexpr double LINEAR_VELOCITY = 0.5;
    static constexpr int dt_ms = 100; // Control loop interval in milliseconds

    void controlLoop();
    
};

#endif
