#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar",  // topic name from gazebo bridge
        10,        // QoS depth
        std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Process the incoming Lidar data
  RCLCPP_INFO(this->get_logger(), "Received Lidar scan with:");
  RCLCPP_INFO(this->get_logger(), "Angle range: [%f, %f]", msg->angle_min, msg->angle_max);
  RCLCPP_INFO(this->get_logger(), "Angular resolution: %f", msg->angle_increment);
  RCLCPP_INFO(this->get_logger(), "Number of ranges: %zu", msg->ranges.size());
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}