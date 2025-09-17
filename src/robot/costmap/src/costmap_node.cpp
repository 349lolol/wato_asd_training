#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  this->declare_parameter("resolution", 0.1); //10cm
  this->declare_parameter("width", 100); //10 meters, 100 dots
  this->declare_parameter("height", 100); //10 meters, 100 dots

  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar",  // topic name from gazebo bridge
        10,        // QoS depth
        std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  initializeMap();
}

void CostmapNode::initializeMap() {
  costmap_msg_.header.frame_id = "map";
  costmap_msg_.info.resolution = resolution_;
  costmap_msg_.info.width = width_;
  costmap_msg_.info.height = height_;

  costmap_msg_.info.origin.position.x = -width_ * resolution_ / 2.0;
  costmap_msg_.info.origin.position.y = -height_ * resolution_ / 2.0;
  costmap_msg_.info.origin.position.z = 0.0;

  costmap_msg_.info.origin.orientation.x = 0.0;
  costmap_msg_.info.origin.orientation.y = 0.0;
  costmap_msg_.info.origin.orientation.z = 0.0;
  costmap_msg_.info.origin.orientation.w = 1.0;

  costmap_msg_.data.resize(width_ * height_, -1);
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