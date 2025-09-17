#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  this->declare_parameter("resolution", 0.1); //10cm
  this->declare_parameter("width", 100); //10 meters, 100 dots
  this->declare_parameter("height", 100); //10 meters, 100 dots

  inflation_radius_ = this->get_parameter("inflation_radius").as_double(); // 50cm
  max_cost_ = this->get_parameter("max_cost").as_int(); // 255

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

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

void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  x_grid = static_cast<int>((x - costmap_msg_.info.origin.position.x) / resolution_);
  y_grid = static_cast<int>((y - costmap_msg_.info.origin.position.y) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  if(!isValidGridCell(x_grid, y_grid)) return;

  int index = y_grid * width_ + x_grid; //2d dimensions to 1d array
  costmap_msg_.data[index] = 100;
  obstacle_cells_.push_back({x_grid, y_grid});
}

bool CostmapNode::isValidGridCell(int x, int y) {
  return (x >= 0 && x < width_ && y >= 0 && y < height_);
}

void CostmapNode::inflateObstacles() {
  int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

  for(const auto& obstacle : obstacle_cells_) {
    int ox = obstacle.first;
    int oy = obstacle.second;

    for(int dx = -inflation_cells; dx <= inflation_cells; dx++) {
      for(int dy = -inflation_cells; dy <= inflation_cells; dy++) {
        int x = ox + dx;
        int y = oy + dy;
        if(!isValidGridCell(x, y)) {
          continue;
        } 

        double distance = std::hypot(dx * resolution_, dy * resolution_);
        if(distance > inflation_radius_) {
          continue;
        }
        int cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));
        int index = y * width_ + x;
        if(cost > costmap_msg_.data[index]) {
          costmap_msg_.data[index] = cost;
        }
      }
    }
  }
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Clear previous obstacles
  obstacle_cells_.clear();
  std::fill(costmap_msg_.data.begin(), costmap_msg_.data.end(), 0);
  //process scanned data
  for(size_t i = 0; i < msg->ranges.size(); i++) {
    double angle = msg->angle_min + i * msg->angle_increment;
    double range = msg->ranges[i];

    if(std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  inflateObstacles();
  publishCostmap();
}

void CostmapNode::publishCostmap() {
  costmap_msg_.header.stamp = this->now();
  costmap_pub_->publish(costmap_msg_);
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