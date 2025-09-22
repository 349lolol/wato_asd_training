#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

// C++ Standard Library
#include <chrono>        // For time operations
#include <memory>        // For smart pointers
#include <cmath>         // For trigonometric functions
#include <vector>        // For std::vector
#include <utility>       // For std::pair
#include <algorithm>     // For std::fill
#include <string>        // For std::string
#include <cstdint>      // For int8_t

// ROS2 Core
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/string.hpp"


// ROS2 Messages
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Local includes
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

    void publishMessage();

private:
    // Constants
    static constexpr int WIDTH = 300;
    static constexpr int HEIGHT = 300;
    static constexpr double RESOLUTION = 0.1;
    static constexpr int8_t MAX_COST = 100;
    static constexpr double INFLATION_RADIUS = 0.2;

    // Member functions
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void convertToGrid(double x, double y, int& x_grid, int& y_grid) const;
    void markObstacle(std::vector<std::vector<int8_t>>& costmap, int x_grid, int y_grid) const;
    void inflateObstacles(std::vector<std::vector<int8_t>>& costmap, int x_grid, int y_grid) const;
    bool isValidGridCell(int x, int y) const;
    void publishCostmap(const std::vector<std::vector<int8_t>>& costmap, 
    const std_msgs::msg::Header& header);

    // Member variables
    robot::CostmapCore costmap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};
 
#endif 