#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  private:
    void initializeMap();

    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap();
    bool isValidGridCell(int x, int y);


    robot::CostmapCore costmap_;
  
    nav_msgs::msg::OccupancyGrid costmap_msg_;

    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    double resolution_;
    int width_;
    int height_;

    double inflation_radius_;
    int max_cost_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    std::vector<std::pair<int, int>> obstacle_cells_;
};
 
#endif 