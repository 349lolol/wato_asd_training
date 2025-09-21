#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timerCallback();
    void updateMap();

    robot::MapMemoryCore map_memory_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::OccupancyGrid global_map_;

    double robot_x_;
    double robot_y_;
    double theta_;
    double prev_x_;
    double prev_y_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr map_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    bool update_map_ = false;
    bool costmap_updated_ = false;

    //cons 
    static constexpr uint32_t MAP_PUB_RATE = 1000;    // Update rate in ms
    static constexpr double DIST_UPDATE = 1.5;         // Distance threshold for updates
    static constexpr double MAP_RES = 0.1;            // Map resolution (m/c
    static constexpr int MAP_WIDTH = 300;             // Map width in cells
    static constexpr int MAP_HEIGHT = 300;            // Map height in cells
    static constexpr double MAP_ORIGIN_X = -15.0;     // Map origin x (m)
    static constexpr double MAP_ORIGIN_Y = -15.0;     // Map origin y (m)
};

#endif 
