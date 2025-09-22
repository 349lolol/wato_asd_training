#include "planner_node.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
 
  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this)); 

  state_ = State::WAITING_FOR_GOAL;

  current_map_ = nav_msgs::msg::OccupancyGrid();
  goal_ = geometry_msgs::msg::PointStamped();
  robot_pose_ = geometry_msgs::msg::Pose();
}


void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
      if (goalReached()) {
          RCLCPP_INFO(this->get_logger(), "Goal reached!");
          state_ = State::WAITING_FOR_GOAL;
      } else {
          RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
          planPath();
      }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
      return;
  }

  // A* Implementation
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "sim_world";
  std::vector<CellIndex> path_cells;

  // Convert start and goal positions to grid coordinates
  int start_x = static_cast<int>((robot_pose_.position.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int start_y = static_cast<int>((robot_pose_.position.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  int goal_x = static_cast<int>((goal_.point.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int goal_y = static_cast<int>((goal_.point.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  
  CellIndex start(start_x, start_y);
  CellIndex goal(goal_x, goal_y);
  
  // Validate start and goal positions
  if (start_x < 0 || start_x >= static_cast<int>(current_map_.info.width) ||
      start_y < 0 || start_y >= static_cast<int>(current_map_.info.height)) {
    RCLCPP_WARN(this->get_logger(), "Start position is outside map bounds!");
    path_pub_->publish(path);
    return;
  }
  
  if (goal_x < 0 || goal_x >= static_cast<int>(current_map_.info.width) ||
      goal_y < 0 || goal_y >= static_cast<int>(current_map_.info.height)) {
    RCLCPP_WARN(this->get_logger(), "Goal position is outside map bounds!");
    path_pub_->publish(path);
    return;
  }
  
  // Check if start and goal are not occupied
  int start_index = start_y * current_map_.info.width + start_x;
  int goal_index = goal_y * current_map_.info.width + goal_x;
  
  if (current_map_.data[start_index] != 0) {
    RCLCPP_WARN(this->get_logger(), "Start position is occupied!");
    path_pub_->publish(path);
    return;
  }
  
  if (current_map_.data[goal_index] != 0) {
    RCLCPP_WARN(this->get_logger(), "Goal position is occupied!");
    path_pub_->publish(path);
    return;
  }
  
  // A* Algorithm Implementation
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, double, CellIndexHash> f_score;
  std::unordered_map<CellIndex, bool, CellIndexHash> in_open_set;
  
  // Initialize scores
  g_score[start] = 0.0;
  f_score[start] = std::sqrt((start.x - goal.x) * (start.x - goal.x) + (start.y - goal.y) * (start.y - goal.y));
  open_set.push(AStarNode(start, f_score[start]));
  in_open_set[start] = true;
  
  RCLCPP_INFO(this->get_logger(), "Starting A* pathfinding from (%d, %d) to (%d, %d)", 
              start.x, start.y, goal.x, goal.y);
  
  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();
    in_open_set[current.index] = false;
    
    // Check if we reached the goal
    if (current.index == goal) {
      RCLCPP_INFO(this->get_logger(), "Path found!");
      
      // Reconstruct path
      CellIndex path_current = goal;
      
      while (path_current != start) {
        path_cells.push_back(path_current);
        auto it = came_from.find(path_current);
        if (it == came_from.end()) {
          RCLCPP_WARN(this->get_logger(), "Path reconstruction failed!");
          path_pub_->publish(path);
          return;
        }
        path_current = it->second;
      }
      path_cells.push_back(start);
      std::reverse(path_cells.begin(), path_cells.end());
      
      // Convert path to ROS Path message
      for (const auto& cell : path_cells) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        pose_stamped.pose.position.x = cell.x * current_map_.info.resolution + current_map_.info.origin.position.x;
        pose_stamped.pose.position.y = cell.y * current_map_.info.resolution + current_map_.info.origin.position.y;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
        path.poses.push_back(pose_stamped);
      }
      
      RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path.poses.size());
      path_pub_->publish(path);
      return;
    }
    
    // Explore 8-connected neighbors
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        
        CellIndex neighbor(current.index.x + dx, current.index.y + dy);
        
        // Check if neighbor is within bounds
        if (neighbor.x < 0 || neighbor.x >= static_cast<int>(current_map_.info.width) ||
            neighbor.y < 0 || neighbor.y >= static_cast<int>(current_map_.info.height)) {
          continue;
        }
        
        // Check if neighbor is not occupied
        int neighbor_index = neighbor.y * current_map_.info.width + neighbor.x;
        if (current_map_.data[neighbor_index] != 0) {
          continue;
        }
        
        // Calculate tentative g_score
        double tentative_g_score = g_score[current.index] + 
          std::sqrt((current.index.x - neighbor.x) * (current.index.x - neighbor.x) + 
                   (current.index.y - neighbor.y) * (current.index.y - neighbor.y));
        
        // Update if this path to neighbor is better
        if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
          came_from[neighbor] = current.index;
          g_score[neighbor] = tentative_g_score;
          f_score[neighbor] = tentative_g_score + 
            std::sqrt((neighbor.x - goal.x) * (neighbor.x - goal.x) + 
                     (neighbor.y - goal.y) * (neighbor.y - goal.y));
          
          if (!in_open_set[neighbor]) {
            open_set.push(AStarNode(neighbor, f_score[neighbor]));
            in_open_set[neighbor] = true;
          }
        }
      }
    }
  }
  
  // No path found
  RCLCPP_WARN(this->get_logger(), "No path found to goal!");

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
