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

  // A* Implementation with Dynamic Replanning and Collision Detection
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
  
  // Collision Detection and Recovery for Start Position
  if (current_map_.data[start_index] != 0) {
    RCLCPP_WARN(this->get_logger(), "Start position is occupied! Attempting recovery...");
    
    // Find nearest free cell for recovery
    bool found_free_cell = false;
    int search_radius = 1;
    int max_search_radius = 10; // Maximum search radius
    
    while (!found_free_cell && search_radius <= max_search_radius) {
      for (int dx = -search_radius; dx <= search_radius; dx++) {
        for (int dy = -search_radius; dy <= search_radius; dy++) {
          if (dx == 0 && dy == 0) continue;
          
          int new_x = start_x + dx;
          int new_y = start_y + dy;
          
          // Check bounds
          if (new_x < 0 || new_x >= static_cast<int>(current_map_.info.width) ||
              new_y < 0 || new_y >= static_cast<int>(current_map_.info.height)) {
            continue;
          }
          
          int new_index = new_y * current_map_.info.width + new_x;
          if (current_map_.data[new_index] == 0) {
            start_x = new_x;
            start_y = new_y;
            start = CellIndex(start_x, start_y);
            found_free_cell = true;
            RCLCPP_INFO(this->get_logger(), "Found free cell at (%d, %d) for recovery", start_x, start_y);
            break;
          }
        }
        if (found_free_cell) break;
      }
      if (!found_free_cell) search_radius++;
    }
    
    if (!found_free_cell) {
      RCLCPP_ERROR(this->get_logger(), "No free cell found for recovery! Robot is completely surrounded!");
      path_pub_->publish(path);
      return;
    }
  }
  
  if (current_map_.data[goal_index] != 0) {
    RCLCPP_WARN(this->get_logger(), "Goal position is occupied! Attempting goal relaxation...");
    
    // Find nearest free cell to goal
    bool found_free_goal = false;
    int search_radius = 1;
    int max_search_radius = 15; // Larger search radius for goal
    
    while (!found_free_goal && search_radius <= max_search_radius) {
      for (int dx = -search_radius; dx <= search_radius; dx++) {
        for (int dy = -search_radius; dy <= search_radius; dy++) {
          if (dx == 0 && dy == 0) continue;
          
          int new_x = goal_x + dx;
          int new_y = goal_y + dy;
          
          // Check bounds
          if (new_x < 0 || new_x >= static_cast<int>(current_map_.info.width) ||
              new_y < 0 || new_y >= static_cast<int>(current_map_.info.height)) {
            continue;
          }
          
          int new_index = new_y * current_map_.info.width + new_x;
          if (current_map_.data[new_index] == 0) {
            goal_x = new_x;
            goal_y = new_y;
            goal = CellIndex(goal_x, goal_y);
            found_free_goal = true;
            RCLCPP_INFO(this->get_logger(), "Relaxed goal to (%d, %d)", goal_x, goal_y);
            break;
          }
        }
        if (found_free_goal) break;
      }
      if (!found_free_goal) search_radius++;
    }
    
    if (!found_free_goal) {
      RCLCPP_ERROR(this->get_logger(), "No free cell found near goal! Goal is completely surrounded!");
      path_pub_->publish(path);
      return;
    }
  }
  
  // A* Algorithm Implementation with Dynamic Replanning
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
  
  // Dynamic replanning variables
  int max_iterations = 10000; // Prevent infinite loops
  int iteration_count = 0;
  int replan_attempts = 0;
  int max_replan_attempts = 3;
  
  while (!open_set.empty() && iteration_count < max_iterations) {
    iteration_count++;
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
    
    // Dynamic replanning: Check if current cell is now occupied (collision detection)
    int current_index = current.index.y * current_map_.info.width + current.index.x;
    if (current_map_.data[current_index] != 0) {
      RCLCPP_WARN(this->get_logger(), "Path blocked at (%d, %d)! Attempting dynamic replanning...", 
                  current.index.x, current.index.y);
      
      // Skip this node and continue searching
      continue;
    }
    
    // Explore 8-connected neighbors with enhanced collision detection
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        
        CellIndex neighbor(current.index.x + dx, current.index.y + dy);
        
        // Check if neighbor is within bounds
        if (neighbor.x < 0 || neighbor.x >= static_cast<int>(current_map_.info.width) ||
            neighbor.y < 0 || neighbor.y >= static_cast<int>(current_map_.info.height)) {
          continue;
        }
        
        // Enhanced collision detection with safety margin
        int neighbor_index = neighbor.y * current_map_.info.width + neighbor.x;
        if (current_map_.data[neighbor_index] != 0) {
          // Check surrounding cells for potential collision risk
          bool collision_risk = false;
          for (int check_dx = -1; check_dx <= 1; check_dx++) {
            for (int check_dy = -1; check_dy <= 1; check_dy++) {
              int check_x = neighbor.x + check_dx;
              int check_y = neighbor.y + check_dy;
              if (check_x >= 0 && check_x < static_cast<int>(current_map_.info.width) &&
                  check_y >= 0 && check_y < static_cast<int>(current_map_.info.height)) {
                int check_index = check_y * current_map_.info.width + check_x;
                if (current_map_.data[check_index] != 0) {
                  collision_risk = true;
                  break;
                }
              }
            }
            if (collision_risk) break;
          }
          
          if (collision_risk) {
            RCLCPP_DEBUG(this->get_logger(), "Skipping neighbor (%d, %d) due to collision risk", 
                        neighbor.x, neighbor.y);
            continue;
          }
        }
        
        // Calculate tentative g_score with dynamic cost adjustment
        double base_cost = std::sqrt((current.index.x - neighbor.x) * (current.index.x - neighbor.x) + 
                                    (current.index.y - neighbor.y) * (current.index.y - neighbor.y));
        
        // Add penalty for cells near obstacles (safety margin)
        double safety_penalty = 0.0;
        for (int check_dx = -1; check_dx <= 1; check_dx++) {
          for (int check_dy = -1; check_dy <= 1; check_dy++) {
            int check_x = neighbor.x + check_dx;
            int check_y = neighbor.y + check_dy;
            if (check_x >= 0 && check_x < static_cast<int>(current_map_.info.width) &&
                check_y >= 0 && check_y < static_cast<int>(current_map_.info.height)) {
              int check_index = check_y * current_map_.info.width + check_x;
              if (current_map_.data[check_index] != 0) {
                safety_penalty += 0.1; // Small penalty for being near obstacles
              }
            }
          }
        }
        
        double tentative_g_score = g_score[current.index] + base_cost + safety_penalty;
        
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
  
  // No path found - Enhanced error handling and recovery
  if (iteration_count >= max_iterations) {
    RCLCPP_ERROR(this->get_logger(), "A* search exceeded maximum iterations! Path may be too complex.");
  } else {
    RCLCPP_WARN(this->get_logger(), "No path found to goal! Goal may be unreachable.");
  }
  
  // Attempt recovery by trying to find a partial path to the nearest reachable point
  RCLCPP_INFO(this->get_logger(), "Attempting recovery by finding nearest reachable point...");
  
  // Find the best partial path (closest to goal that was reached)
  CellIndex best_reached = start;
  double best_distance_to_goal = std::sqrt((start.x - goal.x) * (start.x - goal.x) + 
                                          (start.y - goal.y) * (start.y - goal.y));
  
  for (const auto& pair : g_score) {
    if (pair.first != start) {
      double distance_to_goal = std::sqrt((pair.first.x - goal.x) * (pair.first.x - goal.x) + 
                                         (pair.first.y - goal.y) * (pair.first.y - goal.y));
      if (distance_to_goal < best_distance_to_goal) {
        best_reached = pair.first;
        best_distance_to_goal = distance_to_goal;
      }
    }
  }
  
  if (best_reached != start) {
    RCLCPP_INFO(this->get_logger(), "Found partial path to (%d, %d), distance to goal: %.2f", 
                best_reached.x, best_reached.y, best_distance_to_goal);
    
    // Reconstruct partial path
    CellIndex path_current = best_reached;
    while (path_current != start) {
      path_cells.push_back(path_current);
      auto it = came_from.find(path_current);
      if (it == came_from.end()) {
        RCLCPP_WARN(this->get_logger(), "Partial path reconstruction failed!");
        break;
      }
      path_current = it->second;
    }
    path_cells.push_back(start);
    std::reverse(path_cells.begin(), path_cells.end());
    
    // Convert partial path to ROS Path message
    for (const auto& cell : path_cells) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path.header;
      pose_stamped.pose.position.x = cell.x * current_map_.info.resolution + current_map_.info.origin.position.x;
      pose_stamped.pose.position.y = cell.y * current_map_.info.resolution + current_map_.info.origin.position.y;
      pose_stamped.pose.position.z = 0.0;
      pose_stamped.pose.orientation.w = 1.0;
      path.poses.push_back(pose_stamped);
    }
    
    RCLCPP_INFO(this->get_logger(), "Published partial path with %zu waypoints", path.poses.size());
  } else {
    RCLCPP_ERROR(this->get_logger(), "No partial path found! Robot may be completely trapped.");
  }

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
