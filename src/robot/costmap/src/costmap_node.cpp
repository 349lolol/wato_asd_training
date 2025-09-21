#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
    // Declare parameters with default values matching our constants
    this->declare_parameter("inflation_radius", INFLATION_RADIUS);
    this->declare_parameter("obstacle_threshold", OBSTACLE_THRESHOLD);
    this->declare_parameter("max_cost", MAX_COST);
    this->declare_parameter("resolution", RESOLUTION);
    this->declare_parameter("width", WIDTH);
    this->declare_parameter("height", HEIGHT);

    // Create the costmap publisher
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    // Create the lidar subscription
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar",
        10,
        std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
}


void CostmapNode::convertToGrid(double x, double y, int& x_grid, int& y_grid) const {
    x_grid = static_cast<int>(x / RESOLUTION + WIDTH / 2);
    y_grid = static_cast<int>(y / RESOLUTION + HEIGHT / 2);
}

bool CostmapNode::isValidGridCell(int x, int y) const {
    return (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT);
}

void CostmapNode::markObstacle(std::vector<std::vector<int8_t>>& costmap, int x_grid, int y_grid) const {
    if (!isValidGridCell(x_grid, y_grid)) return;
    costmap[x_grid][y_grid] = MAX_COST;
}

void CostmapNode::inflateObstacles(std::vector<std::vector<int8_t>>& costmap, int x_grid, int y_grid) const {
    int inflation_cells = static_cast<int>(INFLATION_RADIUS / RESOLUTION);
    
    for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
        for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
            int new_x = x_grid + dx;
            int new_y = y_grid + dy;
            
            if (!isValidGridCell(new_x, new_y)) continue;
            
            double distance = std::hypot(dx * RESOLUTION, dy * RESOLUTION);
            if (distance > INFLATION_RADIUS) continue;
            
            int cost = static_cast<int>(MAX_COST * (1.0 - std::min(1.0, distance / INFLATION_RADIUS)));
            costmap[new_x][new_y] = std::max(static_cast<int>(costmap[new_x][new_y]), cost);
        }
    }
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<std::vector<int8_t>> costmap(WIDTH, std::vector<int8_t>(HEIGHT, 0));

    for (size_t i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < msg->range_min || 
            msg->ranges[i] > msg->range_max || 
            std::isnan(msg->ranges[i])) continue;

        double angle = msg->angle_min + i * msg->angle_increment;
        double x = msg->ranges[i] * std::cos(angle);
        double y = msg->ranges[i] * std::sin(angle);
        
        int x_grid, y_grid;
        convertToGrid(x, y, x_grid, y_grid);
        
        if (!isValidGridCell(x_grid, y_grid)) continue;

        if (msg->ranges[i] < OBSTACLE_THRESHOLD) {
            markObstacle(costmap, x_grid, y_grid);
            inflateObstacles(costmap, x_grid, y_grid);
        }
    }

    publishCostmap(costmap, msg->header);
}

void CostmapNode::publishCostmap(const std::vector<std::vector<int8_t>>& costmap,
                                const std_msgs::msg::Header& header) {
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header = header;
    occupancy_grid.info.resolution = RESOLUTION;
    occupancy_grid.info.width = WIDTH;
    occupancy_grid.info.height = HEIGHT;
    occupancy_grid.info.origin.position.x = -15;
    occupancy_grid.info.origin.position.y = -15;
    occupancy_grid.data.resize(WIDTH * HEIGHT);

    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            occupancy_grid.data[j * WIDTH + i] = costmap[j][i];
        }
    }

    costmap_pub_->publish(occupancy_grid);
}

 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}