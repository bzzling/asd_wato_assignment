#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
 

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    costmap_data_.resize(width * height); // 400 x 400 grid represented as 1-D array
    std::fill(costmap_data_.begin(), costmap_data_.end(), 0);
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles();
 
    // Step 4: Publish costmap
    publishCostmap();
}

void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  double x = range * cos(angle);
  double y = range * sin(angle);

  x_grid = static_cast<int>(x / resolution + width / 2);
  y_grid = static_cast<int>(y / resolution + height / 2);
}

void CostmapNode::markObstacle(int x, int y) {
  if (x < 0 || x >= width || y < 0 || y >= height) {
    return;
  }
  costmap_data_[y * width + x] = obstacle_cost;
}

void CostmapNode::inflateObstacles() {
    std::vector<int8_t> inflated_map = costmap_data_;
    
    int inflation_cells = static_cast<int>(inflation_radius / resolution);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (costmap_data_[y * width + x] == obstacle_cost) {
                for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
                        int new_x = x + dx;
                        int new_y = y + dy;
                        
                        if (new_x < 0 || new_x >= width || new_y < 0 || new_y >= height) {
                            continue;
                        }

                        double distance = std::sqrt(dx*dx + dy*dy) * resolution;
                        
                        int cost = static_cast<int>(inflation_cost * (1.0 - distance/inflation_radius));
                        
                        int idx = new_y * width + new_x;
                        if (distance <= inflation_radius && cost > inflated_map[idx]) {
                            inflated_map[idx] = cost;
                        }
                    }
                }
            }
        }
    }
    
    costmap_data_ = inflated_map;
}

void CostmapNode::publishCostmap() {
  auto msg = nav_msgs::msg::OccupancyGrid();
  msg.header.stamp = this->now();
  msg.header.frame_id = "robot/chassis/lidar";

  msg.info.resolution = resolution;
  msg.info.width = width;
  msg.info.height = height;
  msg.info.origin.position.x = -width * resolution / 2;
  msg.info.origin.position.y = -height * resolution / 2;
  msg.info.origin.position.z = 0;
  msg.info.origin.orientation.w = 1;

  msg.data = costmap_data_;

  occupancy_grid_pub_->publish(msg);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}