#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
 
  private:
    const double resolution = 0.1;
    const int width = 200;
    const int height = 200;
    const int obstacle_cost = 100;
    const double inflation_radius = 1.0;
    const int inflation_cost = 75;

    robot::CostmapCore costmap_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x, int y);
    void inflateObstacles();
    void publishCostmap();

    std::vector<int8_t> costmap_data_;
};
 
#endif 