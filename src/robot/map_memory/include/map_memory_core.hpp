#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <memory>

namespace robot
{

class MapMemoryCore {
public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    
    // Main processing functions
    void handleCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool shouldUpdateMap() const;
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;
    void updateMap();
    void initializeParameters(
        const std::string& costmap_topic,
        const std::string& odom_topic,
        const std::string& map_topic,
        int map_pub_rate,
        double update_distance,
        double resolution,
        int width,
        int height,
        double origin_x,
        double origin_y,
        double orientation_w);

private:
    rclcpp::Logger logger_;
    
    // Helper functions
    void integrateCostmap();
    double calculateDistance(double x1, double y1, double x2, double y2) const;
    void initializeGlobalMap();

    // State variables
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    double last_update_x_;
    double last_update_y_;
    bool costmap_updated_;
    bool should_update_map_;

    // Parameters
    struct Parameters {
        std::string costmap_topic;
        std::string odom_topic;
        std::string map_topic;
        int map_pub_rate;
        double update_distance;
        double resolution;
        int width;
        int height;
        double origin_x;
        double origin_y;
        double orientation_w;
    } params_;
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_