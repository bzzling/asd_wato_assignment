#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
    : logger_(logger),
      last_update_x_(0.0),
      last_update_y_(0.0),
      costmap_updated_(false),
      should_update_map_(false)
{
}

void MapMemoryCore::initializeParameters(
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
    double orientation_w)
{
    params_.costmap_topic = costmap_topic;
    params_.odom_topic = odom_topic;
    params_.map_topic = map_topic;
    params_.map_pub_rate = map_pub_rate;
    params_.update_distance = update_distance;
    params_.resolution = resolution;
    params_.width = width;
    params_.height = height;
    params_.origin_x = origin_x;
    params_.origin_y = origin_y;
    params_.orientation_w = orientation_w;

    initializeGlobalMap();
}

void MapMemoryCore::initializeGlobalMap()
{
    global_map_.header.frame_id = "map";
    global_map_.info.resolution = params_.resolution;
    global_map_.info.width = params_.width;
    global_map_.info.height = params_.height;
    global_map_.info.origin.position.x = params_.origin_x;
    global_map_.info.origin.position.y = params_.origin_y;
    global_map_.info.origin.orientation.w = params_.orientation_w;
    
    // Initialize all cells as unknown (-1)
    global_map_.data.resize(params_.width * params_.height, -1);
    
    RCLCPP_INFO(logger_, "Global map initialized with dimensions %dx%d and resolution %.2f",
                params_.width, params_.height, params_.resolution);
}

void MapMemoryCore::handleCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

void MapMemoryCore::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    double distance = calculateDistance(current_x, current_y, last_update_x_, last_update_y_);

    if (distance >= params_.update_distance) {
        last_update_x_ = current_x;
        last_update_y_ = current_y;
        should_update_map_ = true;
        RCLCPP_DEBUG(logger_, "Distance threshold reached: %.2f meters", distance);
    }
}

bool MapMemoryCore::shouldUpdateMap() const
{
    return should_update_map_ && costmap_updated_;
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const
{
    return global_map_;
}

void MapMemoryCore::updateMap()
{
    if (shouldUpdateMap()) {
        integrateCostmap();
        should_update_map_ = false;
        costmap_updated_ = false;
    }
}

void MapMemoryCore::integrateCostmap()
{
    if (global_map_.info.resolution != latest_costmap_.info.resolution) {
        RCLCPP_WARN(logger_, "Resolution mismatch between global and local maps");
        return;
    }

    // Linear fusion of costmaps
    for (size_t i = 0; i < latest_costmap_.data.size(); ++i) {
        if (latest_costmap_.data[i] >= 0) {  // If cell is known (not -1)
            global_map_.data[i] = latest_costmap_.data[i];
        }
    }
}

double MapMemoryCore::calculateDistance(double x1, double y1, double x2, double y2) const
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

}  // namespace robot