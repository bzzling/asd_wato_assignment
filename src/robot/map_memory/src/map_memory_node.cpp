#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      map_memory_(robot::MapMemoryCore(this->get_logger()))
{
    declareParameters();
    initializeInterfaces();
}

void MapMemoryNode::declareParameters()
{
    // Declare all parameters with default values
    declare_parameter("local_costmap_topic", "/costmap");
    declare_parameter("odom_topic", "/odom/filtered");
    declare_parameter("map_topic", "/map");
    declare_parameter("map_pub_rate", 3000);
    declare_parameter("update_distance", 1.5);
    declare_parameter("global_map.resolution", 0.5);
    declare_parameter("global_map.width", 60);
    declare_parameter("global_map.height", 60);
    declare_parameter("global_map.origin.position.x", -15.0);
    declare_parameter("global_map.origin.position.y", -15.0);
    declare_parameter("global_map.origin.orientation.w", 1.0);

    // Initialize core with parameters
    map_memory_.initializeParameters(
        get_parameter("local_costmap_topic").as_string(),
        get_parameter("odom_topic").as_string(),
        get_parameter("map_topic").as_string(),
        get_parameter("map_pub_rate").as_int(),
        get_parameter("update_distance").as_double(),
        get_parameter("global_map.resolution").as_double(),
        get_parameter("global_map.width").as_int(),
        get_parameter("global_map.height").as_int(),
        get_parameter("global_map.origin.position.x").as_double(),
        get_parameter("global_map.origin.position.y").as_double(),
        get_parameter("global_map.origin.orientation.w").as_double()
    );
}

void MapMemoryNode::initializeInterfaces()
{
    // Create subscribers
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        get_parameter("local_costmap_topic").as_string(),
        10,
        std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("odom_topic").as_string(),
        10,
        std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    // Create publisher
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        get_parameter("map_topic").as_string(),
        10);

    // Create timer
    timer_ = create_wall_timer(
        std::chrono::milliseconds(get_parameter("map_pub_rate").as_int()),
        std::bind(&MapMemoryNode::timerCallback, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_memory_.handleCostmap(msg);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    map_memory_.handleOdometry(msg);
}

void MapMemoryNode::timerCallback()
{
    if (map_memory_.shouldUpdateMap()) {
        map_memory_.updateMap();
        map_pub_->publish(map_memory_.getGlobalMap());
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}