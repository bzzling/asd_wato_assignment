#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <optional>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit ControlCore(const rclcpp::Logger& logger);
    
    // Main control functions
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path::SharedPtr& path,
        const nav_msgs::msg::Odometry::SharedPtr& odom,
        double lookahead_distance);
    
    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::PoseStamped& target,
        const nav_msgs::msg::Odometry::SharedPtr& odom,
        double linear_speed);

    // Helper functions
    double computeDistance(const geometry_msgs::msg::Point& a, 
                         const geometry_msgs::msg::Point& b);
    double extractYaw(const geometry_msgs::msg::Quaternion& quat);

  private:
    rclcpp::Logger logger_;
};

} 

#endif 
