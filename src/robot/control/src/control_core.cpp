#include "control_core.hpp"
#include <tf2/utils.h>
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path::SharedPtr& path,
    const nav_msgs::msg::Odometry::SharedPtr& odom,
    double lookahead_distance)
{
    if (path->poses.empty()) {
        RCLCPP_WARN(logger_, "Path is empty");
        return std::nullopt;
    }

    // Get robot's current position
    const auto& robot_pos = odom->pose.pose.position;

    // Find the closest point on the path that is at least lookahead_distance away
    for (const auto& pose : path->poses) {
        double distance = computeDistance(robot_pos, pose.pose.position);
        if (distance >= lookahead_distance) {
            return pose;
        }
    }

    // If no point is found, return the last point in the path
    return path->poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::PoseStamped& target,
    const nav_msgs::msg::Odometry::SharedPtr& odom,
    double linear_speed)
{
    geometry_msgs::msg::Twist cmd_vel;

    // Get current robot pose
    const auto& robot_pos = odom->pose.pose.position;
    double robot_yaw = extractYaw(odom->pose.pose.orientation);

    // Calculate angle to target
    double dx = target.pose.position.x - robot_pos.x;
    double dy = target.pose.position.y - robot_pos.y;
    double target_angle = std::atan2(dy, dx);

    // Calculate steering angle
    double steering_angle = target_angle - robot_yaw;

    // Normalize steering angle
    while (steering_angle > M_PI) steering_angle -= 2 * M_PI;
    while (steering_angle < -M_PI) steering_angle += 2 * M_PI;

    // Set linear and angular velocities
    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = 2.0 * linear_speed * std::sin(steering_angle) / lookahead_distance;

    return cmd_vel;
}

double ControlCore::computeDistance(
    const geometry_msgs::msg::Point& a, 
    const geometry_msgs::msg::Point& b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat)
{
    return tf2::getYaw(quat);
}

}  
