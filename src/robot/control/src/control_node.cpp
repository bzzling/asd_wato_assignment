#include "control_node.hpp"

ControlNode::ControlNode()
: Node("control"), 
  control_(robot::ControlCore(this->get_logger()))
{
    declareParameters();

    // Create subscribers
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "path", 10, 
        std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

    // Create publisher
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create timer for control loop
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ControlNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Control node initialized");
}

void ControlNode::declareParameters()
{
    // Declare parameters with default values
    declare_parameter("lookahead_distance", 1.0);
    declare_parameter("goal_tolerance", 0.1);
    declare_parameter("linear_speed", 0.5);

    // Get parameter values
    lookahead_distance_ = get_parameter("lookahead_distance").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    linear_speed_ = get_parameter("linear_speed").as_double();
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    current_path_ = msg;
    RCLCPP_INFO(get_logger(), "Received new path with %zu poses", msg->poses.size());
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_odom_ = msg;
}

void ControlNode::controlLoop()
{
    // Skip if we don't have path or odometry data
    if (!current_path_ || !robot_odom_) {
        return;
    }

    // Find lookahead point
    auto lookahead_point = control_.findLookaheadPoint(
        current_path_, robot_odom_, lookahead_distance_);

    if (!lookahead_point) {
        RCLCPP_WARN(get_logger(), "No valid lookahead point found");
        return;
    }

    // Check if we've reached the goal
    if (control_.computeDistance(
        robot_odom_->pose.pose.position,
        current_path_->poses.back().pose.position) < goal_tolerance_)
    {
        RCLCPP_INFO(get_logger(), "Goal reached!");
        // Publish zero velocity
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    // Compute and publish velocity command
    auto cmd_vel = control_.computeVelocity(*lookahead_point, robot_odom_, linear_speed_);
    cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
