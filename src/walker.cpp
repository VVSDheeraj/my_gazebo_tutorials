/**
 * @file walker.cpp
 * @brief Implementation of the Walker ROS2 node and states.
 * @author Sivaram Dheeraj Vishnubhotla
 * @version 1.0
 * @date 2024-11-19
 * @copyright Copyright (c) 2024
 * @license Apache 2.0
 */

#include "walker/walker.hpp"

Walker::Walker()
    : Node("walker"),
      obstacle_detected_(false),
      rotate_clockwise_(true),
      min_range_(0.5) {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>
        ("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Walker::scanCallback,
            this, std::placeholders::_1));

    // Start in MovingForward state
    setState(std::make_unique<MovingForward>());
    RCLCPP_INFO(this->get_logger(), BLUE "Walker node is initialized." NC);
}

void Walker::setState(std::unique_ptr<WalkerState> new_state) {
    current_state_ = std::move(new_state);
    RCLCPP_INFO(this->get_logger(), ORANGE "State changed." NC);
}

void Walker::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    obstacle_detected_ = false;

    // Indices for the front of the robot for 40 degrees on either hemisphere
    int left_start_index = 0;
    int left_end_index = 40;
    int right_start_index = 320;
    int right_end_index = 359;

    auto check_obstacle = [&](int start, int end) {
        for (int i = start; i <= end; ++i) {
            if (msg->ranges[i] < min_range_) {
                obstacle_detected_ = true;
                RCLCPP_INFO(this->get_logger(), RED
                    "Obstacle detected in front. Range: %f at index %d" NC,
                        msg->ranges[i], i);
                return true;
            }
        }
        return false;
    };

    if (!check_obstacle(left_start_index, left_end_index)) {
    check_obstacle(right_start_index, right_end_index);
    }
    // Handle the state transition
    current_state_->handleState(*this);
}


void Walker::publishVelocity(float linear, float angular) {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_vel_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(),
        "Published velocity - Linear: %f, Angular: %f", linear, angular);
}

void MovingForward::handleState(Walker& walker) {
    if (walker.isObstacleDetected()) {
        if (walker.isRotateClockwise()) {
            walker.setState(std::make_unique<RotatingClockwise>());
        } else {
            walker.setState(std::make_unique<RotatingCounterclockwise>());
        }
    } else {
        walker.publishVelocity(0.2, 0.0);  // Move forward
    }
}

void RotatingClockwise::handleState(Walker& walker) {
    if (!walker.isObstacleDetected()) {
        RCLCPP_INFO(walker.get_logger(),
            "Obstacle cleared, switching to moving forward.");
        walker.setRotateClockwise(false);
        walker.setState(std::make_unique<MovingForward>());
    } else {
        walker.publishVelocity(0.0, -0.2);  // Rotate clockwise
        RCLCPP_INFO(walker.get_logger(), GREEN "Rotating clockwise." NC);
    }
}

void RotatingCounterclockwise::handleState(Walker& walker) {
    if (!walker.isObstacleDetected()) {
        RCLCPP_INFO(walker.get_logger(),
            "Obstacle cleared, switching to moving forward.");
        walker.setRotateClockwise(true);
        walker.setState(std::make_unique<MovingForward>());
    } else {
        walker.publishVelocity(0.0, 0.2);  // Rotate counterclockwise
        RCLCPP_INFO(walker.get_logger(), YELLOW "Rotating clockwise." NC);
    }
}

int main(int argc, char **argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create a node of type Walker
    auto walker_node = std::make_shared<Walker>();

    // Spin the node to process callbacks
    rclcpp::spin(walker_node);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
