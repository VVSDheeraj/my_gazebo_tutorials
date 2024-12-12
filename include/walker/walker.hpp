/**
 * @file walker.hpp
 * @brief Declaration of the Walker ROS2 node and states.
 * @author Sivaram Dheeraj Vishnubhotla
 * @version 1.0
 * @date 2024-11-19
 * @copyright Copyright (c) 2024
 * @license Apache 2.0
 */

#ifndef WALKER_HPP
#define WALKER_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// Forward declarations for state classes
class Walker;
class WalkerState;
class MovingForward;
class RotatingClockwise;
class RotatingCounterclockwise;

// ANSI color code macros for terminal output
#define GREEN "\033[0;32m"
#define YELLOW "\033[1;33m"
#define BLUE "\033[0;34m"
#define RED "\033[0;31m"
#define ORANGE "\033[38;5;214m"
#define NC "\033[0m"  // No Color

/**
 * @brief Base class for Walker states.
 * Defines the interface for the state machine.
 */
class WalkerState {
public:
    virtual ~WalkerState() = default;
    virtual void handleState(Walker& walker) = 0;
};

/**
 * @brief The Walker class represents the robot's state machine and actions.
 */
class Walker : public rclcpp::Node {
public:
    Walker();

    void setState(std::unique_ptr<WalkerState> new_state);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publishVelocity(float linear, float angular);

    // Getter methods for the private variables
    bool isObstacleDetected() const { return obstacle_detected_; }
    bool isRotateClockwise() const { return rotate_clockwise_; }
    void setRotateClockwise(bool clockwise) { rotate_clockwise_ = clockwise; }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    bool obstacle_detected_;
    bool rotate_clockwise_;
    float min_range_;

    std::unique_ptr<WalkerState> current_state_;
};

/**
 * @brief State for moving forward in the Walker state machine.
 */
class MovingForward : public WalkerState {
public:
    void handleState(Walker& walker) override;
};

/**
 * @brief State for rotating clockwise in the Walker state machine.
 */
class RotatingClockwise : public WalkerState {
public:
    void handleState(Walker& walker) override;
};

/**
 * @brief State for rotating counterclockwise in the Walker state machine.
 */
class RotatingCounterclockwise : public WalkerState {
public:
    void handleState(Walker& walker) override;
};

#endif  // WALKER_HPP
