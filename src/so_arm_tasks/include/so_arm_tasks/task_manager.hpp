// Copyright 2025 SO-ARM Project
// Licensed under the Apache License, Version 2.0

/**
 * @file task_manager.hpp
 * @brief High-level task management for SO-ARM
 *
 * This class provides task-level abstractions on top of SoArmCommander,
 * making it easier to implement common robotic tasks like pick-and-place,
 * waypoint sequences, and predefined routines.
 */

#ifndef SO_ARM_TASKS__TASK_MANAGER_HPP_
#define SO_ARM_TASKS__TASK_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "so_arm_commander_cpp/so_arm_commander.hpp"
#include <memory>
#include <vector>
#include <string>
#include <map>

namespace so_arm_tasks
{

/**
 * @brief Waypoint structure for trajectory sequences
 */
struct Waypoint
{
    std::string name;
    std::vector<double> joint_positions;
    double velocity_scaling;
    double pause_duration;  // seconds to pause at this waypoint

    Waypoint(const std::string& n = "",
             const std::vector<double>& joints = {},
             double vel_scale = 0.5,
             double pause = 0.0)
        : name(n), joint_positions(joints),
          velocity_scaling(vel_scale), pause_duration(pause) {}
};

/**
 * @brief Task execution result
 */
struct TaskResult
{
    bool success;
    std::string message;
    double execution_time;  // seconds

    TaskResult(bool s = false, const std::string& m = "", double t = 0.0)
        : success(s), message(m), execution_time(t) {}
};

/**
 * @brief High-level task manager for SO-ARM
 */
class TaskManager
{
public:
    /**
     * @brief Constructor
     * @param commander Shared pointer to SoArmCommander instance
     */
    explicit TaskManager(std::shared_ptr<so_arm_commander::SoArmCommander> commander);

    /**
     * @brief Initialize the task manager
     * @return true if successful
     */
    bool initialize();

    // ========================================
    // Named Position Tasks
    // ========================================

    /**
     * @brief Move to a named position (home, ready, observe, etc.)
     * @param position_name Name of the target position
     * @return Task execution result
     */
    TaskResult moveToNamed(const std::string& position_name);

    /**
     * @brief Register a custom named position
     * @param name Position name
     * @param joint_positions Joint angles
     * @return true if successfully registered
     */
    bool registerNamedPosition(const std::string& name,
                              const std::vector<double>& joint_positions);

    /**
     * @brief Get all registered named positions
     * @return Vector of position names
     */
    std::vector<std::string> getNamedPositions() const;

    // ========================================
    // Waypoint Sequence Tasks
    // ========================================

    /**
     * @brief Execute a sequence of waypoints
     * @param waypoints Vector of waypoints to execute
     * @param loop If true, repeat the sequence
     * @param loop_count Number of loops (0 = infinite, requires manual stop)
     * @return Task execution result
     */
    TaskResult executeWaypointSequence(const std::vector<Waypoint>& waypoints,
                                      bool loop = false,
                                      int loop_count = 1);

    /**
     * @brief Stop ongoing waypoint sequence
     */
    void stopSequence();

    // ========================================
    // Pick and Place Tasks
    // ========================================

    /**
     * @brief Execute a pick operation
     * @param pick_pose Target pose for picking
     * @param approach_distance Distance to approach from above (meters)
     * @return Task execution result
     */
    TaskResult pickObject(const geometry_msgs::msg::Pose& pick_pose,
                         double approach_distance = 0.05);

    /**
     * @brief Execute a place operation
     * @param place_pose Target pose for placing
     * @param retreat_distance Distance to retreat after placing (meters)
     * @return Task execution result
     */
    TaskResult placeObject(const geometry_msgs::msg::Pose& place_pose,
                          double retreat_distance = 0.05);

    /**
     * @brief Execute complete pick-and-place task
     * @param pick_pose Where to pick the object
     * @param place_pose Where to place the object
     * @return Task execution result
     */
    TaskResult pickAndPlace(const geometry_msgs::msg::Pose& pick_pose,
                           const geometry_msgs::msg::Pose& place_pose);

    // ========================================
    // Pattern Tasks
    // ========================================

    /**
     * @brief Move in a square pattern (useful for testing/demo)
     * @param center Center point of the square
     * @param size Size of the square (meters)
     * @param height Height above the table (meters)
     * @return Task execution result
     */
    TaskResult moveSquarePattern(const geometry_msgs::msg::Point& center,
                                double size = 0.1,
                                double height = 0.15);

    /**
     * @brief Move in a circular pattern
     * @param center Center point of the circle
     * @param radius Radius of the circle (meters)
     * @param height Height above the table (meters)
     * @param num_points Number of points in the circle
     * @return Task execution result
     */
    TaskResult moveCirclePattern(const geometry_msgs::msg::Point& center,
                                double radius = 0.08,
                                double height = 0.15,
                                int num_points = 8);

    // ========================================
    // Configuration
    // ========================================

    /**
     * @brief Set default velocity scaling for tasks
     * @param scaling Velocity scaling factor (0.0 to 1.0)
     */
    void setDefaultVelocityScaling(double scaling);

    /**
     * @brief Set default acceleration scaling for tasks
     * @param scaling Acceleration scaling factor (0.0 to 1.0)
     */
    void setDefaultAccelerationScaling(double scaling);

    /**
     * @brief Set gripper positions
     * @param open_position Gripper open position
     * @param close_position Gripper close position
     */
    void setGripperPositions(double open_position, double close_position);

    /**
     * @brief Get current task status
     * @return Status string
     */
    std::string getStatus() const;

private:
    // SoArmCommander instance
    std::shared_ptr<so_arm_commander::SoArmCommander> commander_;

    // Named positions storage
    std::map<std::string, std::vector<double>> named_positions_;

    // Configuration
    double default_velocity_scaling_;
    double default_acceleration_scaling_;
    double gripper_open_position_;
    double gripper_close_position_;

    // Sequence control
    bool sequence_running_;
    bool stop_sequence_flag_;

    // Helper functions
    TaskResult createResult(bool success, const std::string& message, double time = 0.0);
    geometry_msgs::msg::Pose createApproachPose(const geometry_msgs::msg::Pose& target,
                                                 double distance);
    bool openGripper();
    bool closeGripper();

    // Logger
    rclcpp::Logger logger_;
};

}  // namespace so_arm_tasks

#endif  // SO_ARM_TASKS__TASK_MANAGER_HPP_
