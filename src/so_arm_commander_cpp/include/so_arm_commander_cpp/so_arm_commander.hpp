// Copyright 2025 SO-ARM Project
// Licensed under the Apache License, Version 2.0

#ifndef SO_ARM_COMMANDER_CPP__SO_ARM_COMMANDER_HPP_
#define SO_ARM_COMMANDER_CPP__SO_ARM_COMMANDER_HPP_

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Standard Library
#include <vector>
#include <string>
#include <map>
#include <memory>

namespace so_arm_commander
{

/**
 * @class SoArmCommander
 * @brief High-level interface for controlling SO-ARM robotic arm
 *
 * This class provides a simplified API for common robotic arm operations:
 * - Joint space motion control
 * - Cartesian space motion control
 * - Named pose targets
 * - Gripper control
 * - Pick and place operations
 *
 * Example usage:
 * @code
 * auto commander = std::make_shared<SoArmCommander>();
 * commander->goHome();
 * commander->moveToPose(0.2, 0.1, 0.3, 0, M_PI/4, 0);
 * @endcode
 */
class SoArmCommander : public rclcpp::Node
{
public:
    // ========================================
    // Constructor and Destructor
    // ========================================

    /**
     * @brief Constructor
     * @param options ROS2 node options
     */
    explicit SoArmCommander(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Destructor
     */
    ~SoArmCommander();

    /**
     * @brief Initialize MoveIt interfaces (must be called after construction)
     * @return true if successful
     */
    bool initialize();

    // ========================================
    // Basic Motion Control
    // ========================================

    /**
     * @brief Move to home (zero) position
     * @return true if successful
     */
    bool goHome();

    /**
     * @brief Move to specified joint positions
     * @param positions Vector of 6 joint angles in radians
     * @return true if successful
     */
    bool moveToJointPosition(const std::vector<double>& positions);

    /**
     * @brief Get current joint positions
     * @return Vector of current joint angles
     */
    std::vector<double> getCurrentJointPositions();

    // ========================================
    // Cartesian Space Motion
    // ========================================

    /**
     * @brief Move to specified Cartesian pose
     * @param target Target pose
     * @return true if successful
     */
    bool moveToPose(const geometry_msgs::msg::Pose& target);

    /**
     * @brief Move to specified Cartesian coordinates (convenience function)
     * @param x, y, z Position in meters
     * @param roll, pitch, yaw Orientation in radians
     * @return true if successful
     */
    bool moveToPose(double x, double y, double z,
                    double roll, double pitch, double yaw);

    /**
     * @brief Execute Cartesian linear motion
     * @param target Target pose
     * @param velocity_scaling Velocity scaling factor (0.0-1.0)
     * @return true if successful
     */
    bool moveLinear(const geometry_msgs::msg::Pose& target,
                    double velocity_scaling = 0.5);

    /**
     * @brief Get current end-effector pose
     * @return Current pose
     */
    geometry_msgs::msg::Pose getCurrentPose();

    // ========================================
    // Named Poses
    // ========================================

    /**
     * @brief Move to a named target pose
     * @param target_name Name of the target pose (e.g., "home", "ready", "observe")
     * @return true if successful
     */
    bool setNamedTarget(const std::string& target_name);

    /**
     * @brief Add a new named pose
     * @param name Name for the pose
     * @param positions Joint positions
     * @return true if successful
     */
    bool addNamedPose(const std::string& name,
                      const std::vector<double>& positions);

    /**
     * @brief Get list of all available named poses
     * @return Vector of pose names
     */
    std::vector<std::string> getNamedPosesList();

    // ========================================
    // Gripper Control
    // ========================================

    /**
     * @brief Open the gripper
     * @return true if successful
     */
    bool openGripper();

    /**
     * @brief Close the gripper
     * @return true if successful
     */
    bool closeGripper();

    /**
     * @brief Set gripper position
     * @param position Gripper opening (0.0=closed, 1.0=fully open)
     * @return true if successful
     */
    bool setGripperPosition(double position);

    /**
     * @brief Get current gripper position
     * @return Gripper opening (0.0-1.0)
     */
    double getGripperPosition();

    // ========================================
    // High-Level Operations
    // ========================================

    /**
     * @brief Pick an object
     * @param target Grasp pose
     * @param approach_distance Distance to approach from above (meters)
     * @return true if successful
     */
    bool pick(const geometry_msgs::msg::Pose& target,
              double approach_distance = 0.10);

    /**
     * @brief Place an object
     * @param target Place pose
     * @param retreat_distance Distance to retreat after placing (meters)
     * @return true if successful
     */
    bool place(const geometry_msgs::msg::Pose& target,
               double retreat_distance = 0.10);

    /**
     * @brief Execute pick and place sequence
     * @param pick_pose Grasp pose
     * @param place_pose Place pose
     * @return true if successful
     */
    bool pickAndPlace(const geometry_msgs::msg::Pose& pick_pose,
                      const geometry_msgs::msg::Pose& place_pose);

    // ========================================
    // Configuration
    // ========================================

    /**
     * @brief Set planning time limit
     * @param seconds Planning time in seconds
     */
    void setPlanningTime(double seconds);

    /**
     * @brief Set velocity scaling factor
     * @param factor Scaling factor (0.0-1.0)
     */
    void setVelocityScaling(double factor);

    /**
     * @brief Set acceleration scaling factor
     * @param factor Scaling factor (0.0-1.0)
     */
    void setAccelerationScaling(double factor);

    /**
     * @brief Set workspace bounds for safety
     * @param min_x, max_x, min_y, max_y, min_z, max_z Bounds in meters
     */
    void setWorkspaceBounds(double min_x, double max_x,
                            double min_y, double max_y,
                            double min_z, double max_z);

    /**
     * @brief Check if position is within safe workspace
     * @param x, y, z Position coordinates
     * @return true if position is safe
     */
    bool isInWorkspace(double x, double y, double z);

private:
    // ========================================
    // MoveIt Interfaces
    // ========================================
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

    // ========================================
    // Configuration Parameters
    // ========================================
    std::string planning_group_;           ///< Planning group name (default: "arm")
    double planning_time_;                 ///< Planning time limit in seconds
    double velocity_scaling_factor_;       ///< Velocity scaling factor
    double acceleration_scaling_factor_;   ///< Acceleration scaling factor

    // ========================================
    // Named Poses
    // ========================================
    std::map<std::string, std::vector<double>> named_poses_;  ///< Named pose storage

    // ========================================
    // Gripper Parameters
    // ========================================
    double gripper_open_position_;         ///< Gripper fully open position
    double gripper_closed_position_;       ///< Gripper closed position
    size_t gripper_joint_index_;           ///< Index of gripper joint (typically 5)

    // ========================================
    // Workspace Limits
    // ========================================
    double workspace_min_x_, workspace_max_x_;
    double workspace_min_y_, workspace_max_y_;
    double workspace_min_z_, workspace_max_z_;

    // ========================================
    // Helper Functions
    // ========================================

    /**
     * @brief Initialize MoveIt interfaces
     * @return true if successful
     */
    bool initializeMoveIt();

    /**
     * @brief Load named poses from parameter server or config file
     * @return true if successful
     */
    bool loadNamedPoses();

    /**
     * @brief Validate joint positions
     * @param positions Joint positions to validate
     * @return true if valid
     */
    bool validateJointPositions(const std::vector<double>& positions);

    /**
     * @brief Create Pose message from position and orientation
     * @param x, y, z Position
     * @param roll, pitch, yaw Orientation (Euler angles)
     * @return Pose message
     */
    geometry_msgs::msg::Pose createPose(double x, double y, double z,
                                         double roll, double pitch, double yaw);

    /**
     * @brief Plan and execute motion
     * @return true if successful
     */
    bool planAndExecute();

    /**
     * @brief Wait for motion execution to complete
     * @param timeout Timeout in seconds
     * @return true if completed successfully
     */
    bool waitForExecution(double timeout = 10.0);
};

}  // namespace so_arm_commander

#endif  // SO_ARM_COMMANDER_CPP__SO_ARM_COMMANDER_HPP_
