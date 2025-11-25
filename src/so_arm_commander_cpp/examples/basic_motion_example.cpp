// Copyright 2025 SO-ARM Project
// Licensed under the Apache License, Version 2.0

/**
 * @file basic_motion_example.cpp
 * @brief Basic motion control example for SO-ARM Commander
 *
 * This example demonstrates:
 * - Creating a SoArmCommander instance
 * - Moving to home position
 * - Moving to custom joint positions
 * - Moving to Cartesian poses
 * - Getting current joint and Cartesian positions
 */

#include <rclcpp/rclcpp.hpp>
#include "so_arm_commander_cpp/so_arm_commander.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create logger
    auto logger = rclcpp::get_logger("basic_motion_example");

    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "SO-ARM Basic Motion Control Example");
    RCLCPP_INFO(logger, "========================================");

    // Create executor first
    rclcpp::executors::SingleThreadedExecutor executor;

    // Create commander instance
    auto commander = std::make_shared<so_arm_commander::SoArmCommander>();
    executor.add_node(commander);

    // Spin in a separate thread
    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    // Wait for node to be ready
    std::this_thread::sleep_for(1s);

    // Initialize MoveIt (must be called after executor is spinning)
    if (!commander->initialize()) {
        RCLCPP_ERROR(logger, "Failed to initialize SoArmCommander!");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    // Wait for MoveIt initialization to complete
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 1: Move to Home Position
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 1] Moving to home position...");
    if (commander->goHome()) {
        RCLCPP_INFO(logger, " Successfully moved to home position");
    } else {
        RCLCPP_ERROR(logger, " Failed to move to home position");
    }
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 2: Get Current Joint Positions
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 2] Reading current joint positions...");
    auto current_joints = commander->getCurrentJointPositions();
    RCLCPP_INFO(logger, "Current joint positions:");
    for (size_t i = 0; i < current_joints.size(); ++i) {
        RCLCPP_INFO(logger, "  Joint %zu: %.3f rad (%.1f�)",
                    i, current_joints[i], current_joints[i] * 180.0 / M_PI);
    }

    // ========================================
    // Example 3: Move to Custom Joint Position
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 3] Moving to custom joint position...");
    std::vector<double> target_joints = {0.0, -0.5, 0.8, 0.3, 0.0, 0.0};
    RCLCPP_INFO(logger, "Target: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                target_joints[0], target_joints[1], target_joints[2],
                target_joints[3], target_joints[4], target_joints[5]);

    if (commander->moveToJointPosition(target_joints)) {
        RCLCPP_INFO(logger, " Successfully moved to target joint position");
    } else {
        RCLCPP_ERROR(logger, " Failed to move to target joint position");
    }
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 4: Get Current Cartesian Pose
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 4] Reading current Cartesian pose...");
    auto current_pose = commander->getCurrentPose();
    RCLCPP_INFO(logger, "Current pose:");
    RCLCPP_INFO(logger, "  Position: x=%.3f, y=%.3f, z=%.3f",
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z);
    RCLCPP_INFO(logger, "  Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w);

    // ========================================
    // Example 5: Move to Cartesian Pose (using Pose message)
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 5] Moving to Cartesian pose (Pose message)...");
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.25;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.20;

    // Orientation: identity quaternion (no rotation)
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    RCLCPP_INFO(logger, "Target pose: x=%.3f, y=%.3f, z=%.3f",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);

    if (commander->moveToPose(target_pose)) {
        RCLCPP_INFO(logger, " Successfully moved to target Cartesian pose");
    } else {
        RCLCPP_ERROR(logger, " Failed to move to target Cartesian pose");
    }
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 6: Move to Cartesian Pose (using coordinates)
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 6] Moving to Cartesian pose (coordinates)...");
    double x = 0.20;
    double y = 0.10;
    double z = 0.25;
    double roll = 0.0;
    double pitch = M_PI / 6;  // 30 degrees
    double yaw = 0.0;

    RCLCPP_INFO(logger, "Target: x=%.3f, y=%.3f, z=%.3f, roll=%.2f�, pitch=%.2f�, yaw=%.2f�",
                x, y, z,
                roll * 180.0 / M_PI,
                pitch * 180.0 / M_PI,
                yaw * 180.0 / M_PI);

    if (commander->moveToPose(x, y, z, roll, pitch, yaw)) {
        RCLCPP_INFO(logger, " Successfully moved to target Cartesian pose");
    } else {
        RCLCPP_ERROR(logger, " Failed to move to target Cartesian pose");
    }
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 7: Return to Home
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 7] Returning to home position...");
    if (commander->goHome()) {
        RCLCPP_INFO(logger, " Successfully returned to home position");
    } else {
        RCLCPP_ERROR(logger, " Failed to return to home position");
    }

    // ========================================
    // Cleanup
    // ========================================
    RCLCPP_INFO(logger, "\n========================================");
    RCLCPP_INFO(logger, "Example completed!");
    RCLCPP_INFO(logger, "========================================");

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}
