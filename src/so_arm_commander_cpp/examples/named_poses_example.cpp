// Copyright 2025 SO-ARM Project
// Licensed under the Apache License, Version 2.0

/**
 * @file named_poses_example.cpp
 * @brief Named poses demonstration for SO-ARM Commander
 *
 * This example demonstrates:
 * - Using pre-defined named poses
 * - Creating custom named poses
 * - Listing available named poses
 * - Sequencing through multiple poses
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
    auto logger = rclcpp::get_logger("named_poses_example");

    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "SO-ARM Named Poses Example");
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

    // Wait for initialization
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 1: List Available Named Poses
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 1] Listing available named poses...");

    auto pose_names = commander->getNamedPosesList();
    RCLCPP_INFO(logger, "Available named poses (%zu total):", pose_names.size());
    for (const auto& name : pose_names) {
        RCLCPP_INFO(logger, "  - %s", name.c_str());
    }

    // ========================================
    // Example 2: Use Pre-defined Poses
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 2] Moving through pre-defined poses...");

    // Home position
    RCLCPP_INFO(logger, "\nMoving to 'home' position...");
    if (commander->setNamedTarget("home")) {
        RCLCPP_INFO(logger, " Reached 'home' position");
        auto joints = commander->getCurrentJointPositions();
        RCLCPP_INFO(logger, "  Joints: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
    } else {
        RCLCPP_ERROR(logger, " Failed to reach 'home' position");
    }
    std::this_thread::sleep_for(2s);

    // Ready position
    RCLCPP_INFO(logger, "\nMoving to 'ready' position...");
    if (commander->setNamedTarget("ready")) {
        RCLCPP_INFO(logger, " Reached 'ready' position");
        auto joints = commander->getCurrentJointPositions();
        RCLCPP_INFO(logger, "  Joints: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
    } else {
        RCLCPP_ERROR(logger, " Failed to reach 'ready' position");
    }
    std::this_thread::sleep_for(2s);

    // Observe position
    RCLCPP_INFO(logger, "\nMoving to 'observe' position...");
    if (commander->setNamedTarget("observe")) {
        RCLCPP_INFO(logger, " Reached 'observe' position");
        auto joints = commander->getCurrentJointPositions();
        RCLCPP_INFO(logger, "  Joints: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

        auto pose = commander->getCurrentPose();
        RCLCPP_INFO(logger, "  Cartesian: x=%.3f, y=%.3f, z=%.3f",
                    pose.position.x, pose.position.y, pose.position.z);
    } else {
        RCLCPP_ERROR(logger, " Failed to reach 'observe' position");
    }
    std::this_thread::sleep_for(2s);

    // Sleep position
    RCLCPP_INFO(logger, "\nMoving to 'sleep' position...");
    if (commander->setNamedTarget("sleep")) {
        RCLCPP_INFO(logger, " Reached 'sleep' position");
        auto joints = commander->getCurrentJointPositions();
        RCLCPP_INFO(logger, "  Joints: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
    } else {
        RCLCPP_ERROR(logger, " Failed to reach 'sleep' position");
    }
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 3: Create Custom Named Poses
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 3] Creating custom named poses...");

    // Return to home first
    commander->goHome();
    std::this_thread::sleep_for(2s);

    // Create "scan_left" pose
    std::vector<double> scan_left = {0.5, -0.3, 0.6, 0.2, 0.0, 0.0};
    RCLCPP_INFO(logger, "\nCreating 'scan_left' pose...");
    if (commander->addNamedPose("scan_left", scan_left)) {
        RCLCPP_INFO(logger, " 'scan_left' pose created");
    } else {
        RCLCPP_ERROR(logger, " Failed to create 'scan_left' pose");
    }

    // Create "scan_right" pose
    std::vector<double> scan_right = {-0.5, -0.3, 0.6, 0.2, 0.0, 0.0};
    RCLCPP_INFO(logger, "\nCreating 'scan_right' pose...");
    if (commander->addNamedPose("scan_right", scan_right)) {
        RCLCPP_INFO(logger, " 'scan_right' pose created");
    } else {
        RCLCPP_ERROR(logger, " Failed to create 'scan_right' pose");
    }

    // Create "scan_center" pose
    std::vector<double> scan_center = {0.0, -0.3, 0.6, 0.2, 0.0, 0.0};
    RCLCPP_INFO(logger, "\nCreating 'scan_center' pose...");
    if (commander->addNamedPose("scan_center", scan_center)) {
        RCLCPP_INFO(logger, " 'scan_center' pose created");
    } else {
        RCLCPP_ERROR(logger, " Failed to create 'scan_center' pose");
    }

    // List updated poses
    RCLCPP_INFO(logger, "\nUpdated list of named poses:");
    auto updated_pose_names = commander->getNamedPosesList();
    for (const auto& name : updated_pose_names) {
        RCLCPP_INFO(logger, "  - %s", name.c_str());
    }

    // ========================================
    // Example 4: Use Custom Named Poses
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 4] Using custom named poses...");

    // Move through scanning sequence
    RCLCPP_INFO(logger, "\nExecuting scanning sequence...");

    RCLCPP_INFO(logger, "Moving to 'scan_center'...");
    commander->setNamedTarget("scan_center");
    std::this_thread::sleep_for(2s);

    RCLCPP_INFO(logger, "Moving to 'scan_left'...");
    commander->setNamedTarget("scan_left");
    std::this_thread::sleep_for(2s);

    RCLCPP_INFO(logger, "Moving to 'scan_center'...");
    commander->setNamedTarget("scan_center");
    std::this_thread::sleep_for(2s);

    RCLCPP_INFO(logger, "Moving to 'scan_right'...");
    commander->setNamedTarget("scan_right");
    std::this_thread::sleep_for(2s);

    RCLCPP_INFO(logger, "Moving to 'scan_center'...");
    commander->setNamedTarget("scan_center");
    std::this_thread::sleep_for(2s);

    RCLCPP_INFO(logger, " Scanning sequence completed!");

    // ========================================
    // Example 5: Pose Sequence Demonstration
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 5] Running pose sequence demonstration...");

    std::vector<std::string> demo_sequence = {
        "home",
        "ready",
        "observe",
        "ready",
        "scan_left",
        "scan_center",
        "scan_right",
        "scan_center",
        "ready",
        "home"
    };

    RCLCPP_INFO(logger, "\nExecuting demonstration sequence:");
    for (size_t i = 0; i < demo_sequence.size(); ++i) {
        RCLCPP_INFO(logger, "[%zu/%zu] Moving to '%s'...",
                    i + 1, demo_sequence.size(), demo_sequence[i].c_str());

        if (commander->setNamedTarget(demo_sequence[i])) {
            RCLCPP_INFO(logger, "   Reached '%s'", demo_sequence[i].c_str());
        } else {
            RCLCPP_ERROR(logger, "   Failed to reach '%s'", demo_sequence[i].c_str());
        }

        std::this_thread::sleep_for(2s);
    }

    RCLCPP_INFO(logger, " Demonstration sequence completed!");

    // ========================================
    // Example 6: Return to Home
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 6] Returning to home position...");
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
    RCLCPP_INFO(logger, "\nPre-defined poses used:");
    RCLCPP_INFO(logger, "  - home: All joints at zero position");
    RCLCPP_INFO(logger, "  - ready: Standard ready-to-work position");
    RCLCPP_INFO(logger, "  - observe: High observation position");
    RCLCPP_INFO(logger, "  - sleep: Compact storage position");
    RCLCPP_INFO(logger, "\nCustom poses created:");
    RCLCPP_INFO(logger, "  - scan_left: Left scanning position");
    RCLCPP_INFO(logger, "  - scan_center: Center scanning position");
    RCLCPP_INFO(logger, "  - scan_right: Right scanning position");

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}
