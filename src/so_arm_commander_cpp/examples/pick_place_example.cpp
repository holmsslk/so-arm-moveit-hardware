// Copyright 2025 SO-ARM Project
// Licensed under the Apache License, Version 2.0

/**
 * @file pick_place_example.cpp
 * @brief Pick and place operation example for SO-ARM Commander
 *
 * This example demonstrates:
 * - Gripper control (open/close/position)
 * - Pick operation (grasp an object)
 * - Place operation (release an object)
 * - Complete pick-and-place sequence
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
    auto logger = rclcpp::get_logger("pick_place_example");

    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "SO-ARM Pick and Place Example");
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

    executor.add_node(commander);

    // Spin in a separate thread
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 1: Test Gripper Control
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 1] Testing gripper control...");

    RCLCPP_INFO(logger, "Opening gripper...");
    if (commander->openGripper()) {
        RCLCPP_INFO(logger, " Gripper opened");
        double pos = commander->getGripperPosition();
        RCLCPP_INFO(logger, "  Gripper position: %.2f%% open", pos * 100.0);
    } else {
        RCLCPP_ERROR(logger, " Failed to open gripper");
    }
    std::this_thread::sleep_for(2s);

    RCLCPP_INFO(logger, "Closing gripper...");
    if (commander->closeGripper()) {
        RCLCPP_INFO(logger, " Gripper closed");
        double pos = commander->getGripperPosition();
        RCLCPP_INFO(logger, "  Gripper position: %.2f%% open", pos * 100.0);
    } else {
        RCLCPP_ERROR(logger, " Failed to close gripper");
    }
    std::this_thread::sleep_for(2s);

    RCLCPP_INFO(logger, "Setting gripper to 50%%...");
    if (commander->setGripperPosition(0.5)) {
        RCLCPP_INFO(logger, " Gripper set to 50%%");
        double pos = commander->getGripperPosition();
        RCLCPP_INFO(logger, "  Gripper position: %.2f%% open", pos * 100.0);
    } else {
        RCLCPP_ERROR(logger, " Failed to set gripper position");
    }
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 2: Pick Operation
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 2] Performing pick operation...");

    // Move to ready position first
    RCLCPP_INFO(logger, "Moving to ready position...");
    commander->setNamedTarget("ready");
    std::this_thread::sleep_for(2s);

    // Define pick pose (object location)
    geometry_msgs::msg::Pose pick_pose;
    pick_pose.position.x = 0.25;
    pick_pose.position.y = 0.0;
    pick_pose.position.z = 0.05;  // 5cm above table

    // Orientation: pointing down
    pick_pose.orientation.x = 0.0;
    pick_pose.orientation.y = 0.707;
    pick_pose.orientation.z = 0.0;
    pick_pose.orientation.w = 0.707;

    RCLCPP_INFO(logger, "Pick pose: x=%.3f, y=%.3f, z=%.3f",
                pick_pose.position.x,
                pick_pose.position.y,
                pick_pose.position.z);

    RCLCPP_INFO(logger, "Executing pick sequence...");
    if (commander->pick(pick_pose, 0.10)) {  // Approach from 10cm above
        RCLCPP_INFO(logger, " Successfully picked object!");
    } else {
        RCLCPP_ERROR(logger, " Failed to pick object");
    }
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 3: Place Operation
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 3] Performing place operation...");

    // Define place pose (target location)
    geometry_msgs::msg::Pose place_pose;
    place_pose.position.x = 0.15;
    place_pose.position.y = 0.15;
    place_pose.position.z = 0.05;  // 5cm above table

    // Orientation: pointing down
    place_pose.orientation.x = 0.0;
    place_pose.orientation.y = 0.707;
    place_pose.orientation.z = 0.0;
    place_pose.orientation.w = 0.707;

    RCLCPP_INFO(logger, "Place pose: x=%.3f, y=%.3f, z=%.3f",
                place_pose.position.x,
                place_pose.position.y,
                place_pose.position.z);

    RCLCPP_INFO(logger, "Executing place sequence...");
    if (commander->place(place_pose, 0.10)) {  // Retreat 10cm after placing
        RCLCPP_INFO(logger, " Successfully placed object!");
    } else {
        RCLCPP_ERROR(logger, " Failed to place object");
    }
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 4: Complete Pick-and-Place
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 4] Performing complete pick-and-place sequence...");

    // Return to ready position
    RCLCPP_INFO(logger, "Moving to ready position...");
    commander->setNamedTarget("ready");
    std::this_thread::sleep_for(2s);

    // Define new pick and place locations
    geometry_msgs::msg::Pose pick_pose2;
    pick_pose2.position.x = 0.20;
    pick_pose2.position.y = -0.10;
    pick_pose2.position.z = 0.05;
    pick_pose2.orientation.x = 0.0;
    pick_pose2.orientation.y = 0.707;
    pick_pose2.orientation.z = 0.0;
    pick_pose2.orientation.w = 0.707;

    geometry_msgs::msg::Pose place_pose2;
    place_pose2.position.x = 0.20;
    place_pose2.position.y = 0.10;
    place_pose2.position.z = 0.05;
    place_pose2.orientation.x = 0.0;
    place_pose2.orientation.y = 0.707;
    place_pose2.orientation.z = 0.0;
    place_pose2.orientation.w = 0.707;

    RCLCPP_INFO(logger, "Pick: x=%.3f, y=%.3f, z=%.3f",
                pick_pose2.position.x,
                pick_pose2.position.y,
                pick_pose2.position.z);
    RCLCPP_INFO(logger, "Place: x=%.3f, y=%.3f, z=%.3f",
                place_pose2.position.x,
                place_pose2.position.y,
                place_pose2.position.z);

    RCLCPP_INFO(logger, "Executing complete pick-and-place...");
    if (commander->pickAndPlace(pick_pose2, place_pose2)) {
        RCLCPP_INFO(logger, " Successfully completed pick-and-place!");
    } else {
        RCLCPP_ERROR(logger, " Failed to complete pick-and-place");
    }
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 5: Return to Home
    // ========================================
    RCLCPP_INFO(logger, "\n[Example 5] Returning to home position...");
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
    RCLCPP_INFO(logger, "\nNote: This example assumes:");
    RCLCPP_INFO(logger, "  - Objects are placed at the specified positions");
    RCLCPP_INFO(logger, "  - The workspace is clear of obstacles");
    RCLCPP_INFO(logger, "  - The gripper is properly calibrated");
    RCLCPP_INFO(logger, "  - All positions are within safe workspace bounds");

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}
