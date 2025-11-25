// Copyright 2025 SO-ARM Project
// Licensed under the Apache License, Version 2.0

/**
 * @file task_examples.cpp
 * @brief Examples demonstrating TaskManager usage
 */

#include <rclcpp/rclcpp.hpp>
#include "so_arm_commander_cpp/so_arm_commander.hpp"
#include "so_arm_tasks/task_manager.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

void printResult(const so_arm_tasks::TaskResult& result, const std::string& task_name)
{
    auto logger = rclcpp::get_logger("task_examples");

    if (result.success) {
        RCLCPP_INFO(logger, "[%s] ✓ SUCCESS: %s (%.2fs)",
                   task_name.c_str(), result.message.c_str(), result.execution_time);
    } else {
        RCLCPP_ERROR(logger, "[%s] ✗ FAILED: %s",
                    task_name.c_str(), result.message.c_str());
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto logger = rclcpp::get_logger("task_examples");

    RCLCPP_INFO(logger, "==============================================");
    RCLCPP_INFO(logger, "   SO-ARM Task Manager Examples");
    RCLCPP_INFO(logger, "==============================================");

    // Create executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // Create commander
    auto commander = std::make_shared<so_arm_commander::SoArmCommander>();
    executor.add_node(commander);

    // Spin in background
    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    std::this_thread::sleep_for(1s);

    // Initialize commander
    RCLCPP_INFO(logger, "Initializing SoArmCommander...");
    if (!commander->initialize()) {
        RCLCPP_ERROR(logger, "Failed to initialize SoArmCommander!");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    std::this_thread::sleep_for(2s);

    // Create task manager
    RCLCPP_INFO(logger, "Creating TaskManager...");
    auto task_manager = std::make_shared<so_arm_tasks::TaskManager>(commander);

    if (!task_manager->initialize()) {
        RCLCPP_ERROR(logger, "Failed to initialize TaskManager!");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    RCLCPP_INFO(logger, "\n");

    // ========================================
    // Example 1: Named Positions
    // ========================================
    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "Example 1: Named Positions");
    RCLCPP_INFO(logger, "========================================");

    // List available positions
    auto positions = task_manager->getNamedPositions();
    RCLCPP_INFO(logger, "Available named positions:");
    for (const auto& pos : positions) {
        RCLCPP_INFO(logger, "  - %s", pos.c_str());
    }

    // Move to home
    auto result = task_manager->moveToNamed("home");
    printResult(result, "Move to Home");
    std::this_thread::sleep_for(2s);

    // Move to ready
    result = task_manager->moveToNamed("ready");
    printResult(result, "Move to Ready");
    std::this_thread::sleep_for(2s);

    // Move to observe
    result = task_manager->moveToNamed("observe");
    printResult(result, "Move to Observe");
    std::this_thread::sleep_for(2s);

    // Return home
    result = task_manager->moveToNamed("home");
    printResult(result, "Return to Home");
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 2: Custom Named Position
    // ========================================
    RCLCPP_INFO(logger, "\n========================================");
    RCLCPP_INFO(logger, "Example 2: Register Custom Position");
    RCLCPP_INFO(logger, "========================================");

    std::vector<double> custom_pos = {0.5, -1.0, 1.2, -0.8, 0.3, 0.0};
    task_manager->registerNamedPosition("custom_demo", custom_pos);

    result = task_manager->moveToNamed("custom_demo");
    printResult(result, "Move to Custom");
    std::this_thread::sleep_for(2s);

    // Return home
    result = task_manager->moveToNamed("home");
    printResult(result, "Return to Home");
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 3: Waypoint Sequence
    // ========================================
    RCLCPP_INFO(logger, "\n========================================");
    RCLCPP_INFO(logger, "Example 3: Waypoint Sequence");
    RCLCPP_INFO(logger, "========================================");

    std::vector<so_arm_tasks::Waypoint> waypoints = {
        so_arm_tasks::Waypoint("start", {0.0, -0.5, 0.8, 0.3, 0.0, 0.0}, 0.3, 1.0),
        so_arm_tasks::Waypoint("mid1", {0.3, -0.7, 1.0, 0.0, 0.0, 0.0}, 0.4, 0.5),
        so_arm_tasks::Waypoint("mid2", {-0.3, -0.7, 1.0, 0.0, 0.0, 0.0}, 0.4, 0.5),
        so_arm_tasks::Waypoint("end", {0.0, -0.5, 0.8, 0.3, 0.0, 0.0}, 0.3, 0.0)
    };

    result = task_manager->executeWaypointSequence(waypoints, false, 2);
    printResult(result, "Waypoint Sequence");
    std::this_thread::sleep_for(1s);

    // Return home
    result = task_manager->moveToNamed("home");
    printResult(result, "Return to Home");
    std::this_thread::sleep_for(2s);

    // ========================================
    // Example 4: Square Pattern (Joint Space)
    // ========================================
    RCLCPP_INFO(logger, "\n========================================");
    RCLCPP_INFO(logger, "Example 4: Square Pattern");
    RCLCPP_INFO(logger, "========================================");

    // Note: Square pattern uses Cartesian space, which may fail
    // if the target is unreachable. This is for demonstration.
    geometry_msgs::msg::Point center;
    center.x = 0.20;
    center.y = 0.0;
    center.z = 0.0;  // Will be set by height parameter

    result = task_manager->moveSquarePattern(center, 0.08, 0.18);
    printResult(result, "Square Pattern");
    std::this_thread::sleep_for(1s);

    // Return home
    result = task_manager->moveToNamed("home");
    printResult(result, "Return to Home");

    // ========================================
    // Summary
    // ========================================
    RCLCPP_INFO(logger, "\n========================================");
    RCLCPP_INFO(logger, "All Examples Completed!");
    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "\nTaskManager Status: %s", task_manager->getStatus().c_str());

    // Cleanup
    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}
