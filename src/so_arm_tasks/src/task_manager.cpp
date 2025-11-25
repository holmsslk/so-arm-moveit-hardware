// Copyright 2025 SO-ARM Project
// Licensed under the Apache License, Version 2.0

#include "so_arm_tasks/task_manager.hpp"
#include <cmath>
#include <chrono>
#include <thread>

namespace so_arm_tasks
{

TaskManager::TaskManager(std::shared_ptr<so_arm_commander::SoArmCommander> commander)
    : commander_(commander)
    , default_velocity_scaling_(0.5)
    , default_acceleration_scaling_(0.5)
    , gripper_open_position_(1.0)
    , gripper_close_position_(0.0)
    , sequence_running_(false)
    , stop_sequence_flag_(false)
    , logger_(rclcpp::get_logger("task_manager"))
{
    RCLCPP_INFO(logger_, "TaskManager created");
}

bool TaskManager::initialize()
{
    RCLCPP_INFO(logger_, "Initializing TaskManager...");

    // Load default named positions
    named_positions_["home"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    named_positions_["ready"] = {0.0, -0.5, 0.8, 0.3, 0.0, 0.0};
    named_positions_["observe"] = {0.0, -0.8, 1.0, -0.2, 0.0, 0.0};
    named_positions_["rest"] = {0.0, 1.2, -1.5, 0.3, 0.0, 0.0};

    RCLCPP_INFO(logger_, "TaskManager initialized with %zu named positions",
                named_positions_.size());
    return true;
}

// ========================================
// Named Position Tasks
// ========================================

TaskResult TaskManager::moveToNamed(const std::string& position_name)
{
    auto start_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(logger_, "Moving to named position: %s", position_name.c_str());

    // Check if position exists
    if (named_positions_.find(position_name) == named_positions_.end()) {
        return createResult(false, "Named position '" + position_name + "' not found");
    }

    // Set velocity scaling
    commander_->setVelocityScaling(default_velocity_scaling_);
    commander_->setAccelerationScaling(default_acceleration_scaling_);

    // Execute movement
    bool success = commander_->moveToJointPosition(named_positions_[position_name]);

    auto end_time = std::chrono::steady_clock::now();
    double duration = std::chrono::duration<double>(end_time - start_time).count();

    if (success) {
        return createResult(true, "Successfully moved to " + position_name, duration);
    } else {
        return createResult(false, "Failed to move to " + position_name, duration);
    }
}

bool TaskManager::registerNamedPosition(const std::string& name,
                                       const std::vector<double>& joint_positions)
{
    if (joint_positions.size() != 6) {
        RCLCPP_ERROR(logger_, "Invalid joint positions size: %zu (expected 6)",
                     joint_positions.size());
        return false;
    }

    named_positions_[name] = joint_positions;
    RCLCPP_INFO(logger_, "Registered named position: %s", name.c_str());
    return true;
}

std::vector<std::string> TaskManager::getNamedPositions() const
{
    std::vector<std::string> names;
    for (const auto& pair : named_positions_) {
        names.push_back(pair.first);
    }
    return names;
}

// ========================================
// Waypoint Sequence Tasks
// ========================================

TaskResult TaskManager::executeWaypointSequence(const std::vector<Waypoint>& waypoints,
                                               bool loop,
                                               int loop_count)
{
    auto start_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(logger_, "Executing waypoint sequence with %zu waypoints (loop=%s, count=%d)",
                waypoints.size(), loop ? "true" : "false", loop_count);

    sequence_running_ = true;
    stop_sequence_flag_ = false;

    int current_loop = 0;
    bool infinite_loop = (loop_count == 0);

    while ((infinite_loop || current_loop < loop_count) && !stop_sequence_flag_) {
        for (const auto& waypoint : waypoints) {
            if (stop_sequence_flag_) {
                RCLCPP_WARN(logger_, "Waypoint sequence stopped by user");
                sequence_running_ = false;
                return createResult(false, "Sequence stopped by user");
            }

            RCLCPP_INFO(logger_, "Moving to waypoint: %s", waypoint.name.c_str());

            // Set velocity scaling for this waypoint
            commander_->setVelocityScaling(waypoint.velocity_scaling);

            // Execute movement
            bool success = commander_->moveToJointPosition(waypoint.joint_positions);

            if (!success) {
                RCLCPP_ERROR(logger_, "Failed to reach waypoint: %s", waypoint.name.c_str());
                sequence_running_ = false;
                return createResult(false, "Failed at waypoint: " + waypoint.name);
            }

            // Pause at waypoint if specified
            if (waypoint.pause_duration > 0.0) {
                RCLCPP_DEBUG(logger_, "Pausing for %.2f seconds", waypoint.pause_duration);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(waypoint.pause_duration * 1000)));
            }
        }

        current_loop++;
        if (!infinite_loop && current_loop < loop_count) {
            RCLCPP_INFO(logger_, "Completed loop %d/%d", current_loop, loop_count);
        }
    }

    sequence_running_ = false;

    auto end_time = std::chrono::steady_clock::now();
    double duration = std::chrono::duration<double>(end_time - start_time).count();

    return createResult(true, "Waypoint sequence completed", duration);
}

void TaskManager::stopSequence()
{
    RCLCPP_WARN(logger_, "Stop sequence requested");
    stop_sequence_flag_ = true;
}

// ========================================
// Pick and Place Tasks
// ========================================

TaskResult TaskManager::pickObject(const geometry_msgs::msg::Pose& pick_pose,
                                  double approach_distance)
{
    auto start_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(logger_, "Executing pick task at (%.3f, %.3f, %.3f)",
                pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);

    // Step 1: Open gripper
    RCLCPP_INFO(logger_, "Step 1: Opening gripper");
    if (!openGripper()) {
        return createResult(false, "Failed to open gripper");
    }

    // Step 2: Move to approach pose
    RCLCPP_INFO(logger_, "Step 2: Moving to approach position");
    auto approach_pose = createApproachPose(pick_pose, approach_distance);

    if (!commander_->moveToPose(approach_pose)) {
        return createResult(false, "Failed to reach approach position");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Step 3: Move down to pick pose
    RCLCPP_INFO(logger_, "Step 3: Moving down to object");
    commander_->setVelocityScaling(0.2);  // Slow down for precision

    if (!commander_->moveToPose(pick_pose)) {
        return createResult(false, "Failed to reach pick position");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Step 4: Close gripper
    RCLCPP_INFO(logger_, "Step 4: Closing gripper");
    if (!closeGripper()) {
        return createResult(false, "Failed to close gripper");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Step 5: Lift object
    RCLCPP_INFO(logger_, "Step 5: Lifting object");
    commander_->setVelocityScaling(0.3);

    if (!commander_->moveToPose(approach_pose)) {
        return createResult(false, "Failed to lift object");
    }

    // Reset velocity
    commander_->setVelocityScaling(default_velocity_scaling_);

    auto end_time = std::chrono::steady_clock::now();
    double duration = std::chrono::duration<double>(end_time - start_time).count();

    RCLCPP_INFO(logger_, "Pick task completed in %.2f seconds", duration);
    return createResult(true, "Pick successful", duration);
}

TaskResult TaskManager::placeObject(const geometry_msgs::msg::Pose& place_pose,
                                   double retreat_distance)
{
    auto start_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(logger_, "Executing place task at (%.3f, %.3f, %.3f)",
                place_pose.position.x, place_pose.position.y, place_pose.position.z);

    // Step 1: Move to approach pose
    RCLCPP_INFO(logger_, "Step 1: Moving to approach position");
    auto approach_pose = createApproachPose(place_pose, retreat_distance);

    if (!commander_->moveToPose(approach_pose)) {
        return createResult(false, "Failed to reach approach position");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Step 2: Move down to place pose
    RCLCPP_INFO(logger_, "Step 2: Moving down to place position");
    commander_->setVelocityScaling(0.2);

    if (!commander_->moveToPose(place_pose)) {
        return createResult(false, "Failed to reach place position");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Step 3: Open gripper
    RCLCPP_INFO(logger_, "Step 3: Opening gripper to release object");
    if (!openGripper()) {
        return createResult(false, "Failed to open gripper");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Step 4: Retreat
    RCLCPP_INFO(logger_, "Step 4: Retreating");
    commander_->setVelocityScaling(0.3);

    if (!commander_->moveToPose(approach_pose)) {
        return createResult(false, "Failed to retreat");
    }

    // Reset velocity
    commander_->setVelocityScaling(default_velocity_scaling_);

    auto end_time = std::chrono::steady_clock::now();
    double duration = std::chrono::duration<double>(end_time - start_time).count();

    RCLCPP_INFO(logger_, "Place task completed in %.2f seconds", duration);
    return createResult(true, "Place successful", duration);
}

TaskResult TaskManager::pickAndPlace(const geometry_msgs::msg::Pose& pick_pose,
                                    const geometry_msgs::msg::Pose& place_pose)
{
    auto start_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(logger_, "Executing pick-and-place task");

    // Execute pick
    auto pick_result = pickObject(pick_pose);
    if (!pick_result.success) {
        return createResult(false, "Pick-and-place failed at pick: " + pick_result.message);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Execute place
    auto place_result = placeObject(place_pose);
    if (!place_result.success) {
        return createResult(false, "Pick-and-place failed at place: " + place_result.message);
    }

    auto end_time = std::chrono::steady_clock::now();
    double duration = std::chrono::duration<double>(end_time - start_time).count();

    RCLCPP_INFO(logger_, "Pick-and-place completed in %.2f seconds", duration);
    return createResult(true, "Pick-and-place successful", duration);
}

// ========================================
// Pattern Tasks
// ========================================

TaskResult TaskManager::moveSquarePattern(const geometry_msgs::msg::Point& center,
                                         double size,
                                         double height)
{
    RCLCPP_INFO(logger_, "Executing square pattern: center(%.3f, %.3f), size=%.3f, height=%.3f",
                center.x, center.y, size, height);

    double half_size = size / 2.0;

    // Create 4 corner poses
    std::vector<geometry_msgs::msg::Pose> corners(4);
    for (auto& corner : corners) {
        corner.position.z = height;
        corner.orientation.w = 1.0;  // Identity quaternion
    }

    // Corner 1: front-right
    corners[0].position.x = center.x + half_size;
    corners[0].position.y = center.y + half_size;

    // Corner 2: front-left
    corners[1].position.x = center.x + half_size;
    corners[1].position.y = center.y - half_size;

    // Corner 3: back-left
    corners[2].position.x = center.x - half_size;
    corners[2].position.y = center.y - half_size;

    // Corner 4: back-right
    corners[3].position.x = center.x - half_size;
    corners[3].position.y = center.y + half_size;

    // Execute square movement
    for (size_t i = 0; i < corners.size(); ++i) {
        RCLCPP_INFO(logger_, "Moving to corner %zu", i + 1);

        if (!commander_->moveToPose(corners[i])) {
            return createResult(false, "Failed at corner " + std::to_string(i + 1));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Return to first corner to close the square
    if (!commander_->moveToPose(corners[0])) {
        return createResult(false, "Failed to return to start");
    }

    return createResult(true, "Square pattern completed");
}

TaskResult TaskManager::moveCirclePattern(const geometry_msgs::msg::Point& center,
                                         double radius,
                                         double height,
                                         int num_points)
{
    RCLCPP_INFO(logger_, "Executing circle pattern: center(%.3f, %.3f), radius=%.3f, points=%d",
                center.x, center.y, radius, num_points);

    for (int i = 0; i <= num_points; ++i) {
        double angle = 2.0 * M_PI * i / num_points;

        geometry_msgs::msg::Pose pose;
        pose.position.x = center.x + radius * cos(angle);
        pose.position.y = center.y + radius * sin(angle);
        pose.position.z = height;
        pose.orientation.w = 1.0;

        RCLCPP_DEBUG(logger_, "Circle point %d: (%.3f, %.3f, %.3f)",
                     i, pose.position.x, pose.position.y, pose.position.z);

        if (!commander_->moveToPose(pose)) {
            return createResult(false, "Failed at circle point " + std::to_string(i));
        }
    }

    return createResult(true, "Circle pattern completed");
}

// ========================================
// Configuration
// ========================================

void TaskManager::setDefaultVelocityScaling(double scaling)
{
    default_velocity_scaling_ = std::clamp(scaling, 0.01, 1.0);
    RCLCPP_INFO(logger_, "Default velocity scaling set to %.2f", default_velocity_scaling_);
}

void TaskManager::setDefaultAccelerationScaling(double scaling)
{
    default_acceleration_scaling_ = std::clamp(scaling, 0.01, 1.0);
    RCLCPP_INFO(logger_, "Default acceleration scaling set to %.2f", default_acceleration_scaling_);
}

void TaskManager::setGripperPositions(double open_position, double close_position)
{
    gripper_open_position_ = open_position;
    gripper_close_position_ = close_position;
    RCLCPP_INFO(logger_, "Gripper positions set: open=%.3f, close=%.3f",
                open_position, close_position);
}

std::string TaskManager::getStatus() const
{
    if (sequence_running_) {
        return "RUNNING_SEQUENCE";
    } else {
        return "IDLE";
    }
}

// ========================================
// Helper Functions
// ========================================

TaskResult TaskManager::createResult(bool success, const std::string& message, double time)
{
    return TaskResult(success, message, time);
}

geometry_msgs::msg::Pose TaskManager::createApproachPose(const geometry_msgs::msg::Pose& target,
                                                          double distance)
{
    geometry_msgs::msg::Pose approach_pose = target;
    approach_pose.position.z += distance;
    return approach_pose;
}

bool TaskManager::openGripper()
{
    // Get current joint positions
    auto joints = commander_->getCurrentJointPositions();
    if (joints.size() < 6) {
        RCLCPP_ERROR(logger_, "Invalid joint state size");
        return false;
    }

    // Set gripper joint (index 5) to open position
    joints[5] = gripper_open_position_;

    return commander_->moveToJointPosition(joints);
}

bool TaskManager::closeGripper()
{
    // Get current joint positions
    auto joints = commander_->getCurrentJointPositions();
    if (joints.size() < 6) {
        RCLCPP_ERROR(logger_, "Invalid joint state size");
        return false;
    }

    // Set gripper joint (index 5) to close position
    joints[5] = gripper_close_position_;

    return commander_->moveToJointPosition(joints);
}

}  // namespace so_arm_tasks
