// Copyright 2025 SO-ARM Project
// Licensed under the Apache License, Version 2.0

#include "so_arm_commander_cpp/so_arm_commander.hpp"
#include <cmath>
#include <chrono>
#include <thread>

namespace so_arm_commander
{

// ========================================
// Constructor and Destructor
// ========================================

SoArmCommander::SoArmCommander(const rclcpp::NodeOptions& options)
  : Node("so_arm_commander",
         rclcpp::NodeOptions(options)
           .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true))
  , planning_group_("arm")
  , planning_time_(5.0)
  , velocity_scaling_factor_(0.5)
  , acceleration_scaling_factor_(0.5)
  , gripper_open_position_(1.0)
  , gripper_closed_position_(0.0)
  , gripper_joint_index_(5)
  , workspace_min_x_(0.05)
  , workspace_max_x_(0.35)
  , workspace_min_y_(-0.25)
  , workspace_max_y_(0.25)
  , workspace_min_z_(0.05)
  , workspace_max_z_(0.40)
{
    RCLCPP_INFO(this->get_logger(), "SoArmCommander node created");
}

bool SoArmCommander::initialize()
{
    RCLCPP_INFO(this->get_logger(), "Initializing SoArmCommander...");

    // Initialize MoveIt
    if (!initializeMoveIt()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt!");
        return false;
    }

    // Load named poses
    if (!loadNamedPoses()) {
        RCLCPP_WARN(this->get_logger(), "Failed to load named poses, using defaults");
    }

    RCLCPP_INFO(this->get_logger(), "SoArmCommander initialized successfully!");
    return true;
}


SoArmCommander::~SoArmCommander()
{
    RCLCPP_INFO(this->get_logger(), "SoArmCommander shutting down...");
}

// ========================================
// Basic Motion Control
// ========================================

bool SoArmCommander::goHome()
{
    RCLCPP_INFO(this->get_logger(), "Moving to home position...");
    std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    return moveToJointPosition(home_position);
}

bool SoArmCommander::moveToJointPosition(const std::vector<double>& positions)
{
    if (!validateJointPositions(positions)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid joint positions!");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Moving to joint position...");

    // Set target
    arm_group_->setJointValueTarget(positions);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        return false;
    }

    // Execute
    auto result = arm_group_->execute(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Motion completed successfully!");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Execution failed!");
        return false;
    }
}

std::vector<double> SoArmCommander::getCurrentJointPositions()
{
    return arm_group_->getCurrentJointValues();
}

// ========================================
// Cartesian Space Motion
// ========================================

bool SoArmCommander::moveToPose(const geometry_msgs::msg::Pose& target)
{
    // Check workspace bounds
    if (!isInWorkspace(target.position.x, target.position.y, target.position.z)) {
        RCLCPP_ERROR(this->get_logger(),
            "Target position (%.3f, %.3f, %.3f) is outside safe workspace!",
            target.position.x, target.position.y, target.position.z);
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
        "Moving to pose: (%.3f, %.3f, %.3f)",
        target.position.x, target.position.y, target.position.z);

    // Set target
    arm_group_->setPoseTarget(target);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Cartesian planning failed!");
        return false;
    }

    // Execute
    auto result = arm_group_->execute(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Cartesian motion completed!");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cartesian execution failed!");
        return false;
    }
}

bool SoArmCommander::moveToPose(double x, double y, double z,
                                 double roll, double pitch, double yaw)
{
    geometry_msgs::msg::Pose target = createPose(x, y, z, roll, pitch, yaw);
    return moveToPose(target);
}

bool SoArmCommander::moveLinear(const geometry_msgs::msg::Pose& target,
                                 double velocity_scaling)
{
    RCLCPP_INFO(this->get_logger(), "Executing linear motion...");

    // Save current velocity scaling
    double original_scaling = velocity_scaling_factor_;
    setVelocityScaling(velocity_scaling);

    // Compute Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = arm_group_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.95) {
        RCLCPP_WARN(this->get_logger(),
            "Linear path only %.2f%% complete", fraction * 100.0);
    }

    // Execute trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    auto result = arm_group_->execute(plan);

    // Restore velocity scaling
    setVelocityScaling(original_scaling);

    return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

geometry_msgs::msg::Pose SoArmCommander::getCurrentPose()
{
    return arm_group_->getCurrentPose().pose;
}

// ========================================
// Named Poses
// ========================================

bool SoArmCommander::setNamedTarget(const std::string& target_name)
{
    if (named_poses_.find(target_name) == named_poses_.end()) {
        RCLCPP_ERROR(this->get_logger(),
            "Named pose '%s' not found!", target_name.c_str());

        // Print available poses
        RCLCPP_INFO(this->get_logger(), "Available poses:");
        for (const auto& pair : named_poses_) {
            RCLCPP_INFO(this->get_logger(), "  - %s", pair.first.c_str());
        }
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
        "Moving to named pose: %s", target_name.c_str());

    return moveToJointPosition(named_poses_[target_name]);
}

bool SoArmCommander::addNamedPose(const std::string& name,
                                   const std::vector<double>& positions)
{
    if (!validateJointPositions(positions)) {
        RCLCPP_ERROR(this->get_logger(),
            "Invalid joint positions for named pose '%s'", name.c_str());
        return false;
    }

    named_poses_[name] = positions;
    RCLCPP_INFO(this->get_logger(),
        "Added named pose: %s", name.c_str());
    return true;
}

std::vector<std::string> SoArmCommander::getNamedPosesList()
{
    std::vector<std::string> names;
    for (const auto& pair : named_poses_) {
        names.push_back(pair.first);
    }
    return names;
}

// ========================================
// Gripper Control
// ========================================

bool SoArmCommander::openGripper()
{
    RCLCPP_INFO(this->get_logger(), "Opening gripper...");
    return setGripperPosition(1.0);
}

bool SoArmCommander::closeGripper()
{
    RCLCPP_INFO(this->get_logger(), "Closing gripper...");
    return setGripperPosition(0.0);
}

bool SoArmCommander::setGripperPosition(double position)
{
    // Clamp position to [0, 1]
    position = std::max(0.0, std::min(1.0, position));

    // Get current joint positions
    std::vector<double> current_positions = getCurrentJointPositions();

    if (current_positions.size() < 6) {
        RCLCPP_ERROR(this->get_logger(), "Invalid number of joints!");
        return false;
    }

    // Calculate actual gripper angle
    double gripper_angle = gripper_closed_position_ +
        position * (gripper_open_position_ - gripper_closed_position_);

    // Set gripper joint
    current_positions[gripper_joint_index_] = gripper_angle;

    RCLCPP_DEBUG(this->get_logger(),
        "Setting gripper to %.2f%% (%.3f rad)",
        position * 100.0, gripper_angle);

    return moveToJointPosition(current_positions);
}

double SoArmCommander::getGripperPosition()
{
    std::vector<double> positions = getCurrentJointPositions();

    if (positions.size() <= gripper_joint_index_) {
        RCLCPP_ERROR(this->get_logger(), "Cannot get gripper position!");
        return 0.0;
    }

    double gripper_angle = positions[gripper_joint_index_];

    // Convert to normalized position
    double position = (gripper_angle - gripper_closed_position_) /
        (gripper_open_position_ - gripper_closed_position_);

    return std::max(0.0, std::min(1.0, position));
}

// ========================================
// High-Level Operations
// ========================================

bool SoArmCommander::pick(const geometry_msgs::msg::Pose& target,
                           double approach_distance)
{
    RCLCPP_INFO(this->get_logger(), "Executing pick sequence...");

    // 1. Move to pre-grasp position (above target)
    geometry_msgs::msg::Pose pre_grasp = target;
    pre_grasp.position.z += approach_distance;

    RCLCPP_INFO(this->get_logger(), "Moving to pre-grasp position...");
    if (!moveToPose(pre_grasp)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to reach pre-grasp position!");
        return false;
    }

    // 2. Open gripper
    RCLCPP_INFO(this->get_logger(), "Opening gripper...");
    if (!openGripper()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open gripper!");
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 3. Move down to grasp position
    RCLCPP_INFO(this->get_logger(), "Descending to grasp position...");
    if (!moveLinear(target, 0.3)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to reach grasp position!");
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 4. Close gripper
    RCLCPP_INFO(this->get_logger(), "Closing gripper...");
    if (!closeGripper()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to close gripper!");
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 5. Lift up
    RCLCPP_INFO(this->get_logger(), "Lifting object...");
    if (!moveLinear(pre_grasp, 0.3)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to lift object!");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Pick sequence completed successfully!");
    return true;
}

bool SoArmCommander::place(const geometry_msgs::msg::Pose& target,
                            double retreat_distance)
{
    RCLCPP_INFO(this->get_logger(), "Executing place sequence...");

    // 1. Move to pre-place position (above target)
    geometry_msgs::msg::Pose pre_place = target;
    pre_place.position.z += retreat_distance;

    RCLCPP_INFO(this->get_logger(), "Moving to pre-place position...");
    if (!moveToPose(pre_place)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to reach pre-place position!");
        return false;
    }

    // 2. Move down to place position
    RCLCPP_INFO(this->get_logger(), "Descending to place position...");
    if (!moveLinear(target, 0.3)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to reach place position!");
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 3. Open gripper
    RCLCPP_INFO(this->get_logger(), "Opening gripper...");
    if (!openGripper()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open gripper!");
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 4. Retreat
    RCLCPP_INFO(this->get_logger(), "Retreating...");
    if (!moveLinear(pre_place, 0.3)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to retreat!");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Place sequence completed successfully!");
    return true;
}

bool SoArmCommander::pickAndPlace(const geometry_msgs::msg::Pose& pick_pose,
                                   const geometry_msgs::msg::Pose& place_pose)
{
    RCLCPP_INFO(this->get_logger(), "Executing pick and place sequence...");

    if (!pick(pick_pose)) {
        RCLCPP_ERROR(this->get_logger(), "Pick failed!");
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (!place(place_pose)) {
        RCLCPP_ERROR(this->get_logger(), "Place failed!");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Pick and place completed successfully!");
    return true;
}

// ========================================
// Configuration
// ========================================

void SoArmCommander::setPlanningTime(double seconds)
{
    planning_time_ = seconds;
    arm_group_->setPlanningTime(seconds);
    RCLCPP_INFO(this->get_logger(),
        "Planning time set to %.2f seconds", seconds);
}

void SoArmCommander::setVelocityScaling(double factor)
{
    velocity_scaling_factor_ = std::max(0.0, std::min(1.0, factor));
    arm_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    RCLCPP_DEBUG(this->get_logger(),
        "Velocity scaling set to %.2f", velocity_scaling_factor_);
}

void SoArmCommander::setAccelerationScaling(double factor)
{
    acceleration_scaling_factor_ = std::max(0.0, std::min(1.0, factor));
    arm_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    RCLCPP_DEBUG(this->get_logger(),
        "Acceleration scaling set to %.2f", acceleration_scaling_factor_);
}

void SoArmCommander::setWorkspaceBounds(double min_x, double max_x,
                                         double min_y, double max_y,
                                         double min_z, double max_z)
{
    workspace_min_x_ = min_x;
    workspace_max_x_ = max_x;
    workspace_min_y_ = min_y;
    workspace_max_y_ = max_y;
    workspace_min_z_ = min_z;
    workspace_max_z_ = max_z;

    RCLCPP_INFO(this->get_logger(),
        "Workspace bounds set: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
        min_x, max_x, min_y, max_y, min_z, max_z);
}

bool SoArmCommander::isInWorkspace(double x, double y, double z)
{
    return (x >= workspace_min_x_ && x <= workspace_max_x_ &&
            y >= workspace_min_y_ && y <= workspace_max_y_ &&
            z >= workspace_min_z_ && z <= workspace_max_z_);
}

// ========================================
// Helper Functions
// ========================================

bool SoArmCommander::initializeMoveIt()
{
    try {
        RCLCPP_INFO(this->get_logger(), "Setting up robot description parameters...");

        // 创建异步参数客户端来从 /move_group 节点获取参数
        auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(
            this->get_node_base_interface(),
            this->get_node_topics_interface(),
            this->get_node_graph_interface(),
            this->get_node_services_interface(),
            "/move_group");

        // 等待参数服务可用
        if (!param_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(),
                "Parameter service /move_group not available. Is the launch file running?");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Fetching robot_description parameters from /move_group...");

        // 获取必要的参数
        std::vector<std::string> param_names = {
            "robot_description",
            "robot_description_semantic"
        };

        try {
            // 异步获取参数
            auto future = param_client->get_parameters(param_names);

            // 等待结果（最多5秒）
            if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
                auto params = future.get();

                // 设置到当前节点
                for (const auto& param : params) {
                    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                        this->set_parameter(param);
                        RCLCPP_DEBUG(this->get_logger(), "Set parameter: %s", param.get_name().c_str());
                    }
                }

                RCLCPP_INFO(this->get_logger(), "Successfully fetched robot description parameters");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for parameters from /move_group");
                return false;
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to get parameters from /move_group: %s", e.what());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Creating MoveGroupInterface...");

        // Create MoveGroupInterface
        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group_);

        // Create PlanningSceneInterface
        planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Set default parameters
        arm_group_->setPlanningTime(planning_time_);
        arm_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
        arm_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);

        // Print info
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s",
            arm_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s",
            arm_group_->getEndEffectorLink().c_str());

        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(),
            "Exception during MoveIt initialization: %s", e.what());
        return false;
    }
}

bool SoArmCommander::loadNamedPoses()
{
    // Define default named poses
    named_poses_["home"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    named_poses_["ready"] = {0.0, -0.5, 0.8, 0.3, 0.0, 0.0};
    named_poses_["observe"] = {0.0, -0.8, 1.0, -0.2, 0.0, 0.0};
    named_poses_["sleep"] = {0.0, 1.2, -1.5, 0.3, 0.0, 0.0};

    RCLCPP_INFO(this->get_logger(), "Loaded %lu named poses", named_poses_.size());

    // TODO: Load from parameter server or YAML file

    return true;
}

bool SoArmCommander::validateJointPositions(const std::vector<double>& positions)
{
    if (positions.size() != 6) {
        RCLCPP_ERROR(this->get_logger(),
            "Invalid number of joints: expected 6, got %lu", positions.size());
        return false;
    }

    // Check for NaN or Inf
    for (size_t i = 0; i < positions.size(); ++i) {
        if (std::isnan(positions[i]) || std::isinf(positions[i])) {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid value at joint %lu: %f", i, positions[i]);
            return false;
        }
    }

    // TODO: Check against joint limits

    return true;
}

geometry_msgs::msg::Pose SoArmCommander::createPose(double x, double y, double z,
                                                      double roll, double pitch, double yaw)
{
    geometry_msgs::msg::Pose pose;

    // Set position
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    // Convert RPY to quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

bool SoArmCommander::planAndExecute()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        return false;
    }

    auto result = arm_group_->execute(plan);
    return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

bool SoArmCommander::waitForExecution(double timeout)
{
    // Simple timeout-based wait
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - start_time).count();

        if (elapsed >= timeout) {
            RCLCPP_WARN(this->get_logger(), "Execution timeout!");
            return false;
        }

        // TODO: Check actual execution state
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // For now, just return true after short delay
        if (elapsed > 1.0) {
            return true;
        }
    }
}

}  // namespace so_arm_commander
