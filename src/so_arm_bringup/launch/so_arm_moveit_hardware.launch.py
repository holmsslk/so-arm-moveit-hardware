#!/usr/bin/env python3
"""
Complete MoveIt launch file for SO-ARM with real hardware interface
Includes hardware interface + MoveIt move_group + RViz
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port for servo communication",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file",
        )
    )

    # Initialize arguments
    serial_port = LaunchConfiguration("serial_port")
    use_rviz = LaunchConfiguration("use_rviz")

    # Package directories
    so_arm_description = FindPackageShare("so_arm_description")
    so_arm_bringup = FindPackageShare("so_arm_bringup")
    so_arm_moveit_config = FindPackageShare("so_arm_moveit_config")

    # Get URDF via xacro with hardware interface enabled
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([so_arm_description, "urdf", "so101.urdf.xacro"]),
            " ",
            "use_hardware:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get SRDF content
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([so_arm_moveit_config, "config", "so101.srdf"]),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    # Kinematics yaml
    kinematics_yaml = PathJoinSubstitution(
        [so_arm_moveit_config, "config", "kinematics.yaml"]
    )

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = PathJoinSubstitution(
        [so_arm_moveit_config, "config", "ompl_planning.yaml"]
    )

    # Trajectory Execution Configuration
    moveit_controllers_yaml = PathJoinSubstitution(
        [so_arm_moveit_config, "config", "moveit_controllers.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Joint limits
    joint_limits_yaml = PathJoinSubstitution(
        [so_arm_moveit_config, "config", "joint_limits.yaml"]
    )

    # ROS2 Control configuration
    ros2_controllers_yaml = PathJoinSubstitution(
        [so_arm_bringup, "config", "ros2_controllers.yaml"]
    )

    # === Nodes ===

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 2. ROS2 Control Node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_yaml],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # 3. Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 4. Arm Controller Spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # 5. MoveIt Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            ompl_planning_yaml,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": False},
        ],
    )

    # 6. RViz with MoveIt Configuration
    rviz_config_file = PathJoinSubstitution(
        [so_arm_moveit_config, "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
        ],
        condition=IfCondition(use_rviz),
    )

    # === Event Handlers for Sequential Startup ===

    # Delay arm_controller after joint_state_broadcaster
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Delay move_group after arm_controller
    delay_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[TimerAction(period=2.0, actions=[move_group_node])],
        )
    )

    # Delay RViz after move_group starts
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=move_group_node,
            on_start=[TimerAction(period=3.0, actions=[rviz_node])],
        )
    )

    nodes_to_start = [
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        delay_move_group,
        delay_rviz,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
