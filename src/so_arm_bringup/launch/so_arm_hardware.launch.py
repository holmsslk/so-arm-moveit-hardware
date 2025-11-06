#!/usr/bin/env python3
"""
Launch file for SO-ARM with real hardware interface
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


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

    # Get URDF via xacro with hardware interface enabled
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("so_arm_description"), "urdf", "so101.urdf.xacro"]
            ),
            " ",
            "use_hardware:=true",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Get ros2_control configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("so_arm_bringup"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    # Get RViz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("so_arm_bringup"), "config", "so_arm.rviz"]
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ROS2 Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Arm Controller spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay arm_controller start after joint_state_broadcaster
    delay_arm_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        )
    )

    # MoveIt configuration
    moveit_config_package = FindPackageShare("so_arm_moveit_config")

    # MoveIt SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [moveit_config_package, "config", "so101.srdf"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Kinematics configuration
    kinematics_yaml = PathJoinSubstitution(
        [moveit_config_package, "config", "kinematics.yaml"]
    )

    # Planning configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = PathJoinSubstitution(
        [moveit_config_package, "config", "ompl_planning.yaml"]
    )

    # Trajectory execution configuration
    moveit_controllers = PathJoinSubstitution(
        [moveit_config_package, "config", "moveit_controllers.yaml"]
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

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
        ],
    )

    # Delay MoveIt start after arm_controller
    delay_move_group_after_arm_controller = TimerAction(
        period=3.0,
        actions=[move_group_node],
    )

    # RViz node
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
        ],
        condition=IfCondition(use_rviz),
    )

    # Delay RViz start after MoveIt
    delay_rviz_after_move_group = TimerAction(
        period=5.0,
        actions=[rviz_node],
    )

    nodes = [
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_move_group_after_arm_controller,
        delay_rviz_after_move_group,
    ]

    return LaunchDescription(declared_arguments + nodes)
