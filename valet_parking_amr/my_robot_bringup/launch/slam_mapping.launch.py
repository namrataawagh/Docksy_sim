#!/usr/bin/env python3
"""
slam_mapping.launch.py
=======================
Phase 3 — SLAM Toolbox Online Async Mapping

Starts the complete simulation + SLAM stack:
  Gazebo Harmonic  →  robot_state_publisher  →  ros_gz_bridge
  →  static TF publishers  →  slam_toolbox (delayed 5s)  →  RViz2

Does NOT start Nav2 — mapping only.
Phase 4 will load the saved parking_map.yaml for localization + navigation.

Usage:
    ros2 launch my_robot_bringup slam_mapping.launch.py
    ros2 launch my_robot_bringup slam_mapping.launch.py rviz:=false
    ros2 launch my_robot_bringup slam_mapping.launch.py world:=/path/to/custom.sdf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # ── Package share paths ─────────────────────────────────────────────────
    pkg_bringup = get_package_share_directory("my_robot_bringup")
    pkg_description = get_package_share_directory("my_robot_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # ── File paths ──────────────────────────────────────────────────────────
    default_world = os.path.join(pkg_bringup, "worlds", "parking_world.sdf")
    urdf_file = os.path.join(pkg_description, "urdf", "my_robot.urdf.xacro")
    slam_params = os.path.join(pkg_bringup, "config", "mapper_params_online_async.yaml")
    rviz_config = os.path.join(pkg_bringup, "rviz", "slam_mapping.rviz")
    bridge_config = os.path.join(pkg_bringup, "config", "gazebo_bridge.yaml")

    # ── Declared arguments ──────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Full path to the Gazebo SDF world file",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 (set false for headless/CI runs)",
    )

    # ── Robot description — xacro processed at launch time ─────────────────
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_file]),
        value_type=str
    )
    robot_description_param = {"robot_description": robot_description_content}

    # ══════════════════════════════════════════════════════════════════════════
    #  NODE DEFINITIONS — all use_sim_time: True
    # ══════════════════════════════════════════════════════════════════════════

    # 1 ─ Gazebo Harmonic ─────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [LaunchConfiguration("world"), " -r"],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # 2 ─ Robot State Publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description_param,
            {"use_sim_time": True},
        ],
    )

    # 3 ─ Spawn robot in Gazebo ───────────────────────────────────────────────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "-12.0",   # pickup zone — pickup_zone model pose: 0 -12 0 in SDF
            "-z", "0.05",
            "-Y", "0.0",     # yaw=0 — facing east (+X)
        ],
    )

    # 4 ─ ROS–GZ Bridge (reuses existing bridge config from Phase 2) ──────────
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_config},
            {"use_sim_time": True},
        ],
    )

    # 4b ─ Dedicated clock bridge ─────────────────────────────────────────────
    # MUST be a separate node from the main bridge.
    # Reason: if clock is in the YAML config, ros_gz_bridge becomes a
    # publisher on /clock, which Gazebo detects and stops publishing to
    # /clock — switching exclusively to /world/<worldname>/clock.
    # Using [ (unidirectional GZ→ROS) in arguments syntax avoids this.
    # use_sim_time: False intentionally — this node PROVIDES the clock,
    # it cannot wait for a clock that it itself is supposed to publish.
    # Reference: github.com/gazebosim/ros_gz/issues/341
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/world/parking_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        remappings=[
            ("/world/parking_world/clock", "/clock"),
        ],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    # 5 ─ Static TF: base_link → lidar_link  (z=0.23 m above base_link) ──────
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_tf_pub",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.23",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar_link",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # 6 ─ Static TF: base_link → imu_link  (at centre of chassis) ────────────
    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf_pub",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "imu_link",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # 7 ─ SLAM Toolbox — online async mapping ─────────────────────────────────
    # Delayed 8 s so bridge + RSP are fully up before SLAM subscribes to /scan.
    #
    # Lifecycle management via explicit ExecuteProcess calls:
    #   t=8s  → start async_slam_toolbox_node (spawns in 'unconfigured')
    #   t=10s → ros2 lifecycle set /slam_toolbox configure  → 'inactive'
    #   t=12s → ros2 lifecycle set /slam_toolbox activate   → 'active'
    #
    # The autostart param approach is unreliable because the lifecycle state
    # machine initialises before parameters are loaded. Explicit shell calls
    # are guaranteed to run after the node is up and param-ready.
    slam_toolbox_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_params,           # mapper_params_online_async.yaml
                    {"use_sim_time": True}, # must match all other nodes
                ],
            )
        ],
    )

    slam_configure = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
                output="screen",
            )
        ],
    )

    slam_activate = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
                output="screen",
            )
        ],
    )

    # 8 ─ RViz2 with SLAM map display ─────────────────────────────────────────
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # ══════════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        world_arg,
        rviz_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        clock_bridge,       # clock must start before any use_sim_time node
        ros_gz_bridge,      # main bridge
        lidar_tf,
        imu_tf,
        slam_toolbox_node,  # t=8s: start node
        slam_configure,     # t=10s: configure lifecycle
        slam_activate,      # t=12s: activate lifecycle → subscribes /scan, publishes /map
        rviz2,
    ])
