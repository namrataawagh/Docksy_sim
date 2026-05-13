#!/usr/bin/env python3
"""
gazebo_bringup.launch.py
=========================
Starts ONLY the simulation infrastructure — no SLAM, no Nav2, no RViz.

Includes:
  - Gazebo Harmonic (parking_world.sdf)
  - robot_state_publisher
  - ros_gz_bridge  (all 8 topic bridges from gazebo_bridge.yaml)
  - clock_bridge   (dedicated unidirectional GZ→ROS clock)
  - static TF: base_link → lidar_link  (z=0.23 m)
  - static TF: base_link → imu_link    (z=0.0 m)

Used by navigation.launch.py so SLAM toolbox does NOT run during
navigation — AMCL is the sole publisher of the map→odom transform.

Usage (standalone):
    ros2 launch my_robot_bringup gazebo_bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_bringup     = get_package_share_directory("my_robot_bringup")
    pkg_description = get_package_share_directory("my_robot_description")
    pkg_ros_gz_sim  = get_package_share_directory("ros_gz_sim")

    default_world  = os.path.join(pkg_bringup, "worlds", "parking_world.sdf")
    urdf_file      = os.path.join(pkg_description, "urdf", "my_robot.urdf.xacro")
    bridge_config  = os.path.join(pkg_bringup, "config", "gazebo_bridge.yaml")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Full path to Gazebo SDF world file",
    )

    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", urdf_file]
    )
    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ── 1. Gazebo Harmonic ────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [LaunchConfiguration("world"), " -r"],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── 2. Robot State Publisher ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param, {"use_sim_time": True}],
    )

    # ── 3. Spawn robot ──────────────────────────────────────────────────────
    # Pickup zone pose from parking_world.sdf: <pose>0 -12 0</pose>
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-name",  "my_robot",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "-12.0", "-z", "0.05",
            "-Y", "1.5708",   # yaw=90° — facing +Y (north), matches AMCL initial_pose
        ],
    )

    # ── 4. Clock bridge (dedicated — NOT in YAML) ─────────────────────────────
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/world/parking_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        remappings=[("/world/parking_world/clock", "/clock")],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    # ── 5. Main ROS-GZ bridge (8 topics from YAML) ───────────────────────────
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

    # ── 6. Static TF: base_link → lidar_link ─────────────────────────────────
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

    # ── 7. Static TF: base_link → imu_link ───────────────────────────────────
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

    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        clock_bridge,     # clock first — before any use_sim_time node
        ros_gz_bridge,
        lidar_tf,
        imu_tf,
    ])
