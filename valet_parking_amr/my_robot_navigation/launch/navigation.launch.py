#!/usr/bin/env python3
"""
navigation.launch.py — Phase 5 Nav2 Full Stack
===============================================
Starts every Nav2 node individually and manages lifecycle explicitly.

This avoids nav2_bringup timing issues where bt_navigator and
controller_server get stuck in unconfigured/inactive states.

Timeline:
  t=0s   Gazebo + robot + bridge (gazebo_bringup.launch.py, no SLAM)
  t=20s  All Nav2 nodes start (Gazebo clock stable by now)
  t=50s  lifecycle_manager_navigation starts with autostart=True
         → configure+activate all 9 nodes in sequence
         (30 s gap gives AMCL time to establish map frame via set_initial_pose)
  t=55s  Publish /initialpose to seed AMCL → map→odom TF reinforced
  t=58s  RViz2 (costmaps should be publishing by now)
  t=75s  Parking mission node (Nav2 guaranteed active before this)
"""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────────────
    nav_pkg     = get_package_share_directory("my_robot_navigation")
    bringup_pkg = get_package_share_directory("my_robot_bringup")
    nav2_pkg    = get_package_share_directory("nav2_bringup")

    # ── File paths ────────────────────────────────────────────────────────────
    nav2_params = os.path.join(nav_pkg, "config", "nav2_params.yaml")
    map_yaml    = os.path.join(bringup_pkg, "maps", "parking_map.yaml")
    rviz_config = os.path.join(nav2_pkg, "rviz", "nav2_default_view.rviz")

    assert os.path.exists(nav2_params), f"MISSING: {nav2_params}"
    assert os.path.exists(map_yaml),    f"MISSING: {map_yaml}"

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_rviz_arg  = DeclareLaunchArgument("use_rviz",    default_value="true")
    record_bag_arg = DeclareLaunchArgument("record_bag", default_value="false")

    # ══════════════════════════════════════════════════════════════════════════
    #  1 ─ Gazebo + Robot + Bridge  (NO SLAM — AMCL owns map→odom TF)
    # ══════════════════════════════════════════════════════════════════════════
    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "gazebo_bringup.launch.py")
        ),
    )

    # ══════════════════════════════════════════════════════════════════════════
    #  2 ─ Nav2 nodes — each started individually at t=12s
    # ══════════════════════════════════════════════════════════════════════════
    # All nodes spawn in 'unconfigured' state.
    # lifecycle_manager_navigation (starting at t=25s) will configure+activate
    # them in the correct dependency order.

    common = [nav2_params, {"use_sim_time": True}]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            nav2_params,
            {"use_sim_time": True},
            {"yaml_filename": map_yaml},   # explicit map path — not from YAML
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=common,
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=common,
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=common,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=common,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=common,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=common,
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=common,
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=common,
        remappings=[
            ("cmd_vel",          "cmd_vel_nav"),
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )

    nav2_nodes = GroupAction([
        map_server,
        amcl,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
    ])

    # ══════════════════════════════════════════════════════════════════════════
    #  3 ─ Lifecycle manager — starts AFTER all nodes are registered (t=25s)
    # ══════════════════════════════════════════════════════════════════════════
    # bond_timeout: 0.0 prevents manager from shutting down nodes if bond drops
    # autostart: True  triggers configure+activate sequence automatically
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart":    True,
            "bond_timeout": 0.0,
            "node_names": [
                "map_server",
                "amcl",
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            ],
        }],
    )

    # ══════════════════════════════════════════════════════════════════════════
    #  4 ─ RViz2 with Nav2 default view
    # ══════════════════════════════════════════════════════════════════════════
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # ══════════════════════════════════════════════════════════════════════════
    #  5 ─ Parking manager (Phase 5 state machine — replaces mission_node)
    # ══════════════════════════════════════════════════════════════════════════
    parking_manager_node = Node(
        package="my_robot_navigation",
        executable="parking_manager",
        name="parking_manager",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ══════════════════════════════════════════════════════════════════════════
    #  6 ─ Publish /initialpose at t=32s to seed AMCL → map→odom TF
    # ══════════════════════════════════════════════════════════════════════════
    # Published 7 seconds after lifecycle_manager fires (t=25s), so AMCL
    # is fully active before the message arrives.
    # Coordinates = pickup_zone pose from parking_world.sdf: x=0, y=-12
    publish_initial_pose = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'source /home/namrata/Documents/ros2_ws/install/setup.bash && '
            'ros2 topic pub --once /initialpose '
            'geometry_msgs/PoseWithCovarianceStamped "'
            '{header: {frame_id: map}, '
            'pose: {'
            'pose: {'
            'position: {x: 0.0, y: -12.0, z: 0.0}, '
            'orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}, '
            'covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '             0.0, 0.1, 0.0, 0.0, 0.0, 0.0, '
            '             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '             0.0, 0.0, 0.0, 0.0, 0.0, 0.05]}}"'
        ],
        shell=False,
        output='screen',
    )

    # ══════════════════════════════════════════════════════════════════════════
    #  7 ─ Rosbag recording (opt-in via record_bag:=true)
    # ══════════════════════════════════════════════════════════════════════════
    bag_stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    rosbag_node = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "-o",
            f"/home/namrata/Documents/ros2_ws/bags/valet_{bag_stamp}",
            "/valet/status",
            "/navigate_to_pose/_action/status",
            "/odom",
            "/scan",
            "/tf",
            "/tf_static",
            "/amcl_pose",
            "/cmd_vel",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )

    # ══════════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        use_rviz_arg,
        record_bag_arg,
        gazebo_bringup,                                            # t=0s
        rosbag_node,                                               # t=0s (if enabled)
        TimerAction(period=20.0, actions=[nav2_nodes]),            # t=20s (clock stable)
        TimerAction(period=50.0, actions=[lifecycle_manager]),     # t=50s
        TimerAction(period=55.0, actions=[publish_initial_pose]),  # t=55s
        TimerAction(period=58.0, actions=[rviz2]),                 # t=58s
        TimerAction(period=75.0, actions=[parking_manager_node]),  # t=75s (Phase 5)
    ])
