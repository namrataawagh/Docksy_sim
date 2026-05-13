#!/usr/bin/env python3
"""
phase2_verify.launch.py
========================
Phase 2 end-to-end sensor verification launch file.

Starts the complete simulation stack with all use_sim_time parameters
correctly propagated, then prints a step-by-step verification checklist
to the terminal.

Launch with:
    ros2 launch my_robot_bringup phase2_verify.launch.py

Optional arguments:
    world:=<path>      Override the world SDF (default: parking_world.sdf)
    rviz:=true/false   Whether to launch RViz (default: true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package share paths ────────────────────────────────────────────────
    pkg_description = get_package_share_directory("my_robot_description")
    pkg_bringup = get_package_share_directory("my_robot_bringup")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # ── Default paths ──────────────────────────────────────────────────────
    default_world = os.path.join(pkg_bringup, "worlds", "parking_world.sdf")
    default_urdf = os.path.join(pkg_description, "urdf", "my_robot.urdf.xacro")
    default_bridge_cfg = os.path.join(pkg_bringup, "config", "gazebo_bridge.yaml")
    default_rviz_cfg = os.path.join(pkg_bringup, "rviz", "sensors_phase2.rviz")

    # ── Declared arguments ─────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Full path to the Gazebo SDF world file",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Set false to suppress RViz (e.g. in CI headless tests)",
    )

    # ── Robot description (xacro → URDF string) ───────────────────────────
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", default_urdf]
    )
    robot_description = {"robot_description": robot_description_content}

    # ── Nodes ─────────────────────────────────────────────────────────────

    # 1. robot_state_publisher — MUST have use_sim_time
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    # 2. Gazebo Harmonic simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [LaunchConfiguration("world"), " -r"],
        }.items(),
    )

    # 3. Spawn robot from robot_description topic
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=["-topic", "robot_description"],
    )

    # 4. ros_gz_bridge — all sensor + control topics
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {"config_file": default_bridge_cfg},
            {"use_sim_time": True},
        ],
    )

    # 5. Static TF: base_link → lidar_link
    #    z=0.23 = chassis_height(0.20) + lidar_height/2(0.03)
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

    # 6. Static TF: base_link → imu_link
    #    z=0.0 = at center of robot body (center of mass)
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

    # 7. RViz2 with Phase 2 sensor config
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_cfg],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # ── Delayed verification banner (printed after 12s) ──────────────────
    banner = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg=[
                "\n",
                "╔══════════════════════════════════════════════════════════════╗\n",
                "║          PHASE 2 VERIFICATION — AUDIT COMMANDS              ║\n",
                "╚══════════════════════════════════════════════════════════════╝\n",
                "\n",
                "Open a NEW terminal, source the workspace, then run these:\n",
                "\n",
                "── STEP 1: Topic audit ──────────────────────────────────────\n",
                "  ros2 topic list | grep -E '^/scan$|^/imu$|^/camera'\n",
                "\n",
                "── STEP 2: Publish rates (5s each) ──────────────────────────\n",
                "  timeout 5 ros2 topic hz /scan\n",
                "  timeout 5 ros2 topic hz /imu\n",
                "  timeout 5 ros2 topic hz /camera/image_raw\n",
                "\n",
                "── STEP 3: frame_id checks ──────────────────────────────────\n",
                "  ros2 topic echo /scan --once | grep frame_id   # → lidar_link\n",
                "  ros2 topic echo /imu  --once | grep frame_id   # → imu_link\n",
                "\n",
                "── STEP 4: TF tree ──────────────────────────────────────────\n",
                "  ros2 run tf2_tools view_frames\n",
                "\n",
                "── STEP 5: Scan quality ─────────────────────────────────────\n",
                "  ros2 topic echo /scan --once\n",
                "  # Check: ranges has mix of floats + inf (not all-inf)\n",
                "\n",
                "── STEP 6: IMU quality ──────────────────────────────────────\n",
                "  ros2 topic echo /imu --once\n",
                "  # Check: linear_acceleration.z ≈ 9.81 when stationary\n",
                "\n",
                "── STEP 7: use_sim_time audit ───────────────────────────────\n",
                "  ros2 topic hz /clock    # should be ~1000 Hz\n",
                "\n",
                "── STEP 8: Teleop test ──────────────────────────────────────\n",
                "  ros2 run teleop_twist_keyboard teleop_twist_keyboard\n",
                "\n",
                "── STEP 9: Record 30s rosbag ────────────────────────────────\n",
                "  mkdir -p ~/phase2_bags\n",
                "  ros2 bag record /scan /imu /camera/image_raw \\\n",
                "    /tf /tf_static /odom /clock \\\n",
                "    -o ~/phase2_bags/phase2_sensor_check \\\n",
                "    --max-cache-size 100000000\n",
            ]),
        ],
    )

    return LaunchDescription([
        world_arg,
        rviz_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        lidar_tf,
        imu_tf,
        rviz2,
        banner,
    ])
