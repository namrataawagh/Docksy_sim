#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  phase3_audit.sh — Layer 0 topic audit + targeted diagnosis
#
#  Run while the slam_mapping.launch.py session is active:
#    bash phase3_audit.sh 2>&1 | tee /tmp/phase3_audit.txt
#
#  Then paste /tmp/phase3_audit.txt output if you need further help.
# ═══════════════════════════════════════════════════════════════════════════

source /home/namrata/Documents/ros2_ws/install/setup.bash

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              PHASE 3 — LAYER 0 TOPIC AUDIT                  ║"
echo "╚══════════════════════════════════════════════════════════════╝"

echo ""
echo "════ RUNNING NODES ════════════════════════════════════════════"
ros2 node list

echo ""
echo "════ CRITICAL TOPICS ══════════════════════════════════════════"
ros2 topic list | grep -E "^/scan$|^/map$|^/imu$|^/odom$|^/cmd_vel$|^/tf$|^/clock$|^/robot_description$|^/slam"

echo ""
echo "════ PUBLISH RATES (5s each) ══════════════════════════════════"
echo -n "/clock  : "
timeout 5 ros2 topic hz /clock  2>&1 | grep -E "average|no pub" | head -1
echo -n "/scan   : "
timeout 5 ros2 topic hz /scan   2>&1 | grep -E "average|no pub" | head -1
echo -n "/odom   : "
timeout 5 ros2 topic hz /odom   2>&1 | grep -E "average|no pub" | head -1
echo -n "/map    : "
timeout 5 ros2 topic hz /map    2>&1 | grep -E "average|no pub" | head -1
echo -n "/imu    : "
timeout 5 ros2 topic hz /imu    2>&1 | grep -E "average|no pub" | head -1

echo ""
echo "════ FRAME IDs ════════════════════════════════════════════════"
echo -n "/scan frame_id  : "
timeout 5 ros2 topic echo /scan --once 2>/dev/null | grep frame_id | head -1
echo -n "/odom frame_ids : "
timeout 5 ros2 topic echo /odom --once 2>/dev/null | grep -E "frame_id|child_frame" | head -2 | tr '\n' ' '
echo ""

echo ""
echo "════ TF CHAIN CHECK ═══════════════════════════════════════════"
echo -n "map → odom        : "
timeout 3 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -1
echo -n "odom → base_footprint : "
timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -1
echo -n "base_link → lidar_link : "
timeout 3 ros2 run tf2_ros tf2_echo base_link lidar_link 2>&1 | head -1

echo ""
echo "════ SLAM TOOLBOX STATUS ══════════════════════════════════════"
ros2 node list | grep slam_toolbox && echo "  → SLAM node is RUNNING" || echo "  → SLAM node NOT FOUND — crashed or not started"

echo ""
echo "════ LAYER DIAGNOSIS ══════════════════════════════════════════"
SCAN_OK=$(timeout 5 ros2 topic hz /scan 2>&1 | grep -c "average rate")
MAP_OK=$(timeout 5 ros2 topic hz /map 2>&1 | grep -c "average rate")
ODOM_OK=$(timeout 5 ros2 topic hz /odom 2>&1 | grep -c "average rate")
CLOCK_OK=$(timeout 3 ros2 topic hz /clock 2>&1 | grep -c "average rate")
SLAM_OK=$(ros2 node list 2>/dev/null | grep -c slam_toolbox)

if [ "$CLOCK_OK" -eq 0 ]; then
  echo "❌ LAYER 1E: /clock not publishing → check gz clock bridge entry"
elif [ "$SCAN_OK" -eq 0 ]; then
  echo "❌ LAYER 1A: /scan has no publishers"
  echo "   Fix: leading slash added to <topic>/scan</topic> in sensors.xacro"
  echo "   Did you rebuild and relaunch? → colcon build && ros2 launch ..."
elif [ "$ODOM_OK" -eq 0 ]; then
  echo "❌ LAYER 1D: /odom has no publishers → diff-drive plugin or bridge issue"
  echo "   Check: gz topic -l | grep odometry"
elif [ "$SLAM_OK" -eq 0 ]; then
  echo "❌ LAYER 1B: SLAM Toolbox node not running"
  echo "   Launch manually to see error:"
  echo "   ros2 launch slam_toolbox online_async_launch.py \\"
  echo "     slam_params_file:=/home/namrata/Documents/ros2_ws/src/valet_parking_amr/my_robot_bringup/config/mapper_params_online_async.yaml \\"
  echo "     use_sim_time:=true"
elif [ "$MAP_OK" -eq 0 ]; then
  echo "❌ LAYER 1C: /scan and SLAM running but /map not publishing"
  echo "   SLAM not yet received a valid scan pair. Check:"
  echo "   ros2 run tf2_ros tf2_echo map odom  (should return transform)"
else
  echo "✅ All topics OK — proceed with driving and map saving"
  echo "   Save map: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap"
  echo "   \"{name: {data: '/home/namrata/Documents/ros2_ws/src/valet_parking_amr/my_robot_bringup/maps/parking_map'}}\""
fi

echo ""
echo "════ AUDIT COMPLETE ═══════════════════════════════════════════"
echo "Full output saved to /tmp/phase3_audit.txt (if piped)"
