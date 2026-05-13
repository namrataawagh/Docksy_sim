#!/usr/bin/env python3
"""
parking_mission_node.py — Valet Parking AMR, Slot A2 Mission
=============================================================
Stack  : ROS 2 Jazzy + Gazebo Harmonic + Nav2
World  : parking_world.sdf

═══ VERIFIED SDF COORDINATES ═══════════════════════════════════════════
  pickup_zone  : x=  0.0, y=-12.0  (GREEN rectangle, south centre)
  dropoff_zone : x=  0.0, y=+12.0  (RED rectangle, north centre)

  Driving aisle: x = -2.0  (N-S corridor, aisle_center_line in SDF)
                 runs from y=-15 to y=+15

  Row A slots (extend in -X direction):
    slot_A2 anchor: x=-7, y=-2        (only empty Row-A slot)
    Opening face:   x=-4.5, y_range=[-3.25, -0.75]
    Back wall:      x=-9.5
    Slot depth:     5.0 m  (x=-4.5 to x=-9.5)
    Slot width:     2.5 m  (y=-3.25 to y=-0.75)

═══ MISSION SEQUENCE ════════════════════════════════════════════════════
  0. Seed AMCL at pickup (programmatic — map→odom TF appears)
  A. Drive north along aisle (x=-2) from y=-12 to y=-2  (slot A2 row)
  B. Drive west from aisle to approach point east of A2 opening
     → (x=-3.5, y=-2), turn to face west (yaw=π)
  C. Drive FORWARD (west) INTO slot A2
     → (x=-7.0, y=-2.0)  inside slot, facing west
  D. Sit parked 5 seconds
  E. BackUp EAST out of slot → 3.5 m reverse back to approach point
  F. Navigate forward back into aisle → (x=-2.0, y=-2.0) facing north
  G. Drive north along aisle to dropoff zone → (x=0.0, y=+12.0)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import BackUp, NavigateToPose


# ═══════════════════════════════════════════════════════════════════════
#  WORLD COORDINATES — verified against parking_world.sdf
# ═══════════════════════════════════════════════════════════════════════

# ── Pickup zone  (pickup_zone model: <pose>0 -12 0</pose>)  ───────────
PICKUP_X   =  0.0
PICKUP_Y   = -12.0
PICKUP_YAW =  0.0      # facing east (+X) toward aisle on spawn

# ── Driving aisle — x=-2, runs N-S ───────────────────────────────────
AISLE_X    = -2.0

# ── PHASE A: drive north to slot A2 row Y in the aisle ───────────────
AISLE_A2_X   = -2.0
AISLE_A2_Y   = -2.0
AISLE_A2_YAW =  0.0   # still facing east in aisle

# ── PHASE B: approach — 1 m east of slot A2 opening (x=-4.5) ─────────
# At x=-3.5 the robot is clear of the opening line and centred on A2.
# Yaw = π → robot faces west = faces INTO the slot ready to drive in.
APPROACH_X   = -3.5
APPROACH_Y   = -2.0
APPROACH_YAW =  math.pi    # face west (-X = into slot)

# ── PHASE C: drive FORWARD (west) into slot A2 ────────────────────────
# Target = slot A2 centre x: -7.0, same y=-2.0
# Robot travels from x=-3.5 forward (west) to x=-7.0 = 3.5 m inside slot.
# The robot (0.6 m long) fits comfortably; back wall is at x=-9.5.
SLOT_INSIDE_X   = -7.0
SLOT_INSIDE_Y   = -2.0
SLOT_INSIDE_YAW =  math.pi  # still facing west while parked

# ── PHASE D: parked dwell ─────────────────────────────────────────────
PARK_WAIT_SEC = 5.0

# ── PHASE E: BackUp OUT of slot (east = positive X direction) ─────────
# Reverse from x=-7.0 back to x=-3.5 = 3.5 m
BACKUP_DIST  = 3.5
BACKUP_SPEED = 0.08    # m/s — slow for precision

# ── PHASE F: return to aisle from approach point ──────────────────────
# After backing out the robot is at (x=-3.5, y=-2) facing west.
# Navigate forward to aisle (x=-2, y=-2), then face north for dropoff.
EXIT_X   = -2.0
EXIT_Y   = -2.0
EXIT_YAW =  math.pi / 2   # face north (+Y) toward dropoff

# ── PHASE G: dropoff zone  (dropoff_zone model: <pose>0 12 0</pose>) ──
DROPOFF_X   =  0.0
DROPOFF_Y   = +12.0
DROPOFF_YAW =  math.pi / 2   # face north on arrival


# ═══════════════════════════════════════════════════════════════════════
#  HELPERS
# ═══════════════════════════════════════════════════════════════════════

def yaw_to_quat(yaw: float):
    """Return (qz, qw) for a yaw-only rotation."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose(x: float, y: float, yaw: float, frame: str = "map") -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.0
    qz, qw = yaw_to_quat(yaw)
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


# ═══════════════════════════════════════════════════════════════════════
#  MISSION NODE
# ═══════════════════════════════════════════════════════════════════════

class ParkingMissionNode(Node):

    def __init__(self):
        super().__init__("parking_mission_node")
        self.get_logger().info("Parking Mission Node initialising...")

        # /initialpose — transient-local so AMCL receives it even if the
        # message is published slightly before AMCL finishes activating.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._init_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", qos
        )

        # Hard-stop publisher
        self._vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Action clients
        self._nav    = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._backup = ActionClient(self, BackUp,          "backup")

        self.get_logger().info("Waiting for navigate_to_pose server (60 s)...")
        if not self._nav.wait_for_server(timeout_sec=60.0):
            raise RuntimeError(
                "navigate_to_pose server not available — is Nav2 active?")

        self.get_logger().info("Waiting for backup server (30 s)...")
        if not self._backup.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("backup server not available")

        self.get_logger().info("All Nav2 action servers ready ✅")

    # ──────────────────────────────────────────────────────────────────
    def set_initial_pose(self, x: float, y: float, yaw: float):
        """Publish /initialpose to seed AMCL particle filter.

        The launch file already publishes a one-shot pose at t=32s.
        This call provides a second, higher-confidence seed from the
        mission node itself after the action servers are confirmed ready.
        """
        self.get_logger().info(
            f"[AMCL] Seeding pose: x={x:.2f} y={y:.2f} "
            f"yaw={math.degrees(yaw):.1f}°"
        )
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qz, qw = yaw_to_quat(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Low covariance: spawn location is exactly known.
        cov = [0.0] * 36
        cov[0]  = 0.10   # x variance  (~±0.32 m)
        cov[7]  = 0.10   # y variance
        cov[35] = 0.05   # yaw variance (~±13°)
        msg.pose.covariance = cov

        for _ in range(5):       # 5 publishes × 0.4 s = 2 s burst
            self._init_pub.publish(msg)
            time.sleep(0.4)

        self.get_logger().info(
            "Initial pose sent. Waiting 4 s for AMCL convergence..."
        )
        time.sleep(4.0)
        self.get_logger().info(
            "Convergence wait done — map→odom TF should now be live."
        )

    # ──────────────────────────────────────────────────────────────────
    def go_to(self, x: float, y: float, yaw: float,
               label: str = "goal", timeout: float = 120.0) -> bool:
        """Send NavigateToPose; block until success, failure, or timeout."""
        self.get_logger().info(
            f"[NAV] → {label}  x={x:.2f} y={y:.2f} "
            f"yaw={math.degrees(yaw):.1f}°"
        )
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw)
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        future = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f"Goal to {label} REJECTED ❌")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

        if not result_future.done():
            self.get_logger().error(
                f"Goal to {label} TIMED OUT after {timeout:.0f} s ❌")
            handle.cancel_goal_async()
            return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[NAV] Reached {label} ✅")
            return True
        self.get_logger().error(
            f"[NAV] Failed to reach {label} — status={status} ❌")
        return False

    # ──────────────────────────────────────────────────────────────────
    def backup_out(self, distance: float, speed: float) -> bool:
        """Execute BackUp action to reverse robot out of parking slot."""
        self.get_logger().info(
            f"[BACKUP] Reversing {distance:.1f} m at {speed:.2f} m/s..."
        )
        goal = BackUp.Goal()
        goal.target.x = float(distance)   # Nav2 Jazzy: target is geometry_msgs/Point
        goal.speed     = float(speed)

        future = self._backup.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("BackUp goal REJECTED ❌")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=90.0)
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reversed out of slot ✅")
            return True
        self.get_logger().error(f"BackUp failed — status={status} ❌")
        return False

    # ──────────────────────────────────────────────────────────────────
    def stop(self):
        """Publish zero velocity to halt the robot immediately."""
        t = Twist()
        for _ in range(10):
            self._vel_pub.publish(t)
            time.sleep(0.05)

    # ══════════════════════════════════════════════════════════════════
    #  MISSION
    # ══════════════════════════════════════════════════════════════════

    def run_mission(self):
        sep = "═" * 65
        self.get_logger().info(sep)
        self.get_logger().info("  VALET PARKING AMR — SLOT A2 MISSION")
        self.get_logger().info(sep)
        self.get_logger().info(f"  Pickup:  ({PICKUP_X},  {PICKUP_Y})   ← green zone, south")
        self.get_logger().info(f"  Target:  Slot A2  approach ({APPROACH_X}, {APPROACH_Y})")
        self.get_logger().info(f"           Inside   ({SLOT_INSIDE_X}, {SLOT_INSIDE_Y})")
        self.get_logger().info(f"  Dropoff: ({DROPOFF_X},   {DROPOFF_Y})  ← red zone, north")
        self.get_logger().info(sep)

        # ── 0. Seed AMCL (launch file published at t=32s; this is a
        #       second, mission-time seed for high-confidence convergence)
        self.set_initial_pose(PICKUP_X, PICKUP_Y, PICKUP_YAW)

        # ── PHASE A: Drive north along aisle to slot A2 row ───────────
        self.get_logger().info("── PHASE A: Pickup → Aisle at A2 row (y=-2) ──")
        if not self.go_to(AISLE_A2_X, AISLE_A2_Y, AISLE_A2_YAW,
                          label="aisle_A2_row"):
            self.get_logger().fatal("PHASE A failed. Aborting.")
            return

        # ── PHASE B: Drive to approach point, face west into slot ──────
        # Moves west from x=-2 to x=-3.5 and turns to yaw=π
        self.get_logger().info("── PHASE B: Approach slot A2 opening, face west ──")
        if not self.go_to(APPROACH_X, APPROACH_Y, APPROACH_YAW,
                          label="approach_A2_opening"):
            self.get_logger().fatal("PHASE B failed. Aborting.")
            return

        # ── PHASE C: Drive FORWARD (west) INTO slot A2 ────────────────
        # Robot faces west → NavigateToPose drives forward = westward.
        self.get_logger().info("── PHASE C: Drive forward INTO slot A2 ──")
        if not self.go_to(SLOT_INSIDE_X, SLOT_INSIDE_Y, SLOT_INSIDE_YAW,
                          label="slot_A2_parked_inside", timeout=60.0):
            self.get_logger().fatal("PHASE C failed. Aborting.")
            return
        self.stop()

        # ── PHASE D: Parked — sit still 5 s ───────────────────────────
        self.get_logger().info(
            f"── PARKED in slot A2 — waiting {PARK_WAIT_SEC:.0f} s... ──")
        time.sleep(PARK_WAIT_SEC)

        # ── PHASE E: BackUp EAST out of slot ──────────────────────────
        # Robot reverses 3.5 m (from x=-7.0 back to x=-3.5)
        self.get_logger().info("── PHASE E: Reversing OUT of slot A2 ──")
        self.backup_out(BACKUP_DIST, BACKUP_SPEED)
        self.stop()
        time.sleep(1.0)   # 1 s pause to let inertia settle

        # ── PHASE F: Navigate forward back to aisle, face north ───────
        self.get_logger().info("── PHASE F: Return to aisle, face north ──")
        if not self.go_to(EXIT_X, EXIT_Y, EXIT_YAW,
                          label="aisle_post_slot"):
            self.get_logger().error(
                "PHASE F failed — continuing to dropoff anyway")

        # ── PHASE G: Drive north to dropoff zone ──────────────────────
        self.get_logger().info("── PHASE G: Drive north to dropoff zone ──")
        if not self.go_to(DROPOFF_X, DROPOFF_Y, DROPOFF_YAW,
                          label="dropoff_zone"):
            self.get_logger().error("PHASE G failed.")
        self.stop()

        self.get_logger().info(sep)
        self.get_logger().info("  VALET PARKING MISSION COMPLETE ✅")
        self.get_logger().info(f"  Pickup  ({PICKUP_X}, {PICKUP_Y})")
        self.get_logger().info(f"  Parked  Slot A2 ({SLOT_INSIDE_X}, {SLOT_INSIDE_Y})")
        self.get_logger().info(f"  Dropoff ({DROPOFF_X}, {DROPOFF_Y})")
        self.get_logger().info(sep)


# ═══════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ParkingMissionNode()
        node.run_mission()
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Mission interrupted by user.")
            node.stop()
    except RuntimeError as e:
        print(f"[FATAL] {e}")
    finally:
        if node:
            node.stop()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
