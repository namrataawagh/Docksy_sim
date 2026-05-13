#!/usr/bin/env python3
"""
parking_manager.py  —  Valet Parking AMR  (ROS 2 Jazzy + Nav2)  Phase 5
=========================================================================
State machine:
  INIT -> GO_TO_PICKUP -> FIND_SLOT -> GO_TO_SLOT -> DWELL
       -> BACKUP_EXIT -> GO_TO_DROPOFF -> DONE

Phase 5 additions (rules: structure unchanged, only additions):
  - BACKUP_EXIT state: proper BackUp action after dwell
  - Costmap clearing on repeated nav failure
  - /valet/status  : JSON publisher at 1 Hz
  - /valet/park    : Trigger service to restart a cycle
  - /valet/retrieve: Trigger service (stub)
"""

import json
import math
import time
import enum
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as RosDuration
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import BackUp, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import String
from std_srvs.srv import Trigger


# ==============================================================================
#  World constants
# ==============================================================================

PICKUP_X    =  0.0
PICKUP_Y    = -12.0
PICKUP_YAW  =  math.pi / 2

DROPOFF_X   =  0.0
DROPOFF_Y   =  12.0
DROPOFF_YAW =  math.pi / 2

AISLE_X          = -2.0
APPROACH_OFFSET  =  1.5


# ==============================================================================
#  Slot definitions
# ==============================================================================

@dataclass
class ParkingSlot:
    name: str
    center_x: float
    center_y: float
    face_yaw: float
    approach_x: float
    approach_y: float
    pre_occupied: bool
    occupied: bool = field(init=False)

    def __post_init__(self):
        self.occupied = self.pre_occupied


def _build_slots():
    row_a_approach_x = -4.5 + APPROACH_OFFSET   # -3.0
    row_a_face_yaw   = math.pi
    row_b_approach_x = 0.5 - APPROACH_OFFSET     # -1.0
    row_b_face_yaw   = 0.0

    return [
        ParkingSlot("A1", -7.0, -8.0, row_a_face_yaw, row_a_approach_x, -8.0, pre_occupied=True),
        ParkingSlot("A2", -7.0, -2.0, row_a_face_yaw, row_a_approach_x, -2.0, pre_occupied=False),
        ParkingSlot("A3", -7.0,  4.0, row_a_face_yaw, row_a_approach_x,  4.0, pre_occupied=True),
        ParkingSlot("B1",  3.0, -8.0, row_b_face_yaw, row_b_approach_x, -8.0, pre_occupied=False),
        ParkingSlot("B2",  3.0, -2.0, row_b_face_yaw, row_b_approach_x, -2.0, pre_occupied=True),
        ParkingSlot("B3",  3.0,  4.0, row_b_face_yaw, row_b_approach_x,  4.0, pre_occupied=False),
    ]


# ==============================================================================
#  State machine states
# ==============================================================================

class State(enum.Enum):
    INIT              = "INIT"
    GO_TO_PICKUP      = "GO_TO_PICKUP"
    GO_TO_AISLE_ENTRY = "GO_TO_AISLE_ENTRY"
    FIND_SLOT         = "FIND_SLOT"
    GO_TO_STAGING_IN  = "GO_TO_STAGING_IN"
    GO_TO_SLOT        = "GO_TO_SLOT"
    DWELL             = "DWELL"
    BACKUP_EXIT       = "BACKUP_EXIT"
    GO_TO_STAGING_OUT = "GO_TO_STAGING_OUT"
    GO_TO_AISLE_EXIT  = "GO_TO_AISLE_EXIT"
    GO_TO_DROPOFF     = "GO_TO_DROPOFF"
    LOOP_BACK         = "LOOP_BACK"
    DONE              = "DONE"
    ABORT             = "ABORT"


# ==============================================================================
#  Helpers
# ==============================================================================

def yaw_to_quat(yaw: float):
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

# ── STRICT PATH WAYPOINTS ─────────────────────────────
WP_PICKUP       = make_pose(x= 0.0, y=-12.0, yaw= 3.1416)
WP_AISLE_ENTRY  = make_pose(x=-2.0, y=-12.0, yaw= 1.5708)
WP_STAGING_IN   = make_pose(x=-2.0, y= -2.0, yaw= 3.1416)  # face west into slot
WP_SLOT_A2      = make_pose(x=-7.0, y= -2.0, yaw= 3.1416)  # inside slot
WP_STAGING_OUT  = make_pose(x=-2.0, y= -2.0, yaw= 1.5708)  # face north after backup
WP_AISLE_EXIT   = make_pose(x=-2.0, y= 12.0, yaw= 0.0)     # top of aisle face east
WP_DROPOFF      = make_pose(x= 0.0, y= 12.0, yaw=-1.5708)  # face south for loop
WP_LOOP_BACK    = make_pose(x= 0.0, y=-12.0, yaw= 1.5708)  # back to pickup face north


# ==============================================================================
#  Parking Manager Node
# ==============================================================================

class ParkingManager(Node):

    NAV_TIMEOUT_SEC  = 120.0
    DWELL_SEC        = 5.0
    MAX_RETRIES      = 2
    TICK_HZ          = 2.0
    NO_SLOT_WAIT_SEC = 10.0

    # Phase 7: loop control
    REPEAT_MISSION  = True   # set False for single-shot
    MAX_CYCLES      = 2      # 0 = infinite, positive = fixed count
    CYCLE_PAUSE_SEC = 5.0    # seconds between cycles

    def __init__(self):
        super().__init__("parking_manager")
        self.get_logger().info("ParkingManager starting up...")

        # ── Nav action client ──────────────────────────────────────────────
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for navigate_to_pose server...")
        if not self._nav.wait_for_server(timeout_sec=60.0):
            self.get_logger().fatal("navigate_to_pose not available.")
            raise RuntimeError("Nav2 action server unavailable")
        self.get_logger().info("navigate_to_pose server ready ✅")

        # ── Phase 5: BackUp action client ──────────────────────────────────
        self._backup_client = ActionClient(self, BackUp, "/backup")

        # ── Phase 5: Costmap clear service clients ─────────────────────────
        self._clear_local  = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap")
        self._clear_global = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap")

        # ── Hard-stop publisher ────────────────────────────────────────────
        self._vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── Phase 5: /valet/status publisher ──────────────────────────────
        self._status_pub = self.create_publisher(String, "/valet/status", 10)
        self.create_timer(1.0, self._publish_status)

        # ── Phase 5: /valet/park and /valet/retrieve services ─────────────
        self.create_service(Trigger, "/valet/park",     self._park_cb)
        self.create_service(Trigger, "/valet/retrieve", self._retrieve_cb)

        # ── State machine variables ────────────────────────────────────────
        self._state        : State              = State.INIT
        self._slots                             = _build_slots()
        self._target_slot  : Optional[ParkingSlot] = None

        self._goal_handle                       = None
        self._goal_active   : bool              = False
        self._goal_result   : Optional[int]     = None
        self._retries       : int               = 0
        self._goal_sent_at                      = None

        self._dwell_start                       = None
        self._no_slot_start                     = None

        # Phase 5: backup tracking
        self._backup_active    : bool           = False
        self._backup_succeeded : bool           = False

        # Phase 7: cycle tracking
        self._cycle_count   : int               = 0
        self._cycle_start_t                     = None

        # ── Tick timer ────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / self.TICK_HZ, self._tick)
        self.get_logger().info(
            f"State machine running at {self.TICK_HZ} Hz. "
            f"Initial state: {self._state.value}"
        )

    # ──────────────────────────────────────────────────────────────────────
    #  Navigation helpers
    # ──────────────────────────────────────────────────────────────────────

    def _send_goal(self, pose: PoseStamped, label: str = ""):
        if self._goal_active:
            self.get_logger().warn("Goal already active — ignoring duplicate send.")
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        yaw_str = ""
        if "pickup" in label.lower(): yaw_str = "yaw=π"
        elif "aisle_entry" in label.lower(): yaw_str = "yaw=π/2"
        elif "staging_in" in label.lower(): yaw_str = "yaw=π"
        elif "park" in label.lower(): yaw_str = "yaw=π"
        elif "staging_out" in label.lower(): yaw_str = "yaw=π/2"
        elif "aisle_exit" in label.lower(): yaw_str = "yaw=0"
        elif "dropoff" in label.lower(): yaw_str = "yaw=-π/2"
        elif "loop" in label.lower(): yaw_str = "yaw=π/2"

        self.get_logger().info(
            f"→ {self._state.value:<18} goal: ({pose.pose.position.x:.1f}, {pose.pose.position.y:.1f}) {yaw_str}"
        )
        self._goal_active  = True
        self._goal_result  = None
        self._goal_sent_at = self.get_clock().now()
        future = self._nav.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._goal_response_callback(f, label))

    def _goal_response_callback(self, future, label: str):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f"Goal '{label}' REJECTED ❌")
            self._goal_active = False
            self._goal_result = GoalStatus.STATUS_ABORTED
            return
        self.get_logger().info(f"Goal '{label}' accepted ✅")
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._result_callback(f, label))

    def _result_callback(self, future, label: str):
        status = future.result().status
        self._goal_result = status
        self._goal_active = False
        self._goal_handle = None
        ok = status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            f"Goal '{label}' {'SUCCEEDED ✅' if ok else f'FAILED status={status} ❌'}"
        )

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().warn("Cancelling active goal...")
            self._goal_handle.cancel_goal_async()
        self._goal_active = False
        self._goal_result = None
        self._goal_handle = None

    def _goal_timed_out(self) -> bool:
        if not self._goal_active or self._goal_sent_at is None:
            return False
        elapsed = (self.get_clock().now() - self._goal_sent_at).nanoseconds / 1e9
        return elapsed > self.NAV_TIMEOUT_SEC

    def _stop(self):
        t = Twist()
        for _ in range(5):
            self._vel_pub.publish(t)

    # ── Phase 5: BackUp helpers ────────────────────────────────────────────

    def _send_backup(self, distance_m: float = 0.6, speed: float = 0.1):
        if not self._backup_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("BackUp server unavailable — skipping.")
            self._backup_active    = False
            self._backup_succeeded = True   # treat as success so mission continues
            return
        goal = BackUp.Goal()
        goal.target           = Point(x=float(distance_m), y=0.0, z=0.0)
        goal.speed            = float(speed)
        goal.time_allowance   = RosDuration(sec=30)
        self._backup_active    = True
        self._backup_succeeded = False
        self.get_logger().info(f"[←] BackUp {distance_m}m at {speed}m/s")
        fut = self._backup_client.send_goal_async(goal)
        fut.add_done_callback(self._backup_response_cb)

    def _backup_response_cb(self, future):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("BackUp goal REJECTED ❌")
            self._backup_active    = False
            self._backup_succeeded = False
            return
        self.get_logger().info("BackUp goal accepted ✅")
        handle.get_result_async().add_done_callback(self._backup_result_cb)

    def _backup_result_cb(self, future):
        status = future.result().status
        self._backup_active    = False
        self._backup_succeeded = (status == GoalStatus.STATUS_SUCCEEDED)
        self.get_logger().info(
            f"BackUp {'SUCCEEDED ✅' if self._backup_succeeded else 'FAILED ❌'}"
        )

    def _clear_costmaps(self):
        for client, label in [
            (self._clear_local,  "local"),
            (self._clear_global, "global"),
        ]:
            if client.wait_for_service(timeout_sec=3.0):
                client.call_async(ClearEntireCostmap.Request())
                self.get_logger().info(f"Cleared {label} costmap.")
            else:
                self.get_logger().warn(f"Costmap clear service ({label}) unavailable.")

    # ── Phase 5: /valet/status publisher ──────────────────────────────────

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            "state":       self._state.name,
            "cycle":       self._cycle_count,
            "target_slot": self._target_slot.name if self._target_slot else None,
            "nav_active":  self._goal_active,
            "retries":     self._retries,
            "slots": {
                s.name: "occupied" if s.occupied else "free"
                for s in self._slots
            },
        })
        self._status_pub.publish(msg)

    # ── Phase 5: /valet/park and /valet/retrieve services ─────────────────

    def _park_cb(self, request, response):
        idle_states = (State.DONE, State.ABORT, State.INIT)
        if self._state in idle_states:
            self._cycle_count = 0
            self._reset_for_next_cycle()
            response.success = True
            response.message = "Loop restarted from cycle 1."
        else:
            response.success = False
            response.message = (
                f"Active: state={self._state.name} "
                f"cycle={self._cycle_count + 1}"
            )
        return response

    def _retrieve_cb(self, request, response):
        response.success = True
        response.message = "Retrieve acknowledged (stub)."
        return response

    # ──────────────────────────────────────────────────────────────────────
    #  State machine tick
    # ──────────────────────────────────────────────────────────────────────

    def _tick(self):
        # Timeout watchdog
        if self._goal_active and self._goal_timed_out():
            self.get_logger().error(
                f"[{self._state.value}] Goal timed out after "
                f"{self.NAV_TIMEOUT_SEC:.0f} s — cancelling."
            )
            self._cancel_goal()
            self._goal_result = GoalStatus.STATUS_ABORTED

        {
            State.INIT              : self._state_init,
            State.GO_TO_PICKUP      : self._state_go_to_pickup,
            State.GO_TO_AISLE_ENTRY : self._state_go_to_aisle_entry,
            State.FIND_SLOT         : self._state_find_slot,
            State.GO_TO_STAGING_IN  : self._state_go_to_staging_in,
            State.GO_TO_SLOT        : self._state_go_to_slot,
            State.DWELL             : self._state_dwell,
            State.BACKUP_EXIT       : self._state_backup_exit,
            State.GO_TO_STAGING_OUT : self._state_go_to_staging_out,
            State.GO_TO_AISLE_EXIT  : self._state_go_to_aisle_exit,
            State.GO_TO_DROPOFF     : self._state_go_to_dropoff,
            State.LOOP_BACK         : self._state_loop_back,
            State.DONE              : self._state_done,
            State.ABORT             : self._state_abort,
        }[self._state]()

    # ── INIT ──────────────────────────────────────────────────────────────

    def _state_init(self):
        self._cycle_count   = 0
        self._cycle_start_t = self.get_clock().now()
        self.get_logger().info("== MISSION LOOP STARTING ==")
        self._reset_for_next_cycle()

    # ── GO_TO_PICKUP ──────────────────────────────────────────────────────
    def _state_go_to_pickup(self):
        if not self._goal_active and self._goal_result is None:
            self._send_goal(WP_PICKUP, label="pickup")
            return
        if self._goal_active: return
        if self._goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[✓] At PICKUP")
            self._retries = 0
            self._goal_result = None
            self._transition(State.GO_TO_AISLE_ENTRY)
        else:
            self._handle_nav_failure("GO_TO_PICKUP", State.GO_TO_PICKUP, State.ABORT)

    def _state_go_to_aisle_entry(self):
        if not self._goal_active and self._goal_result is None:
            self._send_goal(WP_AISLE_ENTRY, label="aisle_entry")
            return
        if self._goal_active: return
        if self._goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[✓] At aisle entry")
            self._retries = 0
            self._goal_result = None
            self._transition(State.FIND_SLOT)
        else:
            self._handle_nav_failure("GO_TO_AISLE_ENTRY", State.GO_TO_AISLE_ENTRY, State.ABORT)

    # ── FIND_SLOT ─────────────────────────────────────────────────────────
    def _state_find_slot(self):
        free = [s for s in self._slots if not s.occupied]
        if free:
            self._target_slot   = free[0]
            self._no_slot_start = None
            self.get_logger().info(f"Slot {self._target_slot.name} is free. Targeting.")
            self._retries = 0
            self._transition(State.GO_TO_STAGING_IN)
        else:
            self.get_logger().warn("No free slots. Waiting 10s...")
            time.sleep(10.0)
            free2 = [s for s in self._slots if not s.occupied]
            if free2:
                self._target_slot = free2[0]
                self._no_slot_start = None
                self._retries = 0
                self._transition(State.GO_TO_STAGING_IN)
            else:
                self.get_logger().error("All slots occupied. Skipping to DROPOFF.")
                if self.REPEAT_MISSION:
                    self._transition(State.GO_TO_DROPOFF)
                else:
                    self._transition(State.ABORT)
                self._retries = 0

    def _state_go_to_staging_in(self):
        if not self._goal_active and self._goal_result is None:
            self._send_goal(WP_STAGING_IN, label="staging_in")
            return
        if self._goal_active: return
        if self._goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[✓] At staging. Facing slot.")
            self._retries = 0
            self._goal_result = None
            self._transition(State.GO_TO_SLOT)
        else:
            self._handle_nav_failure("GO_TO_STAGING_IN", State.GO_TO_STAGING_IN, State.ABORT)

    # ── GO_TO_SLOT ────────────────────────────────────────────────────────
    def _state_go_to_slot(self):
        if not self._goal_active and self._goal_result is None:
            self._send_goal(WP_SLOT_A2, label="park")
            return
        if self._goal_active: return
        if self._goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[✓] In slot {self._target_slot.name}. Marking occupied.")
            self._target_slot.occupied = True
            self._retries = 0
            self._goal_result = None
            self._dwell_start = self.get_clock().now()
            self._transition(State.DWELL)
        else:
            self._handle_nav_failure("GO_TO_SLOT", State.GO_TO_SLOT, State.FIND_SLOT)

    # ── DWELL ─────────────────────────────────────────────────────────────
    def _state_dwell(self):
        elapsed   = (self.get_clock().now() - self._dwell_start).nanoseconds / 1e9
        remaining = self.DWELL_SEC - elapsed
        if remaining > 0:
            self.get_logger().info(f"DWELL {remaining:.1f}s...", throttle_duration_sec=1.0)
            return
        self.get_logger().info("[✓] Dwell complete.")
        self._retries          = 0
        self._backup_active    = False
        self._backup_succeeded = False
        self._transition(State.BACKUP_EXIT)

    # ── BACKUP_EXIT ───────────────────────────────────────────────────────
    def _state_backup_exit(self):
        if self._backup_active:
            return
        if not self._backup_succeeded:
            if self._retries > self.MAX_RETRIES:
                self.get_logger().warn("BackUp failed repeatedly — clearing costmaps and continuing.")
                self._clear_costmaps()
                self._retries = 0
                self._transition(State.GO_TO_STAGING_OUT)
                return
            self._send_backup(distance_m=5.0, speed=0.15)
            self._retries += 1
        else:
            self.get_logger().info("[✓] Backed out of slot.")
            self._retries = 0
            self._transition(State.GO_TO_STAGING_OUT)

    def _state_go_to_staging_out(self):
        if not self._goal_active and self._goal_result is None:
            self._send_goal(WP_STAGING_OUT, label="staging_out")
            return
        if self._goal_active: return
        if self._goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[✓] Facing north. Resuming aisle.")
            self._retries = 0
            self._goal_result = None
            self._transition(State.GO_TO_AISLE_EXIT)
        else:
            self._handle_nav_failure("GO_TO_STAGING_OUT", State.GO_TO_STAGING_OUT, State.ABORT)

    def _state_go_to_aisle_exit(self):
        if not self._goal_active and self._goal_result is None:
            self._send_goal(WP_AISLE_EXIT, label="aisle_exit")
            return
        if self._goal_active: return
        if self._goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[✓] At top of aisle.")
            self._retries = 0
            self._goal_result = None
            self._transition(State.GO_TO_DROPOFF)
        else:
            self._handle_nav_failure("GO_TO_AISLE_EXIT", State.GO_TO_AISLE_EXIT, State.ABORT)

    # ── GO_TO_DROPOFF ─────────────────────────────────────────────────────
    def _state_go_to_dropoff(self):
        if not self._goal_active and self._goal_result is None:
            self._send_goal(WP_DROPOFF, label="dropoff")
            return
        if self._goal_active: return
        if self._goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[✓] At DROPOFF. Cycle {self._cycle_count+1} complete.")
            self._cycle_count += 1
            self._print_summary()
            self._retries = 0
            self._goal_result = None
            if self.REPEAT_MISSION:
                self._transition(State.LOOP_BACK)
            else:
                self._transition(State.DONE)
        else:
            self._handle_nav_failure("GO_TO_DROPOFF", State.GO_TO_DROPOFF, State.DONE)

    def _state_loop_back(self):
        if not self._goal_active and self._goal_result is None:
            self._send_goal(WP_LOOP_BACK, label="loop_back")
            return
        if self._goal_active: return
        if self._goal_result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[✓] Back at PICKUP. Starting cycle {self._cycle_count+1}.")
            self._retries = 0
            self._goal_result = None
            self._reset_for_next_cycle()
        else:
            self._handle_nav_failure("LOOP_BACK", State.LOOP_BACK, State.ABORT)

    # ── DONE ──────────────────────────────────────────────────────────────

    def _state_done(self):
        self._cycle_count += 1
        self._print_summary()
        if self.MAX_CYCLES > 0 and self._cycle_count >= self.MAX_CYCLES:
            self.get_logger().info(
                f"All {self.MAX_CYCLES} cycle(s) complete. Shutting down."
            )
            self._timer.cancel()
            rclpy.shutdown()
            return
        if not self.REPEAT_MISSION:
            self.get_logger().info("REPEAT_MISSION=False. Shutting down.")
            self._timer.cancel()
            rclpy.shutdown()
            return
        self.get_logger().info(
            f"[Cycle {self._cycle_count} done] "
            f"Pausing {self.CYCLE_PAUSE_SEC}s before next cycle..."
        )
        time.sleep(self.CYCLE_PAUSE_SEC)
        self._reset_for_next_cycle()

    # ── ABORT ─────────────────────────────────────────────────────────────

    def _state_abort(self):
        self.get_logger().error(
            f"Cycle {self._cycle_count + 1} FAILED. "
            f"Completed before failure: {self._cycle_count}. Shutting down safely."
        )
        self._print_summary()
        self._stop()
        self._timer.cancel()
        rclpy.shutdown()

    # ──────────────────────────────────────────────────────────────────────
    #  Shared helpers
    # ──────────────────────────────────────────────────────────────────────

    # ── Phase 7: cycle helpers ────────────────────────────────────────────

    def _reset_for_next_cycle(self):
        """Reset state for a fresh mission cycle (slots, nav vars, timer)."""
        for s in self._slots:
            s.occupied = s.pre_occupied   # restore SDF initial state
        self._target_slot      = None
        self._retries          = 0
        self._goal_active      = False
        self._goal_result      = None
        self._goal_handle      = None
        self._backup_active    = False
        self._backup_succeeded = False
        self._no_slot_start    = None
        self._cycle_start_t    = self.get_clock().now()
        self._state            = State.GO_TO_PICKUP
        self.get_logger().info(f"══ CYCLE {self._cycle_count + 1} START ══")

    def _print_summary(self):
        occupied = [s.name for s in self._slots if s.occupied]
        free     = [s.name for s in self._slots if not s.occupied]
        elapsed  = (
            (self.get_clock().now() - self._cycle_start_t).nanoseconds / 1e9
            if self._cycle_start_t else 0.0
        )
        self.get_logger().info("======================================")
        self.get_logger().info(
            f"  CYCLE {self._cycle_count} COMPLETE | Duration: {elapsed:.1f}s"
        )
        self.get_logger().info(
            f"  Slot used : {self._target_slot.name if self._target_slot else 'none'}"
        )
        self.get_logger().info(f"  Occupied  : {occupied}")
        self.get_logger().info(f"  Free      : {free}")
        self.get_logger().info(f"  Cycles done: {self._cycle_count}")
        self.get_logger().info("======================================")

    def _transition(self, new_state: State):
        self.get_logger().info(
            f"Transition: {self._state.value} -> {new_state.value}"
        )
        self._state = new_state
        if not self._goal_active:
            self._goal_result = None

    def _handle_nav_failure(self, context: str,
                             retry_state: State,
                             abort_state: State):
        if self._retries < self.MAX_RETRIES:
            self._retries += 1
            self.get_logger().warn(
                f"[{context}] Navigation failed — "
                f"retry {self._retries}/{self.MAX_RETRIES}"
            )
            self._goal_result = None
            self._transition(retry_state)
        else:
            self.get_logger().error(
                f"[{context}] Failed after {self.MAX_RETRIES} retries — "
                f"moving to {abort_state.value}"
            )
            # Phase 5: clear costmaps on repeated failure
            self._clear_costmaps()
            self._retries     = 0
            self._goal_result = None
            self._transition(abort_state)


# ==============================================================================
#  Entry point
# ==============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ParkingManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Mission interrupted by user.")
            try:
                node._stop()
            except Exception:
                pass
    except RuntimeError as e:
        print(f"[FATAL] {e}")
    finally:
        if node:
            try:
                node._stop()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
