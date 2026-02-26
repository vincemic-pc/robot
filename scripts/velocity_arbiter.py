#!/usr/bin/env python3
"""
Velocity Arbiter — Single /cmd_vel publisher with safety filter, heading PID & velocity PID.

This node is the SOLE publisher to /cmd_vel on the robot. All other nodes publish
VelocityRequest messages to /cmd_vel_request (or Twist to /cmd_vel_nav2 for Nav2).

Control pipeline per 50 Hz cycle:
  1. Priority arbiter selects the winning request
  2. Heading PID (opt-in via target_heading)
  3. Velocity PID (opt-in via use_velocity_pid)
  4. Safety filter LAST — non-bypassable final gate
  5. Ackerman constraint
  6. Publish to /cmd_vel

See: docs/plans/007-velocity-arbiter.md
"""

import math
import json
import time
from dataclasses import dataclass, field
from typing import Optional, Dict

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from robot_interfaces.msg import VelocityRequest


# ---------------------------------------------------------------------------
# PIDController (Task 1.6)
# ---------------------------------------------------------------------------

class PIDController:
    """Generic PID controller with integral anti-windup."""

    def __init__(self, kp: float, ki: float, kd: float,
                 max_output: float, anti_windup_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.anti_windup_limit = anti_windup_limit

        self._integral = 0.0
        self._prev_error = 0.0
        self._initialized = False

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        """Compute PID correction given setpoint and measurement.

        Returns a correction value clamped to [-max_output, +max_output].
        """
        if dt <= 0.0:
            return 0.0

        error = setpoint - measurement

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup clamp
        self._integral += error * dt
        self._integral = max(-self.anti_windup_limit,
                             min(self.anti_windup_limit, self._integral))
        i_term = self.ki * self._integral

        # Derivative (skip on first call — no previous error)
        if self._initialized:
            d_term = self.kd * (error - self._prev_error) / dt
        else:
            d_term = 0.0
            self._initialized = True

        self._prev_error = error

        output = p_term + i_term + d_term
        return max(-self.max_output, min(self.max_output, output))

    def reset(self):
        """Clear integral and previous error state."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialized = False


# ---------------------------------------------------------------------------
# HeadingController (Task 1.7)
# ---------------------------------------------------------------------------

class HeadingController:
    """Heading PID wrapper — handles angle wrapping (shortest path around +/-pi)."""

    def __init__(self, kp: float, ki: float, kd: float, max_angular: float):
        self.pid = PIDController(kp, ki, kd, max_output=max_angular,
                                 anti_windup_limit=1.0)
        self.max_angular = max_angular

    def compute(self, target_heading: float, current_heading: float,
                dt: float) -> float:
        """Compute angular velocity to steer toward target_heading.

        Normalises error to [-pi, pi] for shortest-path rotation.
        """
        error = self._normalize_angle(target_heading - current_heading)
        # Feed normalised error directly — setpoint=error, measurement=0
        # This avoids wrap-around issues inside the generic PID.
        correction = self.pid.compute(error, 0.0, dt)
        return max(-self.max_angular, min(self.max_angular, correction))

    def reset(self):
        self.pid.reset()

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


# ---------------------------------------------------------------------------
# VelocityController (Task 1.8)
# ---------------------------------------------------------------------------

class VelocityController:
    """Velocity PID using two PIDController instances (linear + angular).

    Compares commanded velocity against measured velocity from a feedback topic.
    Gracefully disables when feedback is stale.
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 max_correction: float, feedback_timeout_s: float):
        self.linear_pid = PIDController(kp, ki, kd, max_output=max_correction,
                                        anti_windup_limit=0.5)
        self.angular_pid = PIDController(kp, ki, kd, max_output=max_correction,
                                         anti_windup_limit=0.5)
        self.feedback_timeout_s = feedback_timeout_s

        self.measured_linear = 0.0
        self.measured_angular = 0.0
        self._last_feedback_time: Optional[float] = None

    def update_feedback(self, linear: float, angular: float):
        """Called from the feedback topic callback with measured velocities."""
        self.measured_linear = linear
        self.measured_angular = angular
        self._last_feedback_time = time.monotonic()

    def feedback_fresh(self) -> bool:
        """Return True if feedback was received within timeout."""
        if self._last_feedback_time is None:
            return False
        return (time.monotonic() - self._last_feedback_time) < self.feedback_timeout_s

    def compute_linear(self, commanded: float, dt: float) -> float:
        """Return linear velocity correction (additive)."""
        if not self.feedback_fresh():
            return 0.0
        return self.linear_pid.compute(commanded, self.measured_linear, dt)

    def compute_angular(self, commanded: float, dt: float) -> float:
        """Return angular velocity correction (additive)."""
        if not self.feedback_fresh():
            return 0.0
        return self.angular_pid.compute(commanded, self.measured_angular, dt)

    def reset(self):
        self.linear_pid.reset()
        self.angular_pid.reset()


# ---------------------------------------------------------------------------
# SafetyState (helper dataclass for SafetyFilter output)
# ---------------------------------------------------------------------------

@dataclass
class SafetyState:
    emergency: bool = False
    stopped: bool = False
    slowdown_factor: float = 1.0
    reason: str = ""
    front_distance: float = float('inf')
    back_distance: float = float('inf')


# ---------------------------------------------------------------------------
# SafetyFilter (Task 1.5)
# ---------------------------------------------------------------------------

class SafetyFilter:
    """Non-bypassable obstacle-based safety filter.

    Ported from voice_mapper.py _compute_safe_velocity() (lines 2062-2099)
    and scan_callback() sector logic (lines 1060-1067).

    Behavioral change vs existing code: existing uses absolute slow_speed=0.06 as the
    floor during proportional slowdown; this arbiter uses min_speed_factor (default 0.3)
    as a proportional floor instead — scales correctly across speed ranges.
    """

    def __init__(self, emergency_distance: float, min_distance: float,
                 slow_distance: float, min_speed_factor: float,
                 scan_timeout_s: float):
        self.emergency_distance = emergency_distance
        self.min_distance = min_distance
        self.slow_distance = slow_distance
        self.min_speed_factor = min_speed_factor
        self.scan_timeout_s = scan_timeout_s

        # Sector distances (updated each cycle from /scan)
        self.sector_distances: Dict[str, float] = {
            "front": float('inf'),
            "front_right": float('inf'),
            "front_left": float('inf'),
            "front_wide": float('inf'),
            "left": float('inf'),
            "right": float('inf'),
            "back": float('inf'),
        }
        self._last_scan_time: Optional[float] = None

    def update_scan(self, scan: LaserScan):
        """Process a LaserScan and update sector distances.

        Ported from voice_mapper.py scan_callback() lines 1033-1122.
        Uses index-based sector slicing matching the original code.
        """
        if not scan.ranges:
            return

        ranges = np.array(scan.ranges)
        num_points = len(ranges)

        max_range = scan.range_max if scan.range_max > 0 else 12.0
        min_range = max(scan.range_min, 0.12) if scan.range_min > 0 else 0.12

        # Clean: replace inf/nan/self-body reflections with max_range
        ranges_clean = np.where(
            np.isinf(ranges) | np.isnan(ranges) | (ranges < min_range),
            max_range, ranges)

        # LiDAR: index 0 = front (0°), counter-clockwise
        # angle_per_point = 360.0 / num_points  (typically 0.5° for 720 points)
        angle_per_point = 360.0 / num_points
        front_half_angle = 45  # degrees
        front_points = int(front_half_angle / angle_per_point)

        # Sector definitions — index-based, matching voice_mapper scan_callback
        sectors = {
            "front": (0, front_points),
            "front_right": (num_points - front_points, num_points),
            "front_left": (front_points, front_points * 2),
            "left": (int(num_points * 0.2), int(num_points * 0.35)),
            "right": (int(num_points * 0.65), int(num_points * 0.8)),
            "back": (int(num_points * 0.4), int(num_points * 0.6)),
        }

        for sector, (start, end) in sectors.items():
            if start < end:
                sector_ranges = ranges_clean[start:end]
            else:
                sector_ranges = np.concatenate(
                    [ranges_clean[start:], ranges_clean[:end]])

            if len(sector_ranges) > 0:
                self.sector_distances[sector] = float(
                    np.percentile(sector_ranges, 10))

        # Narrow emergency arc: +/-30° (5th percentile for conservatism)
        emergency_half_angle = 30
        emergency_points = int(emergency_half_angle / angle_per_point)
        emergency_start = num_points - emergency_points
        emergency_end = emergency_points
        front_wide = np.concatenate(
            [ranges_clean[emergency_start:], ranges_clean[:emergency_end]])
        if len(front_wide) > 0:
            self.sector_distances["front_wide"] = float(
                np.percentile(front_wide, 5))

        self._last_scan_time = time.monotonic()

    def scan_fresh(self) -> bool:
        """Return True if scan data was received within timeout."""
        if self._last_scan_time is None:
            return False
        return (time.monotonic() - self._last_scan_time) < self.scan_timeout_s

    def apply(self, twist: Twist) -> tuple:
        """Apply safety filtering to a Twist command.

        Returns (filtered_twist, SafetyState).
        Ported from _compute_safe_velocity() lines 2062-2099.
        """
        state = SafetyState()
        linear = twist.linear.x
        angular = twist.angular.z

        # If scan data is stale, safety stop
        if not self.scan_fresh():
            state.emergency = True
            state.reason = "scan_stale"
            return self._zero_twist(), state

        front_dist = self.sector_distances.get("front", float('inf'))
        front_wide = self.sector_distances.get("front_wide", float('inf'))
        fl = self.sector_distances.get("front_left", float('inf'))
        fr = self.sector_distances.get("front_right", float('inf'))
        back_dist = self.sector_distances.get("back", float('inf'))

        min_front = min(front_dist, front_wide, fl, fr)
        state.front_distance = min_front
        state.back_distance = back_dist

        # Emergency stop: obstacle in narrow front arc < emergency_distance
        if min_front < self.emergency_distance:
            state.emergency = True
            state.reason = f"emergency_front_{min_front:.2f}m"
            return self._zero_twist(), state

        # Forward obstacle check: stop forward motion if < min_distance
        if linear > 0 and min_front < self.min_distance:
            state.stopped = True
            state.reason = f"obstacle_front_{min_front:.2f}m"
            return self._zero_twist(), state

        # Rear obstacle check: block reverse if back < min_distance
        if linear < 0 and back_dist < self.min_distance:
            state.stopped = True
            state.reason = f"obstacle_back_{back_dist:.2f}m"
            return self._zero_twist(), state

        # Proportional slowdown for forward motion
        result = Twist()
        result.linear.x = linear
        result.angular.z = angular

        if linear > 0 and min_front < self.slow_distance:
            denom = self.slow_distance - self.min_distance
            if denom > 0:
                speed_factor = (min_front - self.min_distance) / denom
                speed_factor = max(self.min_speed_factor, speed_factor)
            else:
                speed_factor = self.min_speed_factor
            result.linear.x = linear * speed_factor
            state.slowdown_factor = speed_factor

        return result, state

    @staticmethod
    def _zero_twist() -> Twist:
        return Twist()


# ---------------------------------------------------------------------------
# PriorityArbiter (Task 1.4)
# ---------------------------------------------------------------------------

@dataclass
class TimestampedRequest:
    """A VelocityRequest with a monotonic receive time for expiry tracking."""
    request: VelocityRequest = field(default_factory=VelocityRequest)
    receive_time: float = 0.0


class PriorityArbiter:
    """Priority-based velocity request arbiter.

    Maintains one active request per priority level. On each cycle, selects the
    highest-priority (lowest number) non-expired request.
    """

    def __init__(self, default_timeout_s: float):
        self.default_timeout_s = default_timeout_s
        self._requests: Dict[int, TimestampedRequest] = {}

    def update(self, request: VelocityRequest):
        """Store/replace a request at its priority level."""
        self._requests[int(request.priority)] = TimestampedRequest(
            request=request,
            receive_time=time.monotonic(),
        )

    def select(self) -> Optional[VelocityRequest]:
        """Return the highest-priority non-expired request, or None."""
        now = time.monotonic()
        expired_keys = []

        best: Optional[VelocityRequest] = None
        best_priority = 255

        for priority, entry in self._requests.items():
            duration = entry.request.duration_s
            if duration <= 0.0:
                duration = self.default_timeout_s

            age = now - entry.receive_time
            if age > duration:
                expired_keys.append(priority)
                continue

            if priority < best_priority:
                best_priority = priority
                best = entry.request

        for key in expired_keys:
            del self._requests[key]

        return best

    def clear(self):
        """Remove all active requests."""
        self._requests.clear()


# ---------------------------------------------------------------------------
# VelocityArbiter Node (Tasks 1.3, 1.9, 1.10)
# ---------------------------------------------------------------------------

class VelocityArbiter(Node):
    """ROS2 node — sole /cmd_vel publisher with safety filter, heading PID & velocity PID."""

    def __init__(self):
        super().__init__('velocity_arbiter')

        # -- Declare ROS2 parameters with defaults --
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('safety.emergency_distance', 0.3)
        self.declare_parameter('safety.min_distance', 0.5)
        self.declare_parameter('safety.slow_distance', 1.0)
        self.declare_parameter('safety.min_speed_factor', 0.3)
        self.declare_parameter('safety.scan_timeout_s', 1.0)
        self.declare_parameter('heading_pid.kp', 1.0)
        self.declare_parameter('heading_pid.ki', 0.0)
        self.declare_parameter('heading_pid.kd', 0.2)
        self.declare_parameter('heading_pid.max_angular', 0.5)
        self.declare_parameter('velocity_pid.kp', 2.0)
        self.declare_parameter('velocity_pid.ki', 0.5)
        self.declare_parameter('velocity_pid.kd', 0.1)
        self.declare_parameter('velocity_pid.feedback_topic', '/odom_raw')
        self.declare_parameter('velocity_pid.feedback_timeout_s', 0.5)
        self.declare_parameter('velocity_pid.max_correction', 0.1)
        self.declare_parameter('arbitration.request_timeout_s', 0.5)
        self.declare_parameter('ackerman.min_linear_for_turn', 0.05)

        # -- Read parameters --
        control_rate = self.get_parameter('control_rate_hz').value

        emergency_dist = self.get_parameter('safety.emergency_distance').value
        min_dist = self.get_parameter('safety.min_distance').value
        slow_dist = self.get_parameter('safety.slow_distance').value
        min_speed_factor = self.get_parameter('safety.min_speed_factor').value
        scan_timeout = self.get_parameter('safety.scan_timeout_s').value

        h_kp = self.get_parameter('heading_pid.kp').value
        h_ki = self.get_parameter('heading_pid.ki').value
        h_kd = self.get_parameter('heading_pid.kd').value
        h_max = self.get_parameter('heading_pid.max_angular').value

        v_kp = self.get_parameter('velocity_pid.kp').value
        v_ki = self.get_parameter('velocity_pid.ki').value
        v_kd = self.get_parameter('velocity_pid.kd').value
        feedback_topic = self.get_parameter('velocity_pid.feedback_topic').value
        feedback_timeout = self.get_parameter('velocity_pid.feedback_timeout_s').value
        v_max_corr = self.get_parameter('velocity_pid.max_correction').value

        request_timeout = self.get_parameter('arbitration.request_timeout_s').value
        self.min_linear_for_turn = self.get_parameter(
            'ackerman.min_linear_for_turn').value

        # -- Instantiate components --
        self.arbiter = PriorityArbiter(default_timeout_s=request_timeout)
        self.safety_filter = SafetyFilter(
            emergency_distance=emergency_dist,
            min_distance=min_dist,
            slow_distance=slow_dist,
            min_speed_factor=min_speed_factor,
            scan_timeout_s=scan_timeout,
        )
        self.heading_ctrl = HeadingController(h_kp, h_ki, h_kd, h_max)
        self.velocity_ctrl = VelocityController(
            v_kp, v_ki, v_kd,
            max_correction=v_max_corr,
            feedback_timeout_s=feedback_timeout,
        )

        # -- Heading state (from /odom) --
        self.current_heading = 0.0
        self._heading_valid = False

        # -- QoS profiles (matching voice_mapper patterns) --
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # -- Subscriptions --
        self.create_subscription(
            VelocityRequest, '/cmd_vel_request',
            self._cmd_vel_request_callback, reliable_qos)

        self.create_subscription(
            Twist, '/cmd_vel_nav2',
            self._nav2_cmd_vel_callback, reliable_qos)

        self.create_subscription(
            LaserScan, '/scan',
            self._scan_callback, sensor_qos)

        self.create_subscription(
            Odometry, '/odom',
            self._odom_callback, sensor_qos)

        self.create_subscription(
            Odometry, feedback_topic,
            self._feedback_callback, sensor_qos)

        self.create_subscription(
            Imu, '/imu/data',
            self._imu_callback, sensor_qos)

        # -- Publishers --
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self.status_pub = self.create_publisher(String, '/arbiter/status',
                                                QoSProfile(
                                                    reliability=ReliabilityPolicy.BEST_EFFORT,
                                                    history=HistoryPolicy.KEEP_LAST,
                                                    depth=1))

        # -- Timers --
        control_period = 1.0 / control_rate
        self.create_timer(control_period, self._control_callback)
        self.create_timer(1.0, self._status_callback)

        # -- Diagnostic state for status publishing --
        self._last_active_source = ""
        self._last_active_priority = -1
        self._last_safety_state = SafetyState()
        self._last_heading_error = 0.0
        self._last_vel_error_linear = 0.0
        self._last_vel_error_angular = 0.0
        self._last_dt = control_period

        self.get_logger().info(
            f"Velocity arbiter started — {control_rate} Hz, "
            f"safety [{emergency_dist}/{min_dist}/{slow_dist}]m, "
            f"feedback: {feedback_topic}")

    # -----------------------------------------------------------------------
    # Subscription callbacks
    # -----------------------------------------------------------------------

    def _cmd_vel_request_callback(self, msg: VelocityRequest):
        self.arbiter.update(msg)

    def _nav2_cmd_vel_callback(self, msg: Twist):
        """Wrap Nav2 Twist as a NAVIGATION-priority VelocityRequest."""
        request = VelocityRequest()
        request.header.stamp = self.get_clock().now().to_msg()
        request.twist = msg
        request.priority = VelocityRequest.PRIORITY_NAVIGATION
        request.source = "nav2"
        request.duration_s = 0.2
        request.target_heading = float('nan')
        request.use_velocity_pid = True
        self.arbiter.update(request)

    def _scan_callback(self, msg: LaserScan):
        self.safety_filter.update_scan(msg)

    def _odom_callback(self, msg: Odometry):
        """Extract yaw from odometry quaternion for heading PID."""
        q = msg.pose.pose.orientation
        # yaw from quaternion: atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)
        self._heading_valid = True

    def _feedback_callback(self, msg: Odometry):
        """Extract measured linear/angular velocity from feedback topic."""
        self.velocity_ctrl.update_feedback(
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )

    def _imu_callback(self, msg: Imu):
        """Backup heading source — only used if /odom hasn't been received."""
        if self._heading_valid:
            return
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)

    # -----------------------------------------------------------------------
    # Main control loop — 50 Hz (Task 1.9)
    # -----------------------------------------------------------------------

    def _control_callback(self):
        dt = self._last_dt

        # 1. Arbiter: select winning request
        request = self.arbiter.select()
        if request is None:
            self._publish_zero()
            self._last_active_source = ""
            self._last_active_priority = -1
            return

        self._last_active_source = request.source
        self._last_active_priority = int(request.priority)

        twist = Twist()
        twist.linear.x = request.twist.linear.x
        twist.angular.z = request.twist.angular.z

        # 2. Heading PID (opt-in) — runs on raw requested twist
        if not math.isnan(request.target_heading):
            heading_error = HeadingController._normalize_angle(
                request.target_heading - self.current_heading)
            self._last_heading_error = heading_error
            angular_correction = self.heading_ctrl.compute(
                request.target_heading, self.current_heading, dt)
            twist.angular.z = angular_correction
        else:
            self._last_heading_error = 0.0

        # 3. Velocity PID — compares commanded vs measured
        if request.use_velocity_pid and self.velocity_ctrl.feedback_fresh():
            lin_corr = self.velocity_ctrl.compute_linear(twist.linear.x, dt)
            ang_corr = self.velocity_ctrl.compute_angular(twist.angular.z, dt)
            self._last_vel_error_linear = twist.linear.x - self.velocity_ctrl.measured_linear
            self._last_vel_error_angular = twist.angular.z - self.velocity_ctrl.measured_angular
            twist.linear.x += lin_corr
            twist.angular.z += ang_corr
        else:
            self._last_vel_error_linear = 0.0
            self._last_vel_error_angular = 0.0

        # 4. Safety filter LAST — non-bypassable final gate
        twist, safety = self.safety_filter.apply(twist)
        self._last_safety_state = safety

        if safety.emergency or safety.stopped:
            self._publish_zero()
            return

        # 5. Ackerman constraint
        if (abs(twist.angular.z) > 0.1
                and abs(twist.linear.x) < self.min_linear_for_turn):
            twist.linear.x = math.copysign(
                self.min_linear_for_turn, twist.linear.x or 1.0)

        # 6. Publish
        self.cmd_vel_pub.publish(twist)

    def _publish_zero(self):
        self.cmd_vel_pub.publish(Twist())

    # -----------------------------------------------------------------------
    # Diagnostic status — 1 Hz (Task 1.10)
    # -----------------------------------------------------------------------

    def _status_callback(self):
        safety = self._last_safety_state
        status = {
            "active_source": self._last_active_source,
            "priority": self._last_active_priority,
            "safety_state": safety.reason if safety.reason else "ok",
            "safety_emergency": safety.emergency,
            "safety_stopped": safety.stopped,
            "safety_slowdown": round(safety.slowdown_factor, 3),
            "front_distance": round(safety.front_distance, 3),
            "back_distance": round(safety.back_distance, 3),
            "heading_error": round(self._last_heading_error, 4),
            "velocity_error_linear": round(self._last_vel_error_linear, 4),
            "velocity_error_angular": round(self._last_vel_error_angular, 4),
            "measured_linear": round(self.velocity_ctrl.measured_linear, 4),
            "measured_angular": round(self.velocity_ctrl.measured_angular, 4),
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = VelocityArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero on shutdown for safety
        node._publish_zero()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
