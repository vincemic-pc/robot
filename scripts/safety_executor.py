"""Safety executor for LLM-driven autonomous exploration.

Validates VLM navigation decisions against real-time sensor data before
execution.  Translates decisions into velocity targets for the control loop.
Manages blocked-action memory, watchdog degradation, and network monitoring.

Plan 006: LLM-Driven Autonomous Exploration Navigation.
"""

import time
import math
import logging
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Twist

from llm_navigator import NavigationDecision

logger = logging.getLogger(__name__)


# -- Target dataclasses (returned by SafetyExecutor for the control loop) ---

@dataclass
class VelocityTarget:
    """Direct velocity command for the control loop to apply."""
    twist: Twist
    duration_s: float

@dataclass
class NavGoalTarget:
    """Nav2 goal delegation -- control loop yields cmd_vel to Nav2."""
    x: float
    y: float
    reason: str = ""

@dataclass
class ObserveTarget:
    """Stop and observe -- control loop publishes zero twist."""
    focus: str = "general"

@dataclass
class StopTarget:
    """Intentional stop with reason."""
    reason: str = "unknown"

@dataclass
class ExecutionResult:
    """Result of executing a VLM decision."""
    success: bool
    target: object  # VelocityTarget | NavGoalTarget | ObserveTarget | StopTarget
    safety_override: bool = False
    safety_message: str = ""
    actual_distance_m: Optional[float] = None
    stopped_reason: Optional[str] = None
    check_result: Optional[dict] = None  # For check_path_clear


# -- Constants ---------------------------------------------------------------

SPEED_MAP = {"slow": 0.08, "medium": 0.12, "fast": 0.18}

# Direction -> (linear_factor, angular_rad_s)
DIRECTION_MAP = {
    "forward":       (1.0,   0.0),
    "forward_left":  (1.0,   0.25),
    "left":          (0.5,   0.45),
    "forward_right": (1.0,  -0.25),
    "right":         (0.5,  -0.45),
    "backward":      (-1.0,  0.0),
}

# Direction -> obstacle_distances keys for pre-validation
DIRECTION_TO_LIDAR = {
    "forward":       ["front", "front_wide"],
    "forward_left":  ["front", "front_left"],
    "left":          ["front_left", "left"],
    "forward_right": ["front", "front_right"],
    "right":         ["front_right", "right"],
    "backward":      ["back"],
}

ACKERMAN_MIN_LINEAR = 0.05   # m/s minimum forward when turning
ACKERMAN_ANGULAR_SPEED = 0.4  # rad/s for arc turns
REJECT_DISTANCE = 0.5        # reject move if clearance < this (matches min_obstacle_dist)
DOWNGRADE_DISTANCE = 1.0     # downgrade to "slow" if clearance < this (matches slow_dist)
BLOCKED_COOLDOWN_S = 15.0
BLOCKED_MAX_RETRIES = 2


# -- BlockedActionMemory (Task 3.1) -----------------------------------------

class BlockedActionMemory:
    """Tracks blocked actions by hash to prevent VLM retrying the same
    blocked direction more than max_retries times within cooldown_s."""

    def __init__(self, cooldown_s=BLOCKED_COOLDOWN_S, max_retries=BLOCKED_MAX_RETRIES):
        self._entries = {}  # hash_key -> {count, first_blocked, last_blocked}
        self._cooldown_s = cooldown_s
        self._max_retries = max_retries

    def _hash_key(self, decision):
        tool, params = decision.tool_name, decision.parameters
        if tool == "move_toward":
            return f"{tool}:{params.get('direction', '?')}"
        if tool == "rotate":
            return f"{tool}:{params.get('degrees', '?')}"
        if tool == "navigate_to_goal":
            return f"{tool}:{params.get('goal_type', '?')}"
        return f"{tool}:*"

    def _clean_expired(self):
        now = time.time()
        expired = [k for k, v in self._entries.items()
                   if now - v['last_blocked'] > self._cooldown_s]
        for k in expired:
            del self._entries[k]

    def should_allow(self, decision):
        """Returns (allowed: bool, warning_msg: str)."""
        self._clean_expired()
        key = self._hash_key(decision)
        entry = self._entries.get(key)
        if entry is None:
            return True, ""
        if entry['count'] >= self._max_retries:
            return False, (f"Blocked: {key} rejected {entry['count']} times "
                           f"in the last {self._cooldown_s:.0f}s")
        return True, ""

    def record_blocked(self, decision):
        key, now = self._hash_key(decision), time.time()
        entry = self._entries.get(key)
        if entry is None:
            self._entries[key] = {'count': 1, 'first_blocked': now, 'last_blocked': now}
        else:
            entry['count'] += 1
            entry['last_blocked'] = now

    def clear(self):
        self._entries.clear()


# -- LLMWatchdog (Task 3.2) -------------------------------------------------

class LLMWatchdog:
    """Evaluates elapsed time since last VLM response, returns degradation tier.

    CONTINUE (0-3s), STOP_WAIT (3-10s), LOCAL_NAV (10-30s), RETURN_HOME (>30s).
    """

    def __init__(self, continue_s=3.0, stop_s=10.0, local_nav_s=30.0):
        self._continue_s = continue_s
        self._stop_s = stop_s
        self._local_nav_s = local_nav_s
        self._last_response_time = time.time()

    def feed(self):
        """Record that a VLM response was received (resets timer)."""
        self._last_response_time = time.time()

    def evaluate(self, elapsed_s=None):
        """Return current watchdog tier string. elapsed_s overrides for testing."""
        if elapsed_s is None:
            elapsed_s = time.time() - self._last_response_time
        if elapsed_s <= self._continue_s:
            return "CONTINUE"
        if elapsed_s <= self._stop_s:
            return "STOP_WAIT"
        if elapsed_s <= self._local_nav_s:
            return "LOCAL_NAV"
        return "RETURN_HOME"


# -- NetworkMonitor (Task 3.3) ----------------------------------------------

class NetworkMonitor:
    """Tracks rolling API call success rate and latency."""

    def __init__(self, window_size=20):
        self._calls = deque(maxlen=window_size)  # (timestamp, latency_ms, success)
        self._last_success_time = time.time()

    def record_call(self, latency_ms, success):
        now = time.time()
        self._calls.append((now, latency_ms, success))
        if success:
            self._last_success_time = now

    @property
    def current_tier(self):
        """0=normal, 1=elevated (avg>3s), 2=degraded (no success 10s), 3=offline (30s)."""
        now = time.time()
        since_success = now - self._last_success_time
        if since_success > 30.0:
            return 3
        if since_success > 10.0:
            return 2
        recent = [lat for t, lat, _ in self._calls if now - t < 30.0]
        if recent and sum(recent) / len(recent) > 3000:
            return 1
        return 0

    @property
    def avg_latency_ms(self):
        if not self._calls:
            return 0.0
        now = time.time()
        recent = [lat for t, lat, _ in self._calls if now - t < 60.0]
        return sum(recent) / len(recent) if recent else 0.0

    @property
    def success_rate(self):
        if not self._calls:
            return 1.0
        now = time.time()
        recent = [s for t, _, s in self._calls if now - t < 60.0]
        return (sum(1 for s in recent if s) / len(recent)) if recent else 1.0


# -- SafetyExecutor (Tasks 3.4-3.7) -----------------------------------------

class SafetyExecutor:
    """Validates and translates VLM navigation decisions into velocity targets.

    Does NOT directly publish cmd_vel.  Returns target objects for the control
    loop to apply.  The control loop is the sole cmd_vel publisher during LLM
    exploration.
    """

    def __init__(self):
        self.blocked_memory = BlockedActionMemory()
        self.watchdog = LLMWatchdog()
        self.network_monitor = NetworkMonitor()

    # -- Pre-validation (Task 3.4) --

    def _pre_validate(self, decision, obstacle_distances):
        """Check LiDAR clearance for the commanded direction.

        Returns (valid, decision, reason).  Decision may have speed downgraded.
        """
        tool, params = decision.tool_name, decision.parameters

        if tool not in ("move_toward", "rotate"):
            return True, decision, ""

        if tool == "move_toward":
            direction = params.get("direction", "forward")
            lidar_keys = DIRECTION_TO_LIDAR.get(direction, ["front"])
            min_clearance = min(obstacle_distances.get(k, 10.0) for k in lidar_keys)

            if direction == "backward":
                back_dist = obstacle_distances.get("back", 10.0)
                if back_dist < REJECT_DISTANCE:
                    return False, decision, (
                        f"Backward blocked: obstacle at {back_dist:.2f}m")
                return True, decision, ""

            if min_clearance < REJECT_DISTANCE:
                return False, decision, (
                    f"Direction '{direction}' blocked: obstacle at {min_clearance:.2f}m")

            if min_clearance < DOWNGRADE_DISTANCE and params.get("speed") != "slow":
                modified_params = dict(params)
                modified_params["speed"] = "slow"
                modified = NavigationDecision(
                    tool_name=tool, parameters=modified_params,
                    reasoning=decision.reasoning, timestamp=decision.timestamp,
                    raw_response=decision.raw_response,
                )
                return True, modified, (
                    f"Speed downgraded to 'slow': obstacle at {min_clearance:.2f}m")

        elif tool == "rotate":
            front_dist = min(obstacle_distances.get("front", 10.0),
                             obstacle_distances.get("front_wide", 10.0))
            if front_dist < REJECT_DISTANCE:
                degrees = params.get("degrees", 0)
                side = "front_left" if degrees < 0 else "front_right"
                side_dist = obstacle_distances.get(side, 10.0)
                if side_dist < REJECT_DISTANCE:
                    return False, decision, (
                        f"Rotation blocked: front {front_dist:.2f}m, "
                        f"{side} {side_dist:.2f}m")

        return True, decision, ""

    # -- Action execution (Task 3.5) --

    def _execute_move(self, params):
        """Translate move_toward -> VelocityTarget."""
        direction = params.get("direction", "forward")
        speed = SPEED_MAP.get(params.get("speed", "medium"), 0.12)
        duration_s = max(1.0, min(8.0, float(params.get("duration_s", 3.0))))
        linear_factor, angular = DIRECTION_MAP.get(direction, (1.0, 0.0))

        twist = Twist()
        twist.linear.x = float(speed * linear_factor)
        twist.angular.z = float(angular)
        if abs(angular) > 0.1 and abs(twist.linear.x) < ACKERMAN_MIN_LINEAR:
            twist.linear.x = float(ACKERMAN_MIN_LINEAR * (1.0 if linear_factor >= 0 else -1.0))

        return VelocityTarget(twist=twist, duration_s=duration_s)

    def _execute_navigate(self, params, vm):
        """Translate navigate_to_goal -> NavGoalTarget or StopTarget."""
        goal_type = params.get("goal_type", "coordinates")
        reason = params.get("reason", "")

        if goal_type == "retrace":
            x, y = 0.0, 0.0
        elif goal_type == "coordinates":
            x, y = float(params.get("x", 0.0)), float(params.get("y", 0.0))
        elif goal_type == "frontier":
            frontier_id = params.get("frontier_id", 1)
            logger.info("Navigate to frontier #%d -- %s", frontier_id, reason)
            return NavGoalTarget(x=0.0, y=0.0,
                                 reason=f"frontier#{frontier_id}: {reason}")
        else:
            return StopTarget(reason=f"unknown_goal_type:{goal_type}")

        if vm.navigate_to(x, y):
            logger.info("Nav2 goal sent: (%.2f, %.2f) -- %s", x, y, reason)
            return NavGoalTarget(x=x, y=y, reason=reason)
        logger.warning("Nav2 goal rejected: (%.2f, %.2f)", x, y)
        return StopTarget(reason="nav2_goal_rejected")

    def _execute_rotate(self, params):
        """Translate rotate -> VelocityTarget (arc turn, Ackerman constraint)."""
        degrees = int(params.get("degrees", 90))
        duration_s = max(0.5, min(6.0, math.radians(abs(degrees)) / ACKERMAN_ANGULAR_SPEED))

        twist = Twist()
        twist.linear.x = float(ACKERMAN_MIN_LINEAR)
        # degrees < 0 = left = positive angular.z in ROS convention
        twist.angular.z = float(ACKERMAN_ANGULAR_SPEED if degrees < 0 else -ACKERMAN_ANGULAR_SPEED)
        return VelocityTarget(twist=twist, duration_s=duration_s)

    def _execute_stop(self, params):
        return StopTarget(reason=params.get("reason", "unknown"))

    # -- Perception / communication (Task 3.6) --

    def _execute_observe(self, params, vm):
        """Return ObserveTarget -- control loop stops robot & spawns vm.observe()."""
        return ObserveTarget(focus=params.get("focus", "general"))

    def _execute_check_path(self, params, obstacle_distances):
        """Query LiDAR arc in direction. Local sensor only, instant."""
        direction = params.get("direction", "forward")
        distance_m = max(0.5, min(5.0, float(params.get("distance_m", 2.0))))
        lidar_keys = DIRECTION_TO_LIDAR.get(direction, ["front"])
        min_obstacle = min(obstacle_distances.get(k, 10.0) for k in lidar_keys)

        clear = min_obstacle >= distance_m
        if min_obstacle < 0.5:
            obstacle_type = "wall"
        elif min_obstacle < 1.0:
            obstacle_type = "close_obstacle"
        elif not clear:
            obstacle_type = "distant_obstacle"
        else:
            obstacle_type = "none"

        return ExecutionResult(
            success=True,
            target=StopTarget(reason="check_path_complete"),
            check_result={"clear": clear, "obstacle_distance_m": round(min_obstacle, 2),
                          "obstacle_type": obstacle_type},
        )

    def _execute_report(self, params, vm, memory):
        """Log discovery + TTS + update memory. Speak is non-blocking."""
        label = params.get("label", "unknown")
        notes = params.get("notes", "")
        significance = params.get("significance", "point_of_interest")
        pos = vm.current_position
        position = {'x': pos.get('x', 0), 'y': pos.get('y', 0)}

        memory.record_discovery(label, notes, significance, position)

        if hasattr(vm, 'discovery_log') and vm.discovery_log is not None:
            vm.discovery_log.add_discovery(
                f"[{significance}] {label}: {notes}",
                pos.get('x', 0), pos.get('y', 0), pos.get('theta', 0), None)

        speech = f"Found {label}. {notes}" if notes else f"Found {label}."
        threading.Thread(target=vm.speak, args=(speech,), daemon=True).start()
        logger.info("Discovery reported: [%s] %s -- %s at (%.1f, %.1f)",
                     significance, label, notes, position['x'], position['y'])
        return ExecutionResult(success=True, target=StopTarget(reason="report_complete"))

    # -- Main entry point (Task 3.7) --

    def execute_decision(self, decision, vm, memory):
        """Validate and translate a VLM NavigationDecision into a target.

        Returns ExecutionResult with the appropriate target. Does NOT publish
        cmd_vel -- the control loop is the sole publisher.
        """
        tool = decision.tool_name
        obstacle_distances = getattr(vm, 'obstacle_distances', {})

        # Blocked-action check
        allowed, warning = self.blocked_memory.should_allow(decision)
        if not allowed:
            logger.warning("Blocked by memory: %s", warning)
            return ExecutionResult(success=False, target=StopTarget(reason="blocked_action"),
                                   safety_override=True, safety_message=warning)

        # Pre-validation (LiDAR clearance)
        valid, decision, reason = self._pre_validate(decision, obstacle_distances)
        if not valid:
            self.blocked_memory.record_blocked(decision)
            logger.warning("Pre-validation rejected: %s", reason)
            return ExecutionResult(success=False, target=StopTarget(reason="safety_rejected"),
                                   safety_override=True, safety_message=reason)

        so = bool(reason)  # safety_override
        params = decision.parameters

        if tool == "move_toward":
            return ExecutionResult(success=True, target=self._execute_move(params),
                                   safety_override=so, safety_message=reason)
        if tool == "rotate":
            return ExecutionResult(success=True, target=self._execute_rotate(params),
                                   safety_override=so, safety_message=reason)
        if tool == "navigate_to_goal":
            tgt = self._execute_navigate(params, vm)
            return ExecutionResult(success=not isinstance(tgt, StopTarget), target=tgt,
                                   safety_override=so, safety_message=reason)
        if tool == "stop_robot":
            return ExecutionResult(success=True, target=self._execute_stop(params))
        if tool == "observe_scene":
            return ExecutionResult(success=True, target=self._execute_observe(params, vm))
        if tool == "check_path_clear":
            return self._execute_check_path(params, obstacle_distances)
        if tool == "report_discovery":
            return self._execute_report(params, vm, memory)

        logger.error("Unknown tool: %s", tool)
        return ExecutionResult(success=False, target=StopTarget(reason=f"unknown_tool:{tool}"))
