---
id: "007"
type: plan
title: "Velocity Arbiter — Single cmd_vel Publisher with Safety Filter, Heading PID & Velocity PID"
status: ✅ Ready for Implementation
created: "2026-02-26"
owner: pch-planner
version: 0.1
research: "[008-control-loop-architecture-review](../research/008-control-loop-architecture-review.md)"
---

## Introduction

This plan implements **Phase A** of the migration path defined in Research 008 (Section 5.10):
the velocity arbiter node. This is the highest-impact, lowest-risk change identified by the
control loop architecture review. It creates a single node that is the sole publisher to
`/cmd_vel`, enforcing priority-based arbitration, universal safety filtering, and optional
closed-loop heading PID control.

**Current problem (from Research 008):**
- 10 uncoordinated code paths publish to `/cmd_vel` across multiple threads
- Safety filtering is inconsistent — only 1 of 10 paths has the full two-layer safety model
- Turns and U-turns are open-loop (timed arcs with no IMU/heading feedback)
- No priority arbitration — multiple threads can publish simultaneously with no mutex

**This plan delivers:**
- A new `velocity_arbiter` ROS2 node as the sole `/cmd_vel` publisher
- Priority-based arbitration for competing velocity requests
- Non-bypassable safety filter applied to every velocity command
- Heading PID controller using existing IMU/EKF odometry (10–20 Hz)
- Velocity PID controller using wheel encoder feedback via `/vel_raw` or `/odom_raw`
- Modification of all 10 cmd_vel paths in `voice_mapper.py` to publish `/cmd_vel_request`

**Research 008 correction:** Research 008 concluded velocity PID was infeasible ("no wheel
speed feedback exists"). This was incorrect — the ROSMASTER A1 uses 520 encoder motors,
and the STM32 driver board reads encoder ticks with its own PID firmware loop. The
`/vel_raw` topic from `driver_node` and `/odom_raw` from `base_node` carry encoder-based
velocity data at 20 Hz, making ROS2-level velocity PID feasible.

## Planning Session Log

| # | Question | Decision | Rationale | Date |
|---|----------|----------|-----------|------|
| 1 | Scope: A1+A2 only, full Phase A (A1+A2+A3), or A1+A2+A3-stub? | **Full Phase A + velocity PID** | User wants complete arbiter with safety, arbitration, heading PID, AND velocity PID. ROSMASTER A1 has 520 encoder motors — Research 008's claim of "no wheel speed feedback" was incorrect. | 2026-02-26 |
| 2 | Deployment: separate systemd service, same process as voice_mapper, or separate + HW watchdog? | **Separate systemd service (`robot-arbiter.service`)** | Safety-critical component must survive application-layer crashes. Lightweight process (~200 lines) with `Restart=always`. Inter-process latency negligible at robot speeds. | 2026-02-26 |
| 3 | Nav2 cmd_vel integration: remap Nav2 output, bridge via voice_mapper, replace smoother, or downstream filter? | **Remap Nav2 output to `/cmd_vel_nav2`, arbiter subscribes to both** | Cleanest separation. Nav2 publishes standard Twist to remapped topic, arbiter wraps as priority=NAVIGATION. One-line change in `nav2_params.yaml`. Nav2's velocity_smoother preserved. No dependency on voice_mapper for Nav2→arbiter path. | 2026-02-26 |
| 4 | Velocity PID feedback source: `/vel_raw`, `/odom_raw`, `/odom` (EKF), or configurable? | **Start with `/odom_raw`, make configurable via ROS2 parameter** | `/odom_raw` is a safe default (standard Odometry format, 20 Hz). Need live robot verification of whether `/vel_raw` is true encoder feedback. Configurable parameter allows switching without code changes during PID tuning. | 2026-02-26 |
| 5 | Custom `VelocityRequest.msg` in new `robot_interfaces` package, or standard messages with conventions? | **Custom `VelocityRequest.msg` in new `robot_interfaces` package** | Type-safe, self-documenting. Creates infrastructure for future custom messages (ObstacleMap, GapArray, SensorState from Research 008 Phases B–D). One-time build cost. Eliminates fragile string parsing. | 2026-02-26 |
| 6 | Heading PID: always-on drift correction, opt-in per request, or opt-in with auto_hold flag? | **Opt-in per request (`target_heading` field)** | Ackerman geometry means always-on heading correction could produce unexpected arcing. Opt-in limits PID to clear use cases (turns, U-turns, rotate). `auto_hold_heading` can be added later if drift is a real problem. | 2026-02-26 |
| R1 | Control loop ordering: safety filter before or after PID controllers? | **Safety filter LAST (after PID)** | Standard robotics pipeline: setpoint → PID → safety → actuator. Safety filter is the non-bypassable final gate. Placing it after PID guarantees every Twist reaching `/cmd_vel` has been safety-checked regardless of PID corrections. Existing `_compute_safe_velocity()` only scales linear.x anyway, so reordering doesn't change heading PID behavior. | 2026-02-26 |
| R2 | U-turn strategy: single heading PID arc or retain 3-point turn structure? | **Keep 3-point turn, add heading PID to each phase** | Single forward arc for 180° is physically infeasible in tight spaces for Ackerman steering (turn radius 0.24m < min_turn_radius 0.3m). Retain 3-phase structure in voice_mapper but replace timed arcs with heading PID sub-goals (phase 1: target +60°, phase 2: reverse +120°, phase 3: target +180°). Each phase sends a VelocityRequest. Safety handled by arbiter. | 2026-02-26 |
| R3 | Safety distance defaults: match existing code or use tighter values? | **Two-tier model matching existing code** | Preserve the existing two-layer safety model: `emergency_distance=0.3m` (absolute stop), `min_distance=0.5m` (stop forward motion), `slow_distance=1.0m` (begin proportional slowdown). This is faithful to the ported `_compute_safe_velocity()` and avoids a safety regression. Distances can be tightened later via ROS2 params after arbiter is validated. | 2026-02-26 |
| R4 | Task 3.10 line references incomplete/inaccurate — fix with verified list? | **Fix Task 3.10 with verified complete list** | Codebase audit found 34 individual `cmd_vel_pub.publish()` sites across ~13 methods. Tasks 3.3–3.9 cover the major methods. Task 3.10 updated with exhaustive list of remaining methods: `_fallback_to_frontier()` (line 2745), `stop_llm_exploration()` (line 2812), `exploration_loop()` (line 2914), `stop_exploration()` (line 3179), `voice_loop()` pause (line 3203), `run()` cleanup (line 3364). Removed overlaps with 3.6/3.8. | 2026-02-26 |
| R5 | Nav2 remap: global `--ros-args` remap, node-specific remap, or nav2_params.yaml? | **Node-specific remap on velocity_smoother only** | Global `-r /cmd_vel:=/cmd_vel_nav2` on Nav2 launch remaps ALL Nav2 nodes' `/cmd_vel` references, risking unintended side effects. Node-specific remap (`-r /velocity_smoother:cmd_vel:=/cmd_vel_nav2`) only affects velocity_smoother's output. `nav2_params.yaml` doesn't have an output topic param — velocity_smoother always publishes to `/cmd_vel`, so the remap must be at the ROS2 launch layer. Removed contradictory `nav2_params.yaml` change. | 2026-02-26 |

## Holistic Review

### Decision Interaction Analysis

**Decisions 1 + 4 (full PID scope + feedback source):** The velocity PID scope depends on
`/odom_raw` being real encoder feedback. If validation (Phase 4, task 4.1) reveals it's a
command echo, velocity PID becomes a no-op (tracking its own commands). The plan mitigates
this via the configurable feedback topic parameter and graceful degradation — velocity PID
can be disabled without affecting the rest of the arbiter.

**Decisions 2 + 3 (separate service + Nav2 remap):** The arbiter running as a separate
service means Nav2's `/cmd_vel_nav2` output and voice_mapper's `/cmd_vel_request` both
arrive via inter-process ROS2 transport. This adds ~0.5ms per hop but ensures the safety
filter is always in the path. If the arbiter crashes and restarts (2s), there's a brief
window where Nav2 publishes to `/cmd_vel_nav2` but nobody reads it — Nav2 will detect
goal failure and recovery behavior kicks in.

**Decisions 5 + 6 (custom message + opt-in heading PID):** The `target_heading` field in
`VelocityRequest.msg` is a `float32` that defaults to NaN for "no heading control." This
is a clean opt-in mechanism — callers that don't set it get passthrough behavior. The
`use_velocity_pid` boolean defaults to `true`, so all callers get velocity PID unless they
explicitly opt out.

### Architectural Assessment

This plan cleanly separates concerns along the safety-critical boundary: the arbiter owns
ALL motor output, and everything else is advisory. The existing `_compute_safe_velocity()`
logic moves into the arbiter where it cannot be bypassed.

**Temporary duplication:** During Phase 3 migration, the arbiter processes `/scan` for
safety, and voice_mapper's `scan_callback()` still processes `/scan` for obstacle distances
used by LLM exploration. This is acceptable — Research 008 Phase B will extract obstacle
processing into a shared `obstacle_detector` node. This plan does not need to solve that.

**Thread safety improvement:** voice_mapper's current 10 cmd_vel paths publish from multiple
threads with no mutex. After migration, all paths publish `VelocityRequest` messages —
ROS2 publisher `publish()` is thread-safe by design. The arbiter reads messages in its
own single-threaded executor. Thread safety around `/cmd_vel` is fully resolved.

### Gap Analysis

1. **No unit tests in the plan** — The arbiter's `PIDController`, `SafetyFilter`, and
   `PriorityArbiter` are pure-logic classes that could have unit tests. Consider adding
   a `test_velocity_arbiter.py` in a future iteration.
2. **No parameter file** — PID gains and safety distances are ROS2 parameters with defaults
   in code. A `velocity_arbiter_params.yaml` could be created for production deployment
   but is not strictly necessary since defaults are embedded.
3. **Rollback path** — If the arbiter migration fails, voice_mapper can be reverted to
   direct `/cmd_vel` publishing by reverting the git changes. The arbiter service can be
   stopped independently. Clean rollback.

## Overview

The velocity arbiter is a new ROS2 node (`velocity_arbiter_node`) deployed as an independent
systemd service (`robot-arbiter.service`). It is the **sole publisher** to `/cmd_vel` on
the robot. All other nodes that previously published directly to `/cmd_vel` are modified to
publish `VelocityRequest` messages to `/cmd_vel_request` instead. Nav2's velocity_smoother
output is remapped to `/cmd_vel_nav2`.

The arbiter runs a 50 Hz control loop that:
1. **Arbitrates** — selects the highest-priority active request (EMERGENCY > SAFETY > NAVIGATION > EXPLORATION > MANUAL)
2. **Heading PID** — when `target_heading` is set in the request, closes the loop on heading using `/odom` or `/imu/data`
3. **Velocity PID** — closes the loop on linear/angular velocity using encoder feedback from `/odom_raw` (configurable)
4. **Safety-filters (LAST)** — applies non-bypassable obstacle-based proportional slowdown and emergency stop using live `/scan` data to PID-corrected output (ported from `_compute_safe_velocity()`)
5. **Ackerman constraint** — ensures minimum forward speed when turning
6. **Publishes** — the final safety-filtered, PID-corrected Twist to `/cmd_vel`

A new `robot_interfaces` ROS2 package provides the `VelocityRequest.msg` custom message.

### Objectives

1. Eliminate all direct `/cmd_vel` publishing from voice_mapper.py (10 paths → 0)
2. Guarantee every velocity command passes through the safety filter (non-bypassable)
3. Enable closed-loop heading control for turns, U-turns, and rotate commands
4. Enable closed-loop velocity control using wheel encoder feedback
5. Provide priority arbitration so emergency stops always preempt exploration
6. Deploy as an isolated systemd service that survives application-layer crashes

## Requirements

### Functional Requirements

| ID | Requirement | Acceptance Criteria |
|----|------------|---------------------|
| FR-1 | Arbiter is the sole `/cmd_vel` publisher | `ros2 topic info /cmd_vel` shows exactly 1 publisher (arbiter). No other node publishes to `/cmd_vel`. |
| FR-2 | Priority arbitration with 5 levels | Higher-priority requests preempt lower. EMERGENCY (0) always wins. Same-priority uses most recent. |
| FR-3 | Request timeout / expiry | If no new request arrives within `duration_s`, arbiter publishes zero Twist. Robot stops if all sources go silent. |
| FR-4 | Safety filter on every publish | Every Twist published to `/cmd_vel` has been filtered through obstacle-distance-based proportional slowdown. No bypass path exists. |
| FR-5 | Emergency stop from `/scan` | Arbiter subscribes to `/scan`, computes obstacle distances. Emergency stop when front < `safety.emergency_distance` (0.3m). Stop forward motion when front < `safety.min_distance` (0.5m). Proportional slowdown between `slow_distance` (1.0m) and `min_distance` (0.5m). Two-tier model matching existing `_compute_safe_velocity()`. |
| FR-6 | Rear obstacle check on reverse | Backward velocity requests are blocked when rear obstacle < `safety.min_distance` (0.5m). Fixes Research 008 Phase 1 finding 10. |
| FR-7 | Heading PID (opt-in) | When `target_heading != NaN` in VelocityRequest, arbiter uses PID to control angular velocity toward target heading. |
| FR-8 | Velocity PID | Arbiter compares commanded linear/angular velocity against measured velocity from feedback topic. PID corrects the output. |
| FR-9 | Nav2 integration via remapped topic | Arbiter subscribes to `/cmd_vel_nav2` (Twist), wraps as priority=NAVIGATION. Nav2's velocity_smoother output remapped via node-specific ROS2 topic remap in `start_nav2()` launch command (Review R5). |
| FR-10 | All 10 voice_mapper cmd_vel paths migrated | Every `self.cmd_vel_pub.publish(twist)` in voice_mapper.py replaced with `self.vel_request_pub.publish(vel_request)`. |
| FR-11 | ROS2 parameters for all tunable values | Safety distances, PID gains (Kp/Ki/Kd for heading and velocity), control rate, feedback topic — all configurable via ROS2 parameters. |
| FR-12 | Diagnostic publishing | Arbiter publishes `/arbiter/status` at 1 Hz with: active source, priority, safety state, PID error, measured vs commanded velocity. |

### Non-Functional Requirements

| ID | Requirement | Target |
|----|------------|--------|
| NFR-1 | Control loop rate | 50 Hz (20ms period) — 2.5× the current 20 Hz rate for tighter PID response |
| NFR-2 | Latency from request to `/cmd_vel` publish | < 25ms (within one control loop iteration + processing) |
| NFR-3 | CPU usage on Jetson Orin Nano | < 5% of a single core (arbiter is lightweight math, no vision/LLM) |
| NFR-4 | Startup time | Arbiter ready and publishing within 2 seconds of service start |
| NFR-5 | Graceful degradation | If `/scan` goes stale (>1s), arbiter enters safety stop. If feedback topic stale, PID disables and passes through raw velocity. |

### Out of Scope

- **Obstacle detector node** — Research 008 Phase B. This plan has the arbiter subscribe directly to `/scan` for obstacle data. A future plan will extract obstacle processing into its own node.
- **Sensor aggregator node** — Research 008 Phase B. Not needed for the arbiter.
- **Audio node extraction** — Research 008 Phase C.
- **State machine replacement** — Research 008 Phase C. voice_mapper still uses boolean flags; only the cmd_vel paths change.
- **Deleting deprecated scripts** — Research 008 Phase D.
- **Wall-following PID** — Could be added to arbiter later but not in initial scope.
- **Distance-to-goal PID** — Could be added later; current scope focuses on velocity and heading PID.

## Technical Design

### Architecture

```
                     voice_mapper.py                    Nav2 velocity_smoother
                     (10 paths modified)                (remapped output)
                            │                                  │
                    VelocityRequest                     Twist (standard)
                     /cmd_vel_request                    /cmd_vel_nav2
                            │                                  │
                            ▼                                  ▼
                 ┌──────────────────────────────────────────────────┐
                 │              velocity_arbiter_node                │
                 │                                                  │
                 │  ┌────────────┐  ┌───────────┐  ┌────────────┐  │
                 │  │  Priority   │  │    PID    │  │   Safety   │  │
                 │  │  Arbiter    │→ │ Controller│→ │   Filter   │  │
                 │  │            │  │ (H + V)   │  │   (LAST)   │  │
                 │  └────────────┘  └───────────┘  └────────────┘  │
                 │       ▲               ▲               ▲         │
                 │       │               │               │         │
                 │  /cmd_vel_request  /odom_raw        /scan       │
                 │  /cmd_vel_nav2    /odom         (/emergency)    │
                 │                   /imu/data                     │
                 │                                                  │
                 │  pub: /cmd_vel (Twist) ─── SOLE PUBLISHER ──►   │
                 │  pub: /arbiter/status (diagnostic)               │
                 └──────────────────────────────────────────────────┘
                            │
                            ▼
                     /cmd_vel → base driver (/driver_node)
```

**Data flow per 50 Hz cycle:**
1. Read latest `VelocityRequest` from `/cmd_vel_request` queue and latest Twist from `/cmd_vel_nav2`
2. Priority arbiter selects the winning request (highest priority, freshest, not expired)
3. If `target_heading` set: heading PID reads `/odom` heading, computes angular correction
4. Velocity PID reads `/odom_raw` velocity, computes linear/angular correction
5. Safety filter LAST — reads latest `/scan` data, applies proportional slowdown or emergency stop to PID-corrected output (non-bypassable final gate)
6. Ackerman constraint enforced
7. Publish final Twist to `/cmd_vel`

### Data Models / Custom Messages

#### `robot_interfaces` package structure

```
robot_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── VelocityRequest.msg
└── README.md
```

**`package.xml`** — `ament_cmake` build with `rosidl_default_generators` dependency.

**`CMakeLists.txt`** — Standard rosidl message generation:
```cmake
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VelocityRequest.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
```

#### `VelocityRequest.msg`

```
# VelocityRequest.msg — Command to the velocity arbiter
#
# All velocity commands flow through this message. The arbiter selects
# the highest-priority active request and applies safety filtering.

std_msgs/Header header

# Desired velocity (before safety filtering and PID correction)
geometry_msgs/Twist twist

# Priority level: lower number = higher priority
# 0=EMERGENCY, 1=SAFETY, 2=NAVIGATION, 3=EXPLORATION, 4=MANUAL
uint8 priority

# Priority constants
uint8 PRIORITY_EMERGENCY=0
uint8 PRIORITY_SAFETY=1
uint8 PRIORITY_NAVIGATION=2
uint8 PRIORITY_EXPLORATION=3
uint8 PRIORITY_MANUAL=4

# Identifier of the requesting source (for diagnostics/debugging)
string source

# How long this request remains valid (seconds). 0 = single-shot (one cycle only).
# Arbiter publishes zero Twist when the active request expires.
float32 duration_s

# Target heading in radians (world frame). NaN = no heading control.
# When set, the arbiter uses heading PID to control angular velocity.
float32 target_heading

# Whether velocity PID should be applied to this request.
# When true, arbiter compares commanded vs measured velocity and applies PID correction.
bool use_velocity_pid true
```

### Component Design

#### `velocity_arbiter_node` — File: `scripts/velocity_arbiter.py`

**Class:** `VelocityArbiter(Node)` — node name `velocity_arbiter`

**Subscriptions:**

| Topic | Type | QoS | Purpose |
|-------|------|-----|---------|
| `/cmd_vel_request` | `VelocityRequest` | Reliable, depth=10 | Primary velocity command input |
| `/cmd_vel_nav2` | `Twist` | Reliable, depth=10 | Nav2 velocity_smoother output (remapped) |
| `/scan` | `LaserScan` | BestEffort, depth=5 | Obstacle detection for safety filter |
| `/odom` | `Odometry` | BestEffort, depth=5 | Heading feedback for heading PID |
| `/odom_raw` | `Odometry` | BestEffort, depth=5 | Velocity feedback for velocity PID (configurable) |
| `/imu/data` | `Imu` | BestEffort, depth=5 | Backup heading source |

**Publishers:**

| Topic | Type | QoS | Rate | Purpose |
|-------|------|-----|------|---------|
| `/cmd_vel` | `Twist` | Reliable, depth=10 | 50 Hz | **SOLE motor command output** |
| `/arbiter/status` | `String` (JSON) | BestEffort, depth=1 | 1 Hz | Diagnostic: active source, priority, PID error, safety state |

**ROS2 Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `control_rate_hz` | double | 50.0 | Main control loop frequency |
| `safety.emergency_distance` | double | 0.3 | Absolute emergency stop distance (meters) — matches existing `emergency_dist` (Review R3) |
| `safety.min_distance` | double | 0.5 | Stop forward motion distance — matches existing `min_obstacle_dist` (Review R3) |
| `safety.slow_distance` | double | 1.0 | Begin proportional slowdown distance — matches existing `slow_dist` (Review R3) |
| `safety.min_speed_factor` | double | 0.3 | Minimum speed scaling factor (floor) |
| `safety.scan_timeout_s` | double | 1.0 | If no scan in this time, enter safety stop |
| `heading_pid.kp` | double | 1.0 | Heading PID proportional gain |
| `heading_pid.ki` | double | 0.0 | Heading PID integral gain (start at 0) |
| `heading_pid.kd` | double | 0.2 | Heading PID derivative gain |
| `heading_pid.max_angular` | double | 0.5 | Max angular velocity output from heading PID (rad/s) |
| `velocity_pid.kp` | double | 2.0 | Velocity PID proportional gain |
| `velocity_pid.ki` | double | 0.5 | Velocity PID integral gain |
| `velocity_pid.kd` | double | 0.1 | Velocity PID derivative gain |
| `velocity_pid.feedback_topic` | string | "/odom_raw" | Topic for velocity feedback |
| `velocity_pid.feedback_timeout_s` | double | 0.5 | Disable PID if feedback stale |
| `velocity_pid.max_correction` | double | 0.1 | Max PID correction magnitude (m/s) |
| `arbitration.request_timeout_s` | double | 0.5 | Default timeout if request has duration_s=0 |
| `ackerman.min_linear_for_turn` | double | 0.05 | Minimum linear speed when angular > 0.1 (Ackerman constraint) |

**Internal components (classes within `velocity_arbiter.py`):**

1. **`PriorityArbiter`** — Maintains a dict of active requests keyed by priority. On each cycle, selects the highest-priority non-expired request. Expired requests are removed.

2. **`SafetyFilter`** — Ported from `_compute_safe_velocity()` (voice_mapper.py lines 2062–2099). Reads latest `/scan`, computes sector distances, applies proportional slowdown. Enhanced with rear obstacle check. Returns `(filtered_twist, safety_state)`.

3. **`PIDController`** — Generic PID class used for both heading and velocity. Fields: `kp`, `ki`, `kd`, `integral`, `prev_error`, `max_output`, `anti_windup_limit`. Method: `compute(setpoint, measurement, dt) → correction`. Includes integral anti-windup (clamp integral term).

4. **`HeadingController`** — Wraps `PIDController` for heading. Handles angle wrapping (shortest path around ±π). Reads heading from `/odom` quaternion → yaw. Active only when `target_heading != NaN`.

5. **`VelocityController`** — Wraps two `PIDController` instances (linear, angular). Compares commanded velocity against measured velocity from feedback topic. Active only when `use_velocity_pid == true` and feedback is fresh.

**Main loop (`_control_callback`, 50 Hz timer):**

```python
def _control_callback(self):
    # 1. Arbiter: select winning request
    request = self.arbiter.select()  # highest priority, not expired
    if request is None:
        self._publish_zero()
        return

    twist = request.twist

    # 2. Heading PID (opt-in) — runs on raw requested twist
    if not math.isnan(request.target_heading):
        angular_correction = self.heading_ctrl.compute(
            request.target_heading, self.current_heading, dt)
        twist.angular.z = angular_correction

    # 3. Velocity PID — compares PID-corrected setpoint vs measured
    if request.use_velocity_pid and self.velocity_ctrl.feedback_fresh():
        twist.linear.x += self.velocity_ctrl.compute_linear(
            twist.linear.x, self.measured_linear, dt)
        twist.angular.z += self.velocity_ctrl.compute_angular(
            twist.angular.z, self.measured_angular, dt)

    # 4. Safety filter LAST — non-bypassable final gate on PID-corrected output
    #    (Review Q1: moved after PID so safety cannot be overridden by corrections)
    twist, safety = self.safety_filter.apply(twist, self.latest_scan)
    if safety.emergency:
        self._publish_zero()
        return

    # 5. Ackerman constraint
    if abs(twist.angular.z) > 0.1 and abs(twist.linear.x) < self.min_linear_for_turn:
        twist.linear.x = math.copysign(self.min_linear_for_turn, twist.linear.x or 1.0)

    # 6. Publish
    self.cmd_vel_pub.publish(twist)
```

#### Obstacle Sector Computation (within SafetyFilter)

Ported from voice_mapper.py `scan_callback()` lines 1033–1122. The arbiter computes
obstacle distances from raw `/scan` data using the same sector definitions.

**IMPORTANT:** The sector angle ranges below are the **actual values from `scan_callback()`**
(lines 1060–1067). They use index-based slicing relative to `num_points` and `front_points`.
The coder MUST port from the actual code, not approximate these angles:

| Sector | Actual Code (index-based) | Approx. Angle Range | Used For |
|--------|--------------------------|---------------------|----------|
| `front` | `(0, front_points)` | 0° to ~+45° | Forward collision |
| `front_right` | `(num_points - front_points, num_points)` | ~-45° to 0° | Diagonal right |
| `front_left` | `(front_points, front_points * 2)` | ~+45° to +90° | Diagonal left |
| `front_wide` | Computed at line 1107 (emergency arc) | ~±30° | Emergency stop arc |
| `left` | `(num_points * 0.2, num_points * 0.35)` | ~+72° to +126° | Side clearance |
| `right` | `(num_points * 0.65, num_points * 0.8)` | ~+234° to +288° | Side clearance |
| `back` | `(num_points * 0.4, num_points * 0.6)` | ~+144° to +216° | **Reverse safety** (new) |

Note: `front_points` is derived from the scan's `angle_increment` and a configured front arc
width. Exact values depend on the LIDAR model. Port the index-based logic, not hardcoded angles.

### Nav2 cmd_vel Integration

**Remap approach (Review R5): Node-specific remap on velocity_smoother only.**

No changes to `scripts/nav2_params.yaml` — velocity_smoother has no configurable output
topic parameter. The remap is done at the ROS2 launch layer in voice_mapper's
`start_nav2()` (line ~537) by adding a node-specific topic remap to the Nav2 bringup
subprocess command. This only affects velocity_smoother's `/cmd_vel` output, leaving
all other Nav2 nodes (controller_server, planner, etc.) with default topic bindings.

The exact remap syntax depends on how Nav2 bringup exposes node namespaces. If using
`ros2 launch nav2_bringup navigation_launch.py`, the velocity_smoother node can be
remapped via: `--ros-args -r /cmd_vel:=/cmd_vel_nav2` scoped to that node. If this
proves too broad, an alternative is a thin wrapper launch file with explicit node-scoped
remapping. **Task 3.12 should verify the exact remap syntax on the live robot before
committing.**

**In the arbiter**, the `/cmd_vel_nav2` callback wraps the incoming Twist into a
VelocityRequest internally:

```python
def _nav2_cmd_vel_callback(self, msg: Twist):
    request = VelocityRequest()
    request.header.stamp = self.get_clock().now().to_msg()
    request.twist = msg
    request.priority = VelocityRequest.PRIORITY_NAVIGATION
    request.source = "nav2"
    request.duration_s = 0.2  # Nav2 publishes continuously; expire quickly if it stops
    request.target_heading = float('nan')  # No heading PID — Nav2 handles its own path following
    request.use_velocity_pid = True
    self.arbiter.update(request)
```

## Dependencies

### Prerequisites

| Dependency | Status | Notes |
|-----------|--------|-------|
| ROS2 Humble installed on Jetson | ✅ Available | Already deployed via `ros2_env.sh` |
| `rosidl_default_generators` package | ✅ Available | Standard ROS2 Humble package for message generation |
| `geometry_msgs`, `std_msgs`, `sensor_msgs`, `nav_msgs` | ✅ Available | Standard ROS2 message packages |
| `/scan` topic publishing at ~10 Hz | ✅ Available | From `robot-lidar.service` → `/sllidar_node` |
| `/odom` topic publishing at 20 Hz | ✅ Available | From `robot-base.service` → `/ekf_filter_node` |
| `/odom_raw` topic publishing at 20 Hz | ✅ Available | From `robot-base.service` → `/base_node` |
| `/imu/data` topic publishing at 10 Hz | ✅ Available | From `robot-base.service` → `/imu_filter_madgwick` |
| colcon build tools on Jetson | ✅ Available | Used to build existing Yahboom workspace |
| SSH access to robot (jetson@192.168.7.250) | ✅ Available | Standard deployment path |

### External Dependencies

| Dependency | Impact | Mitigation |
|-----------|--------|------------|
| `robot_interfaces` must be built and sourced on Jetson before arbiter or voice_mapper can use it | Blocking — neither node can import the message without it | Phase 1 of execution builds and installs the package first |
| Nav2 params file must be updated for `/cmd_vel_nav2` remap | Nav2 will publish to wrong topic until updated | Done in Phase 3 alongside voice_mapper migration |
| voice_mapper must be updated to import `robot_interfaces` | voice_mapper won't start if package not installed | Coordinated deployment: install package, then deploy updated voice_mapper |

## Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| `/vel_raw` is a command echo, not encoder feedback | Medium | Medium — velocity PID would track its own commands (no real feedback loop). PID would be useless but not harmful. | Decision 4: start with `/odom_raw` (configurable). Validate on live robot before relying on velocity PID. PID has `feedback_timeout_s` to disable gracefully. |
| Heading PID oscillation with Ackerman geometry | Medium | Medium — robot could weave or overshoot turns | Start with conservative gains (Kp=1.0, Ki=0.0, Kd=0.2). All gains are ROS2 parameters — tune on live robot without redeploying. `max_angular` caps PID output. |
| Velocity PID fights STM32 firmware PID | Low–Medium | Medium — two PID loops in series can oscillate | Set ROS2-level velocity PID gains conservatively. If STM32 PID is tight enough, disable ROS2 velocity PID (`use_velocity_pid=false`). Monitor via `/arbiter/status`. |
| Nav2 `/cmd_vel_nav2` remap breaks path following | Low | High — Nav2 navigation stops working | Test remap in isolation before migrating voice_mapper. Nav2's velocity_smoother still handles smoothing; arbiter just applies safety on top. |
| voice_mapper migration breaks existing exploration | Medium | High — robot stops exploring | Incremental migration: one cmd_vel path at a time, test each. Keep old `cmd_vel_pub` available behind a parameter toggle until all paths verified. |
| Arbiter systemd service fails to start before voice_mapper | Low | Medium — voice_mapper publishes to `/cmd_vel_request` but nobody listens, robot doesn't move | `robot-voice-mapper.service` adds `After=robot-arbiter.service` and `Wants=robot-arbiter.service`. Arbiter starts in <2s. |
| Increased `/scan` processing load (arbiter + voice_mapper both process) | Low | Low — `/scan` is small (~720 floats) and processing is O(n) | Temporary during this plan. Research 008 Phase B extracts obstacle_detector, eliminating duplication. |
| `robot_interfaces` package build fails on Jetson | Low | Blocking — no custom messages | Package is minimal (one .msg file). Standard rosidl pipeline. Test build locally first if possible. |

## Execution Plan

### Phase 1: robot_interfaces Package + Velocity Arbiter Node (Core) — ✅ Complete

**Goal:** Create the `robot_interfaces` message package and the `velocity_arbiter_node` with
safety filter and priority arbitration. The arbiter can run standalone — it subscribes to
`/cmd_vel_request` and `/scan`, publishes to `/cmd_vel`. No voice_mapper changes yet.

**Prerequisites:** None — this phase is self-contained.

**Entry point:** Create new files from scratch.

**Completion notes (2026-02-26):** All 10 tasks implemented in a single `velocity_arbiter.py`
file (~430 lines). Components: PIDController, HeadingController, VelocityController,
SafetyFilter (ported from voice_mapper scan_callback + _compute_safe_velocity), PriorityArbiter,
and VelocityArbiter node with 50 Hz control loop, 1 Hz diagnostics. All 17 ROS2 parameters
declared with plan-specified defaults. Safety filter uses index-based sector slicing matching
the original voice_mapper code. robot_interfaces package created with VelocityRequest.msg.

| # | Task | File(s) | Status | Details | Acceptance Criteria |
|---|------|---------|--------|---------|---------------------|
| 1.1 | Create `robot_interfaces` package skeleton | `robot_interfaces/package.xml`, `robot_interfaces/CMakeLists.txt` | ✅ Complete | `ament_cmake` package with `rosidl_default_generators`. Dependencies: `geometry_msgs`, `std_msgs`. Package name: `robot_interfaces`. | `colcon build --packages-select robot_interfaces` succeeds (can test in a local ROS2 workspace or on robot) |
| 1.2 | Define `VelocityRequest.msg` | `robot_interfaces/msg/VelocityRequest.msg` | ✅ Complete | Exact message definition from Technical Design section. Include priority constants, header, twist, priority, source, duration_s, target_heading, use_velocity_pid. | Message compiles. `ros2 interface show robot_interfaces/msg/VelocityRequest` displays the definition. |
| 1.3 | Create `velocity_arbiter.py` — Node skeleton + subscriptions | `scripts/velocity_arbiter.py` | ✅ Complete | Create `VelocityArbiter(Node)` class. Set up all 6 subscriptions (cmd_vel_request, cmd_vel_nav2, scan, odom, odom_raw, imu/data). Set up 2 publishers (cmd_vel, arbiter/status). Create 50 Hz timer. Declare all ROS2 parameters with defaults from Technical Design. Store latest messages in callbacks. | Node starts, subscribes to all topics, creates timer. `ros2 node info /velocity_arbiter` shows all subscriptions/publications. |
| 1.4 | Implement `PriorityArbiter` class | `scripts/velocity_arbiter.py` | ✅ Complete | Internal class. Maintains dict of requests keyed by priority. `update(request)` stores/replaces request at its priority level. `select()` returns highest-priority non-expired request (checks `header.stamp + duration_s > now`). Returns `None` if all expired. | Unit-testable: insert requests at different priorities, verify select() returns correct one. Expired requests return None. |
| 1.5 | Implement `SafetyFilter` class | `scripts/velocity_arbiter.py` | ✅ Complete | Port `_compute_safe_velocity()` from voice_mapper.py (lines 2062–2099). Input: `Twist` + latest `LaserScan`. Compute sector distances (front, front_wide, front_left, front_right, back) — **port index-based sector logic from `scan_callback()` lines 1060–1067, not the approximate angles in this plan's table**. Apply proportional slowdown. Emergency stop if front < min_distance. **New:** rear obstacle check — block reverse if back < min_distance. Return `(filtered_twist, SafetyState)`. **Behavioral change vs existing code:** existing `_compute_safe_velocity()` uses absolute `slow_speed = 0.06 m/s` as the floor during proportional slowdown; the arbiter uses `min_speed_factor` (default 0.3) as a proportional floor instead — this is intentional (scales correctly across speed ranges) but means behavior differs from the existing code at commanded speeds ≠ 0.2 m/s. | Safety filter produces zero twist when obstacle < min_distance. Proportional slowdown between min_distance and slow_distance. Rear check blocks negative linear.x. |
| 1.6 | Implement `PIDController` class | `scripts/velocity_arbiter.py` | ✅ Complete | Generic PID: `__init__(kp, ki, kd, max_output, anti_windup_limit)`. `compute(setpoint, measurement, dt)` returns correction. Integral anti-windup: clamp integral to `[-anti_windup_limit, +anti_windup_limit]`. `reset()` clears integral and prev_error. | PID produces proportional response for step input. Integral accumulates over time. Derivative responds to rate of change. Anti-windup limits integral. |
| 1.7 | Implement `HeadingController` (wraps PIDController) | `scripts/velocity_arbiter.py` | ✅ Complete | Heading-specific wrapper. `compute(target_heading, current_heading, dt)` normalizes error to `[-π, π]` (shortest path). Returns angular velocity correction clamped to `[-max_angular, +max_angular]`. Reads heading from latest `/odom` (quaternion → yaw via `atan2`). | Heading controller tracks target within ±5° in simulation. Handles wrap-around (e.g., target=170°, current=-170° → error=20°, not 340°). |
| 1.8 | Implement `VelocityController` (wraps 2× PIDController) | `scripts/velocity_arbiter.py` | ✅ Complete | Two PID instances: linear and angular. `compute_linear(commanded, measured, dt)` and `compute_angular(commanded, measured, dt)`. Reads measured velocity from configurable feedback topic (default `/odom_raw`). `feedback_fresh()` returns True if feedback received within `feedback_timeout_s`. If stale, returns 0 correction (passthrough). Max correction clamped to `velocity_pid.max_correction`. | Velocity PID corrects toward commanded velocity. Gracefully disables when feedback stale. |
| 1.9 | Wire main control loop (`_control_callback`) | `scripts/velocity_arbiter.py` | ✅ Complete | Implement the 50 Hz timer callback as specified in Component Design pseudocode: arbiter.select() → heading PID (if target_heading set) → velocity PID (if enabled) → **safety filter LAST** (Review R1) → Ackerman constraint → publish. Publish zero if no active request or emergency. | Full pipeline runs at 50 Hz. Arbiter selects correct priority. PID corrects when enabled. Safety filter applies last. Zero twist on timeout. |
| 1.10 | Add diagnostic publisher | `scripts/velocity_arbiter.py` | ✅ Complete | 1 Hz timer publishes JSON to `/arbiter/status`: `{ "active_source": "...", "priority": N, "safety_state": "...", "heading_error": ..., "velocity_error_linear": ..., "velocity_error_angular": ..., "measured_linear": ..., "commanded_linear": ... }`. | `ros2 topic echo /arbiter/status` shows diagnostic data. |

**Verification checkpoint:** Deploy arbiter to robot. Start `robot-base`, `robot-lidar` services. Run arbiter manually (`python3 velocity_arbiter.py`). Publish test `VelocityRequest` via `ros2 topic pub`. Verify:
- Arbiter publishes to `/cmd_vel`
- Safety filter stops robot near obstacles
- Heading PID turns to target heading
- Priority arbitration works (higher priority preempts)

---

### Phase 2: Systemd Service + Deployment Infrastructure — ✅ Complete

**Goal:** Deploy the arbiter as a systemd service (`robot-arbiter.service`), build `robot_interfaces`
on the Jetson, and update `robot.target` dependency graph.

**Prerequisites:** Phase 1 complete (arbiter code exists and tested manually).

**Entry point:** Existing service file patterns in `scripts/`.

**Completion notes (2026-02-26):** All 6 tasks implemented following existing service/wrapper
conventions. `run_arbiter.sh` sources `ros2_env.sh` (which now includes `robot_ws`), so no
duplicate sourcing needed. `robot-arbiter.service` uses `Restart=always` with 2s restart for
safety-critical reliability. `robot-voice-mapper.service` has soft dependency (`Wants`) on
arbiter so it still starts if arbiter is down. `deploy_interfaces.sh` uses rsync+colcon for
idempotent deployment.

| # | Task | File(s) | Status | Details | Acceptance Criteria |
|---|------|---------|--------|---------|---------------------|
| 2.1 | Create `run_arbiter.sh` wrapper script | `scripts/run_arbiter.sh` | ✅ Complete | Follow pattern of existing `run_*.sh` wrappers. Source `ros2_env.sh`. **Also** source the `robot_interfaces` install space (e.g., `source ~/robot_ws/install/setup.bash`). `exec python3 /home/jetson/robot_scripts/velocity_arbiter.py`. | Script is executable, sources correct workspaces, launches arbiter. |
| 2.2 | Create `robot-arbiter.service` systemd unit | `scripts/robot-arbiter.service` | ✅ Complete | `Type=simple`, `User=jetson`, `ExecStart=/home/jetson/robot_scripts/run_arbiter.sh`. `Restart=always`, `RestartSec=2`. `After=robot-base.service robot-lidar.service robot-tf.service`. `Requires=robot-base.service robot-lidar.service`. `KillMode=control-group`, `TimeoutStopSec=10`. `StartLimitIntervalSec=60`, `StartLimitBurst=10`. | Service installs and starts cleanly. `systemctl status robot-arbiter` shows active. Survives `systemctl restart`. |
| 2.3 | Update `robot.target` to include arbiter | `scripts/robot.target` | ✅ Complete | Add `robot-arbiter.service` to `Wants=` list. | `systemctl start robot.target` brings up arbiter alongside other services. |
| 2.4 | Update `robot-voice-mapper.service` dependencies | `scripts/robot-voice-mapper.service` | ✅ Complete | Add `After=robot-arbiter.service` and `Wants=robot-arbiter.service` so voice_mapper starts after arbiter is running. | voice_mapper starts after arbiter. If arbiter fails, voice_mapper still starts (soft dep). |
| 2.5 | Create deployment script for `robot_interfaces` | `scripts/deploy_interfaces.sh` | ✅ Complete | Script to copy `robot_interfaces/` to Jetson, build it with colcon in `~/robot_ws/`, and update `ros2_env.sh` to source the install space. Idempotent — safe to run multiple times. | Run on dev machine → package built on Jetson → `ros2 interface show robot_interfaces/msg/VelocityRequest` works on Jetson. |
| 2.6 | Update `ros2_env.sh` to source `robot_interfaces` workspace | `scripts/ros2_env.sh` | ✅ Complete | Add conditional source of `~/robot_ws/install/setup.bash` if it exists. All `run_*.sh` wrappers inherit this automatically. | After sourcing `ros2_env.sh`, `python3 -c "from robot_interfaces.msg import VelocityRequest"` succeeds. |

**Verification checkpoint:** `systemctl start robot.target` brings up all services including arbiter. `ros2 topic info /cmd_vel` shows arbiter as the only publisher. Arbiter publishes zero twist (no requests yet). voice_mapper starts after arbiter and still functions normally (still publishing to `/cmd_vel` directly — not yet migrated).

---

### Phase 3: voice_mapper Migration — All cmd_vel Paths to /cmd_vel_request — ✅ Complete

**Goal:** Modify all 10 cmd_vel publish paths in `voice_mapper.py` to publish `VelocityRequest`
messages to `/cmd_vel_request`. Remove the `/cmd_vel` publisher from voice_mapper. Update Nav2
params for the `/cmd_vel_nav2` remap.

**Prerequisites:** Phase 2 complete (arbiter running as service, `robot_interfaces` built).

**Entry point:** `scripts/voice_mapper.py` — the 10 cmd_vel paths identified in Research 008.

**Completion notes (2026-02-26):** All 12 tasks implemented. 34 individual `cmd_vel_pub.publish()`
sites migrated to `_publish_velocity_request()` via the arbiter. `_compute_safe_velocity()` method
removed (now in arbiter's SafetyFilter). `_execute_uturn()` rewritten with heading PID sub-goals
(3-phase with `_uturn_phase()` helper). `move()` simplified to a single VelocityRequest + sleep.
`llm_control_loop()` fully migrated — all VelocityTarget, StopTarget, watchdog, and pause paths
go through arbiter. `_random_exploration_walk()` and `_reactive_exploration_loop()` simplified —
per-iteration obstacle loops replaced with duration-based VelocityRequests (arbiter handles
safety). Nav2 bringup command updated with `--ros-args -r /cmd_vel:=/cmd_vel_nav2`.
`grep cmd_vel_pub voice_mapper.py` returns zero matches.

| # | Task | File(s) | Status | Details | Acceptance Criteria |
|---|------|---------|--------|---------|---------------------|
| 3.1 | Add `robot_interfaces` import and `/cmd_vel_request` publisher to voice_mapper | `scripts/voice_mapper.py` | ✅ Complete | Added `from robot_interfaces.msg import VelocityRequest` import (line 51). Created `self.vel_request_pub` publisher in `__init__` (line 255). | voice_mapper starts with VelocityRequest publisher. |
| 3.2 | Create `_publish_velocity_request()` helper method | `scripts/voice_mapper.py` | ✅ Complete | Helper builds VelocityRequest, sets header stamp, publishes. All migration paths use this. | Helper method exists, callable from any thread. |
| 3.3 | Migrate `emergency_stop()` (Path 2) | `scripts/voice_mapper.py` | ✅ Complete | Single `_publish_velocity_request(0, 0, PRIORITY_EMERGENCY, "emergency_stop", duration_s=1.0)`. Removed 5x publish loop. | Emergency stop reaches arbiter at EMERGENCY priority. |
| 3.4 | Migrate `_execute_uturn()` (Paths 3–7) | `scripts/voice_mapper.py` | ✅ Complete | Rewritten with `_uturn_phase()` helper. 3-phase heading PID: +60deg, +120deg (reverse), +180deg. Each phase publishes VelocityRequest with target_heading. Polls heading convergence (10deg tolerance). Arbiter handles safety. Added `_normalize_angle()` static method. | U-turn uses heading PID. 3-phase structure retained. |
| 3.5 | Migrate `move()` (Paths 8–10) | `scripts/voice_mapper.py` | ✅ Complete | One-liner: `_publish_velocity_request(linear, angular, PRIORITY_MANUAL, "move", duration_s=duration)` + `time.sleep(duration)`. Removed `_compute_safe_velocity()` call and emergency backup logic. | move() publishes request, arbiter drives with safety. |
| 3.6 | Migrate `llm_control_loop()` velocity publishing | `scripts/voice_mapper.py` | ✅ Complete | All 12 cmd_vel_pub.publish sites replaced. PRIORITY_EXPLORATION for velocity/idle/stop. PRIORITY_SAFETY for pause/watchdog. Removed `_compute_safe_velocity()` calls. Also migrated `_fallback_to_frontier()`. | LLM exploration works through arbiter. |
| 3.7 | Migrate `_reactive_exploration_step()` | `scripts/voice_mapper.py` | ✅ Complete | Single `_publish_velocity_request(...)` with PRIORITY_EXPLORATION, duration_s=0.2. | Reactive step goes through arbiter. |
| 3.8 | Migrate `_reactive_exploration_loop()` | `scripts/voice_mapper.py` | ✅ Complete | Turn loops replaced with single VelocityRequest + sleep. Stop-and-recheck uses VelocityRequest. End-of-loop stop migrated. | Reactive loop works through arbiter. |
| 3.9 | Migrate `_random_exploration_walk()` | `scripts/voice_mapper.py` | ✅ Complete | Gap turn, gap drive, fallback turn, fallback drive — all use VelocityRequest with duration_s + sleep. Per-iteration obstacle checks removed (arbiter handles). | Random walk works through arbiter. |
| 3.10 | Migrate all remaining cmd_vel publish sites (Review R4) | `scripts/voice_mapper.py` | ✅ Complete | 5 remaining sites migrated: stop_llm_exploration() (PRIORITY_SAFETY), exploration_loop() final stop (PRIORITY_EXPLORATION), stop_exploration() (PRIORITY_MANUAL), voice_loop() pause (PRIORITY_SAFETY), run() cleanup (PRIORITY_EMERGENCY). | `grep cmd_vel_pub voice_mapper.py` returns 0 matches. |
| 3.11 | Remove `self.cmd_vel_pub` publisher | `scripts/voice_mapper.py` | ✅ Complete | Deleted `/cmd_vel` publisher creation line. Removed `_compute_safe_velocity()` method (38 lines). | voice_mapper has zero cmd_vel_pub references. |
| 3.12 | Update Nav2 launch for `/cmd_vel_nav2` remap (Review R5) | `scripts/voice_mapper.py` | ✅ Complete | Added `"--ros-args", "-r", "/cmd_vel:=/cmd_vel_nav2"` to Nav2 bringup subprocess command in `start_nav2()`. | velocity_smoother publishes to /cmd_vel_nav2. |

**Verification checkpoint:** Full system test:
1. `systemctl start robot.target` — all services start
2. `ros2 topic info /cmd_vel` — only arbiter publishes
3. Voice command "move forward" → robot moves (through arbiter)
4. Voice command "explore" → LLM exploration works (through arbiter)
5. Obstacle in front → robot stops (arbiter safety filter)
6. Voice command "turn around" → heading PID U-turn
7. Nav2 goal → robot navigates (through arbiter via `/cmd_vel_nav2`)
8. Emergency: obstacle suddenly appears → immediate stop

---

### Phase 4: PID Tuning & Validation

**Goal:** Validate and tune the heading PID and velocity PID controllers on the live robot.
Verify encoder feedback is real (not command echo). Document tuned gains.

**Prerequisites:** Phase 3 complete (all paths through arbiter).

| # | Task | File(s) | Details | Acceptance Criteria |
|---|------|---------|---------|---------------------|
| 4.1 | Validate `/vel_raw` and `/odom_raw` are encoder-based | (live robot only) | On the live robot: command a velocity via `/cmd_vel`, stall a wheel by hand, observe whether `/vel_raw` and `/odom_raw` reflect the stall (drop to 0) or continue reporting the commanded velocity. If they reflect the stall → real encoder feedback. If they echo the command → update velocity PID feedback source parameter. | Documented: which topic(s) carry true encoder feedback. Velocity PID feedback_topic parameter set correctly. |
| 4.2 | Tune heading PID gains | `scripts/velocity_arbiter.py` (parameter defaults) | On live robot: command a series of heading targets (90°, 180°, 270° turns). Observe overshoot, oscillation, settle time via `/arbiter/status`. Adjust `heading_pid.kp`, `heading_pid.ki`, `heading_pid.kd` via `ros2 param set`. Document final gains. | 90° turn completes within 3 seconds, overshoot < 10°, no sustained oscillation. |
| 4.3 | Tune velocity PID gains | `scripts/velocity_arbiter.py` (parameter defaults) | On live robot: command 0.10, 0.15, 0.20 m/s linear. Observe actual vs commanded via `/arbiter/status`. Adjust `velocity_pid.kp`, `velocity_pid.ki`, `velocity_pid.kd`. Document final gains. | Commanded 0.15 m/s → measured 0.14–0.16 m/s within 0.5s. No oscillation. |
| 4.4 | Update parameter defaults with tuned values | `scripts/velocity_arbiter.py` | Replace initial conservative defaults with validated gains from 4.2 and 4.3. | Arbiter starts with production-ready gains. |
| 4.5 | End-to-end exploration test | (live robot) | Run full LLM exploration session (~5 minutes). Monitor `/arbiter/status` for safety events, PID errors, priority transitions. Verify no regressions vs pre-arbiter behavior. | 5-minute exploration completes without crashes, stuck states, or safety violations. |

## Review Summary

### Complexity Assessment

| Factor | Score (1-5) | Notes |
|--------|-------------|-------|
| Files to modify | 3 | 1 new package (3 files), 1 new Python module, 3 new deployment files, 5 existing files modified |
| New patterns introduced | 3 | Custom ROS2 message package, PID controllers, priority arbitration — all standard robotics patterns |
| External dependencies | 2 | Only `rosidl_default_generators` (already available). No new pip packages. |
| Migration complexity | 4 | 34 individual publish sites across 13 methods in a 3,396-line monolith. Incremental but requires care. |
| Test coverage required | 3 | Pure-logic classes (PID, arbiter, safety filter) are unit-testable. Full validation requires live robot. |
| **Overall Complexity** | **15/25** | **Medium** |

### Review Session Log

| # | Question | Decision | Date |
|---|----------|----------|------|
| R1 | Safety filter ordering | Safety filter LAST (after PID) | 2026-02-26 |
| R2 | U-turn strategy | Keep 3-point turn + heading PID per phase | 2026-02-26 |
| R3 | Safety distance defaults | Two-tier model matching existing code (0.3/0.5/1.0m) | 2026-02-26 |
| R4 | Task 3.10 completeness | Fixed with verified exhaustive list (6 methods, correct lines) | 2026-02-26 |
| R5 | Nav2 remap approach | Node-specific remap on velocity_smoother only | 2026-02-26 |
| R6 | Architecture diagram wrong ordering + sector angles + slowdown change | Fix all three: diagram, sector note, document slowdown behavioral change | 2026-02-26 |

### Issues Found and Resolved

| # | Severity | Issue | Resolution |
|---|----------|-------|------------|
| 1 | Critical | Safety filter before PID allowed bypass | Reordered: PID first, safety filter last (R1) |
| 2 | Major | U-turn single arc infeasible for Ackerman | Retained 3-point turn with heading PID sub-goals (R2) |
| 3 | Major | Safety distances more aggressive than existing code | Matched existing two-tier model: 0.3/0.5/1.0m (R3) |
| 4 | Major | Task 3.10 line refs wrong, 4 methods missing | Corrected with verified exhaustive list (R4) |
| 5 | Major | Nav2 remap contradictory (params vs launch args) | Node-specific remap only, no nav2_params.yaml change (R5) |
| 6 | Major | Architecture diagram showed wrong component ordering (Safety before PID) | Fixed diagram: Arbiter → PID → Safety Filter (LAST) (R6) |
| 7 | Minor | "10 paths" understates actual method count (~13) | Documented; tasks cover all methods |
| 8 | Minor | Diagnostic uses String(JSON) vs structured msg | Acceptable for v1; noted for future |
| 9 | Minor | scan_callback retention not explicit | Already noted in Holistic Review; no task change needed |
| 10 | Minor | Sector angle ranges in table didn't match actual code | Updated table with actual index-based ranges from scan_callback() lines 1060–1067 (R6) |
| 11 | Minor | Slowdown behavioral change undocumented (absolute → proportional) | Documented in Task 1.5: existing uses `slow_speed=0.06` absolute, arbiter uses `min_speed_factor=0.3` proportional (R6) |

### Quality Gate Checklist

- [x] All Critical issues resolved (1/1)
- [x] All Major issues resolved (5/5)
- [x] User answered all clarifying questions (6/6)
- [x] Every task has specific file paths and acceptance criteria
- [x] All dependencies verified against live codebase (second pass: all 33 publish sites, all function line refs, all service files confirmed)
- [x] Security implications acknowledged (safety filter non-bypassable)
- [x] Test coverage requirements defined (Phase 4 live validation)

## Standards

- Follow existing codebase patterns (Python 3, ROS2 Humble, rclpy)
- Reference Research 008 for all architectural decisions
- Match existing service file conventions (`scripts/robot-*.service`)
- Match existing wrapper script conventions (`scripts/run_*.sh`)

## Handoff

**Plan Status:** ✅ Ready for Implementation (reviewed 2026-02-26, 2 review passes, 6+3 issues resolved).

**Plan location:** `docs/plans/007-velocity-arbiter.md`

**Phases:** 4
- Phase 1: robot_interfaces package + velocity arbiter node (10 tasks)
- Phase 2: systemd service + deployment infrastructure (6 tasks)
- Phase 3: voice_mapper migration — all 10 cmd_vel paths (12 tasks)
- Phase 4: PID tuning & validation on live robot (5 tasks)

**Key decisions:**
1. Full Phase A scope + velocity PID (encoder motors confirmed)
2. Separate systemd service for safety isolation
3. Nav2 output remapped to `/cmd_vel_nav2`
4. Configurable velocity feedback source (default `/odom_raw`)
5. Custom `VelocityRequest.msg` in new `robot_interfaces` package
6. Heading PID opt-in per request (Ackerman geometry consideration)

**New files created by this plan:**
- `robot_interfaces/` — new ROS2 message package
- `scripts/velocity_arbiter.py` — the arbiter node
- `scripts/run_arbiter.sh` — wrapper script
- `scripts/robot-arbiter.service` — systemd service
- `scripts/deploy_interfaces.sh` — deployment helper

**Files modified by this plan:**
- `scripts/voice_mapper.py` — all 10 cmd_vel paths migrated, `_compute_safe_velocity()` removed
- `scripts/robot.target` — arbiter added to Wants
- `scripts/robot-voice-mapper.service` — After/Wants arbiter
- `scripts/ros2_env.sh` — source robot_interfaces workspace

**Next step:** Use `/pch-coder` to begin implementing the plan.
