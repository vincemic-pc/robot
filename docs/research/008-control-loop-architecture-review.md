---
id: "008"
type: research
title: "Control Loop Architecture Review — Toward Proper ROS2 Subsystem Isolation & PID Design"
status: ✅ Complete
created: "2026-02-26"
current_phase: "5 of 5"
---

## Introduction

The robot's control software has evolved organically, resulting in a hybrid architecture
that is neither a clean ROS2 node graph nor a well-structured PID control system. Key
concerns include:

- **Monolithic nodes** — `VoiceMapper` (3,396 lines) owns sensors, navigation, LLM
  orchestration, motor control, exploration logic, and speech I/O in a single class.
- **Blurred subsystem boundaries** — Sensor processing, decision-making, and actuation
  are interleaved rather than separated by ROS topics/services/actions.
- **Ad-hoc control** — Motor commands are issued via open-loop `move()` calls with
  sleep-based timing rather than closed-loop PID controllers with feedback.
- **Duplicated functionality** — Multiple scripts (`voice_mapper.py`,
  `yahboom_explorer.py`, `llm_robot_brain.py`) implement overlapping functionality
  (cmd_vel publishing, sensor subscriptions, LLM interaction).

This research will systematically audit the control architecture, map the actual data
flow, and identify concrete refactoring opportunities to achieve:

1. Proper ROS2 node decomposition with well-defined interfaces
2. Closed-loop control where appropriate (velocity, heading, obstacle avoidance)
3. Clean separation of concerns (sense → decide → act pipeline)
4. Elimination of duplicated code across scripts

## Objectives

1. Map the complete sensor-to-actuator data flow across all scripts
2. Identify every location where `/cmd_vel` is published and characterize the control strategy
3. Catalog all ROS2 subscriptions/publications/services/actions and their coupling
4. Assess the control loop timing and feedback mechanisms
5. Propose a target ROS2 node architecture with isolated subsystems
6. Identify where PID or closed-loop control should replace open-loop timed moves

## Research Phases

| Phase | Name | Status | Scope | Session |
|-------|------|--------|-------|---------|
| 1 | Monolith Anatomy — `voice_mapper.py` | ✅ Complete | Deep dive into the main 3,396-line node: all pub/sub, control paths, sensor handling, state machines | Session 1 |
| 2 | Parallel Scripts & Duplication Audit | ✅ Complete | `yahboom_explorer.py`, `llm_robot_brain.py`, `llm_navigator.py` — overlap with VoiceMapper, divergent patterns | Session 2 |
| 3 | Safety, Sensing & Decision Pipeline | ✅ Complete | `safety_executor.py`, `sensor_snapshot.py`, `exploration_memory.py` — how sensor data flows into decisions and safety checks | Session 3 |
| 4 | Systemd Services, Launch Files & Runtime Topology | ✅ Complete | Service files, launch files, actual ROS2 node graph at runtime — how subsystems are (or aren't) isolated at deployment | Session 4 |
| 5 | Target Architecture & Recommendations | ✅ Complete | Synthesis: proposed node graph, interface definitions, PID opportunities, migration path | Session 5 |

## Phase 1: Monolith Anatomy — `voice_mapper.py`

**Scope:** Deep dive into the main application file. Map every publisher, subscriber,
timer, service client, and action client. Trace every path from sensor callback to
`/cmd_vel` publish. Identify state management, threading model, and control loop timing.

**Key questions:**
- How many distinct responsibilities does `VoiceMapper` hold?
- Where are the control loops and what feedback do they use?
- What is the timing model (timers vs callbacks vs sleep-based)?
- How does the LLM decision cycle interact with real-time control?

### 1.1 File Structure

The file contains two classes and one entry point:

| Entity | Lines | Role |
|--------|-------|------|
| `DiscoveryLog` | 136–222 | Exploration logging helper (YAML persistence, path/discovery/obstacle tracking) |
| `VoiceMapper(Node)` | 225–3379 | The monolith — extends `rclpy.Node`, owns everything |
| `main()` | 3381–3396 | Entry point: `rclpy.init()`, instantiate, run, shutdown |

Supporting data types defined at module level:
- `CameraType(Enum)` — `OAK_D_PRO`, `REALSENSE` (line 82)
- `SlamMode(Enum)` — `LIDAR`, `VSLAM`, `HYBRID` (line 88)
- `CameraConfig(dataclass)` — topic configuration per camera (line 95)
- `CAMERA_CONFIGS` dict — pre-defined configs for each camera type (line 110)

### 1.2 ROS2 Interface Catalog

#### Publishers (2)

| # | Topic | Type | Line | Purpose |
|---|-------|------|------|---------|
| 1 | `/cmd_vel` | `Twist` | 254 | **Motor commands** — the ONLY actuator output |
| 2 | `/initialpose` | `PoseWithCovarianceStamped` | 477 | AMCL initial pose (Nav2) |

#### Subscribers (8)

| # | Topic | Type | QoS | Line | Callback |
|---|-------|------|-----|------|----------|
| 1 | `{rgb_topic}` (e.g. `/oak/rgb/image_raw`) | `Image` | Reliable | 282 | `camera_callback` — stores `latest_image` |
| 2 | `{depth_topic}` (e.g. `/oak/stereo/image_raw`) | `Image` | Reliable | 289 | `depth_callback` — stores `latest_depth`, clears cache |
| 3 | `{left_topic}` (e.g. `/oak/left/image_rect`) | `Image` | Reliable | 299 | `left_callback` — stores `left_image` (conditional) |
| 4 | `{right_topic}` (e.g. `/oak/right/image_rect`) | `Image` | Reliable | 303 | `right_callback` — stores `right_image` (conditional) |
| 5 | `/visual_slam/tracking/odometry` | `Odometry` | BestEffort | 310 | `vslam_odom_callback` — updates VSLAM pose/path |
| 6 | `/scan` | `LaserScan` | BestEffort | 315 | `scan_callback` — **heaviest callback**: obstacle distances, doorway detection, emergency stop |
| 7 | `/odom` | `Odometry` | BestEffort | 318 | `odom_callback` — position tracking, path recording |
| 8 | `/map` | `OccupancyGrid` | BestEffort | 321 | `map_callback` — map info, coverage calculation |

#### Action Clients (1)

| # | Action | Type | Line | Purpose |
|---|--------|------|------|---------|
| 1 | `navigate_to_pose` | `NavigateToPose` | 474 | Nav2 path-planned navigation |

#### Service Clients (4)

| # | Service | Type | Line | Creation |
|---|---------|------|------|----------|
| 1 | `/global_costmap/clear_entirely_global_costmap` | `ClearEntireCostmap` | 481 | Eager (in `_init_nav2`) |
| 2 | `/local_costmap/clear_entirely_local_costmap` | `ClearEntireCostmap` | 483 | Eager (in `_init_nav2`) |
| 3 | `/visual_slam/save_map` | `FilePath` | 1615 | Lazy (created on first use) |
| 4 | `/visual_slam/load_map` | `FilePath` | 1652 | Lazy (created on first use) |

#### ROS2 Timers

**None.** The node uses zero ROS2 timers. All periodic behavior is implemented via
sleep-based loops in background threads.

### 1.3 Responsibilities Catalog

`VoiceMapper` holds **14 distinct responsibilities**:

| # | Responsibility | Key Methods | Lines | Could Be Separate Node? |
|---|---------------|-------------|-------|------------------------|
| 1 | **Camera Abstraction** | `_detect_camera()`, `enhance_image()`, `image_to_base64()` | 1702–1923 | Yes |
| 2 | **Sensor Data Collection** | 8 subscription callbacks | 989–1211 | Yes (sensor aggregator) |
| 3 | **Obstacle Detection** | `scan_callback()` sector processing, `_detect_doorways()` | 1033–1176 | Yes (obstacle detector node) |
| 4 | **Emergency Safety** | `emergency_stop()`, emergency flag in `scan_callback` | 1113–1122, 1978–1984 | Yes (safety node) |
| 5 | **Motor Control** | `move()`, `_execute_uturn()`, `_compute_safe_velocity()` | 1978–2178 | Yes (velocity controller) |
| 6 | **Nav2 Navigation** | `_init_nav2()`, `start_nav2()`, `navigate_to()`, `cancel_navigation()`, frontier detection | 464–877 | Yes (navigation manager) |
| 7 | **SLAM Management** | `start_slam()`, `stop_slam()` via subprocess | 1392–1496 | Yes (SLAM lifecycle) |
| 8 | **VSLAM Management** | `start_isaac_vslam()`, `stop_isaac_vslam()`, save/load map | 1499–1700 | Yes (VSLAM lifecycle) |
| 9 | **Audio I/O** | `listen()`, `transcribe()`, `speak()`, `beep()`, `_find_microphone()` | 1725–1895 | Yes (audio node) |
| 10 | **LLM Brain** | `think()`, `execute()`, `_build_system_prompt()`, `get_context()` | 878–2377 | Yes (brain/orchestrator) |
| 11 | **Vision/Observation** | `observe()`, `find_object_distance()`, `get_dist()` | 1214–1974 | Yes (vision node) |
| 12 | **Exploration Orchestration** | 4 exploration modes, `exploration_loop()`, `llm_control_loop()`, `vlm_decision_loop()` | 2389–3181 | Yes (exploration manager) |
| 13 | **Discovery Logging** | `DiscoveryLog` integration, path recording | 136–222, 1192 | Yes (logger node) |
| 14 | **Process Management** | subprocess spawn/kill for SLAM, Nav2, VSLAM, ffmpeg | 492–514, 1392–1593, 516–594 | Partial (systemd handles some) |

### 1.4 All Paths to `/cmd_vel` — Control Strategy Analysis

There are **10 distinct code paths** that publish to `/cmd_vel`. This is the central
safety concern: no single point of control over the robot's sole actuator.

#### Path 1: `move()` — Open-Loop Timed Move (line 2101–2137)

```
Caller: execute() for "move" action, _reactive_exploration_loop(), _random_exploration_walk()
Strategy: Open-loop with reactive safety filter
Loop rate: 20 Hz (sleep-based)
Feedback: _compute_safe_velocity() reads obstacle_distances from LiDAR
Duration: Caller-specified (up to 15s)
```
- NOT a PID controller — no setpoint, no integral, no derivative
- `_compute_safe_velocity()` is a **reactive safety filter** that scales velocity proportionally
  to obstacle distance and enforces emergency stops

#### Path 2: `emergency_stop()` — Immediate Stop (line 1978–1984)

```
Caller: move() on emergency, scan_callback flag
Strategy: Publish zero Twist 5 times with 20ms gaps
```

#### Path 3: `_execute_uturn()` — 3-Phase Open-Loop U-Turn (line 1986–2060)

```
Caller: execute() for "turn_around" action
Strategy: Fixed timing, 3 phases (forward-arc, reverse-arc, forward-arc)
Loop rate: 20 Hz (sleep-based), 3s per phase
Feedback: Checks obstacle_distances per iteration, can abort early
```
- Completely open-loop heading control — no gyro/IMU feedback for angle

#### Path 4: `llm_control_loop()` — 20 Hz LLM Exploration Control (line 2612–2739)

```
Caller: start_llm_exploration() thread
Strategy: Rate-controlled loop applying VelocityTargets from VLM decisions
Loop rate: 20 Hz (sleep-based with rate compensation)
Feedback: _compute_safe_velocity() + watchdog timeout tiers
```
- **Closest thing to proper closed-loop control** in the codebase
- Still open-loop for velocity setpoint — no PID on actual wheel velocity
- Has duration-based target expiry, watchdog fallback, voice override pause

#### Path 5: `_reactive_exploration_step()` — Single Direct Publish (line 3053–3071)

```
Caller: _try_nav2_with_reactive_fallback() at ~10 Hz
Strategy: Single Twist publish per call
Feedback: Reads obstacle_distances for direction
```

#### Path 6: `_reactive_exploration_loop()` — Direct Publish + move() (line 3073–3153)

```
Caller: _fallback_to_frontier(), _try_nav2_with_reactive_fallback()
Strategy: Direct publish for turning, delegates to move() for forward
Feedback: obstacle_distances, detected_gaps (doorway priority)
```

#### Path 7: `_random_exploration_walk()` — Direct Publish for Doorway Traversal (line 2917–3027)

```
Caller: exploration_loop(), _reactive_exploration_loop()
Strategy: Direct publish for turning toward gaps, then drive through
Feedback: obstacle_distances, detected_gaps
```

#### Path 8: `exploration_loop()` — Nav2-Based (line 2822–2915)

```
Strategy: Delegates to Nav2 (which publishes its own cmd_vel)
But also calls: _random_exploration_walk() which publishes directly
```

#### Path 9: Voice/exploration pause — Zero Twist (lines 2628, 2738, 2903, 3151, 3178, 3363)

Multiple locations publish zero Twist for stopping: `voice_loop` pause (2628),
`llm_control_loop` end (2738), `stop_exploration()` (3178), `run()` cleanup (3363).

#### Path 10: `move()` backup-on-emergency (line 2118–2124)

```
Strategy: Publish -0.1 linear for 1 second (20 iterations) after emergency stop
No safety check on reverse direction during backup
```

### 1.5 The Safety Filter: `_compute_safe_velocity()`

Located at line 2062–2099, this is the **only shared safety mechanism** used by
both `move()` and `llm_control_loop()`. It is NOT a PID controller.

**What it does:**
1. Emergency stop check (flag from scan_callback)
2. Multi-sector front obstacle check (front, front_wide, front_left, front_right)
3. Backward obstacle check
4. Proportional slowdown: `speed_factor = (dist - min) / (slow_dist - min)`, floor at 0.3
5. Ackerman constraint: forces minimum linear speed (0.05) when angular > 0.1

**What it does NOT do:**
- No velocity PID (no comparison of commanded vs actual velocity)
- No heading PID (no target heading tracking)
- No integral wind-up protection
- No derivative for oscillation damping
- No sensor fusion (uses only LiDAR obstacle distances, not odom velocity)

### 1.6 Threading Model

| Thread | Created | Method | Daemon | Purpose |
|--------|---------|--------|--------|---------|
| Main | — | `run()` → `rclpy.spin_once()` | No | ROS2 callback processing |
| voice_loop | line 3349 | `voice_loop()` | Yes | Blocking audio I/O, LLM think/execute |
| sensor_monitor | line 3345 | `_sensor_monitor_loop()` | Yes | 2s periodic sensor health check |
| vlm_decision | line 2792 | `vlm_decision_loop()` | Yes | VLM API calls for LLM exploration |
| llm_control | line 2794 | `llm_control_loop()` | Yes | 20 Hz cmd_vel control during LLM explore |
| explore_thread | line 2747 | `exploration_loop()` | Yes | Frontier-based exploration (fallback) |
| ad-hoc observe | lines 2850, 2667 | `_observe_in_background()` / `_run_observe_for_llm()` | Yes | Background GPT-4o vision calls |

**Critical: No mutex on `/cmd_vel`.** Multiple threads can publish simultaneously:
- `llm_control_loop` (20 Hz continuous)
- `move()` (called from `execute()` via voice_loop thread)
- `emergency_stop()` (from scan_callback in main thread)
- `stop_exploration()` (from voice_loop thread)

The `llm_nav_paused` threading.Event partially mitigates this during voice commands,
but there is no global cmd_vel arbitration layer.

### 1.7 State Management

**Boolean state flags (13 flags, no state machine):**

| Flag | Line | Purpose | Set by | Cleared by |
|------|------|---------|--------|------------|
| `running` | 338 | Master on/off | `__init__` | `run()` cleanup, KeyboardInterrupt |
| `exploring` | 339 | Any exploration active | `start_exploration()` | `stop_exploration()` |
| `mapping` | 340 | SLAM active | `start_slam()` | `stop_slam()` |
| `speaking` | 341 | TTS playback active | `speak()` enter | `speak()` finally |
| `listening` | 342 | Audio recording active | `listen()` | `listen()` finally |
| `emergency_stop_triggered` | 344 | Emergency flag | `scan_callback()` | `move()` after handling |
| `nav2_available` | 466 | Nav2 stack ready | `start_nav2()` success | `stop_nav2()` |
| `navigating` | 468 | Nav2 goal in progress | `navigate_to()` | nav result callback |
| `vslam_available` | 258 | VSLAM operational | `start_isaac_vslam()` | `stop_isaac_vslam()` |
| `vslam_tracking` | 259 | VSLAM publishing odom | `vslam_odom_callback()` | `stop_isaac_vslam()` |
| `llm_exploring` | 409 | LLM exploration mode | `start_llm_exploration()` | `stop_llm_exploration()` |
| `_observing` | 418 | Background observe running | `_run_observe_for_llm()` | `_run_observe_for_llm()` finally |
| `explore_mode` | 372 | String: "idle"/"llm"/"nav2-frontier"/"reactive"/"random-walk" | various | various |

There is **no formal state machine**. Mode transitions are ad-hoc, relying on boolean
flag combinations that can lead to inconsistent states if transitions happen concurrently.

### 1.8 Exploration Mode Architecture

The node has **4 exploration modes** with fallback chains:

```
start_exploration()
  └─→ start_llm_exploration()           [mode = "llm"]
        ├─→ vlm_decision_loop (thread)
        └─→ llm_control_loop (thread)
              └─ on watchdog timeout: _fallback_to_frontier()
                   └─→ exploration_loop()            [mode = "nav2-frontier"]
                         ├─ on Nav2 failure: _random_exploration_walk()  [mode = "random-walk"]
                         └─ on no frontiers: _random_exploration_walk()

  (if Nav2 unavailable)
  └─→ _try_nav2_with_reactive_fallback()
        ├─→ _reactive_exploration_step()  [mode = "reactive", temp]
        └─→ _reactive_exploration_loop()  [mode = "reactive"]
```

### 1.9 External Process Dependencies

VoiceMapper spawns and manages external processes via `subprocess`:

| Process | Start Method | Stop Method | Health Check |
|---------|-------------|-------------|--------------|
| slam_toolbox | `Popen("ros2 launch slam_toolbox ...")` (line 1406) | `terminate()` + `wait()` | None after initial sleep(3) |
| Nav2 bringup | `Popen("ros2 launch nav2_bringup ...")` (line 542) | `terminate()` + `wait()` | Polls `bt_navigator` lifecycle state for 60s |
| Isaac VSLAM | `Popen("ros2 launch oakd_vslam.launch.py")` (line 1548) | `terminate()` + kill fallback | Waits for `vslam_tracking` flag (30s) |
| ffmpeg (audio) | `subprocess.run()` per TTS call (line 1874) | Timeout (10s) | Return code |
| aplay | `subprocess.run()` per TTS call (line 1881) | Timeout (30s) | Return code |
| pkill (cleanup) | `subprocess.run()` (line 499) | — | — |

### 1.10 Key Findings — Phase 1

1. **14 responsibilities in one class** — VoiceMapper is a textbook god object. Camera,
   sensors, safety, motor control, navigation, SLAM, VSLAM, audio, LLM, vision,
   exploration, logging, monitoring, and process management all live in one 3,396-line class.

2. **10 code paths publish to `/cmd_vel` with no arbitration** — There is no single
   "velocity commander" node or mutex. Multiple threads can publish simultaneously.
   The `llm_nav_paused` event is a partial fix for voice-vs-exploration conflicts, but
   the general case is unprotected.

3. **Zero ROS2 timers** — All periodic behavior uses sleep-based loops in Python threads.
   This bypasses ROS2's executor model and means callbacks don't benefit from ROS2's
   real-time scheduling or lifecycle management.

4. **Safety filter is reactive, not closed-loop** — `_compute_safe_velocity()` scales
   velocity based on obstacle proximity but does no PID control. There is no velocity
   feedback (commanded vs actual), no heading controller, and no integral/derivative
   terms. The robot has no way to know if its wheels are actually spinning at the
   commanded speed.

5. **13 boolean flags, no state machine** — Mode transitions are implicit, relying on
   ad-hoc flag combinations. Concurrent transitions (e.g., voice command while exploration
   mode is switching) can leave the system in inconsistent states.

6. **LLM-to-motor latency: 500–2000ms+** — VLM decision loop captures a snapshot, sends
   it to GPT-4o (API latency dominated), enqueues the result, then `llm_control_loop`
   picks it up within 50ms. Total path from sensor reading to motor command is dominated
   by API latency, making real-time reactive control impossible through the LLM path.

7. **Subprocess management is fragile** — SLAM, Nav2, and VSLAM are spawned as child
   processes with no lifecycle management, restart logic, or health monitoring beyond
   initial startup waits. If a process dies mid-operation, the node may not detect it.

8. **Open-loop heading control** — U-turns and turns are purely time-based with no
   IMU/gyro feedback. The robot cannot verify it actually turned the intended angle.

9. **Sensor monitor clears data to detect staleness** — `_sensor_monitor_loop()` sets
   `latest_scan = None` every 2 seconds (line 3259–3261). This means other code paths
   that read `latest_scan` may see `None` during the monitor's check interval — a
   potential race condition.

10. **`move()` backup-on-emergency has no reverse safety** — When emergency stop triggers,
    the backup maneuver (line 2118–2124) publishes reverse velocity for 1 second without
    checking if the rear is blocked, risking collision while reversing.

## Phase 2: Parallel Scripts & Duplication Audit

**Scope:** Analyze `yahboom_explorer.py` (1,135 lines), `llm_robot_brain.py` (618 lines),
and `llm_navigator.py` (389 lines). For each: catalog ROS interfaces, compare with
`VoiceMapper`, identify shared vs divergent patterns.

**Key questions:**
- Which scripts are active vs legacy?
- What functionality is duplicated across scripts?
- Could these be consolidated or replaced?

### 2.1 Script Status Summary

| Script | Lines | Status | Evidence |
|--------|-------|--------|----------|
| `yahboom_explorer.py` | 1,135 | **DEPRECATED** | Lines 2–4: explicit deprecation notice; hardcoded for removed HP60C camera |
| `llm_robot_brain.py` | 618 | **UNUSED PROTOTYPE** | No systemd service, no launch file, no reference from any active code |
| `llm_navigator.py` | 389 | **ACTIVE (support module)** | Imported by `voice_mapper.py`'s `vlm_decision_loop()` (Plan 006) |

A copy of `yahboom_explorer.py` and `llm_robot_brain.py` exists in `rosmaster-a1-robot/scripts/`
but these are much smaller earlier stubs (35 and 75 lines respectively) — not maintained.

### 2.2 Script Analysis: `yahboom_explorer.py` (DEPRECATED)

**Class:** `YahboomExplorer(Node)` — node name `yahboom_explorer`

#### ROS2 Interfaces

**Publishers (1):**

| Topic | Type | Line |
|-------|------|------|
| `/cmd_vel` | `Twist` | 92 |

**Subscribers (4):**

| Topic | Type | QoS | Line | Notes |
|-------|------|-----|------|-------|
| `/ascamera_hp60c/camera_publisher/rgb0/image` | `Image` | Reliable | 108 | **HP60C-specific** — camera removed |
| `/ascamera_hp60c/camera_publisher/depth0/image` | `Image` | Reliable | 115 | **HP60C-specific** — camera removed |
| `/scan` | `LaserScan` | BestEffort | 121 | Same as VoiceMapper |
| `/odom` | `Odometry` | BestEffort | 126 | Same as VoiceMapper |

**No action clients, no service clients, no timers** — far simpler than VoiceMapper.

#### Responsibilities (~8)

| # | Responsibility | Key Methods | Overlap with VoiceMapper? |
|---|---------------|-------------|--------------------------|
| 1 | Sensor callbacks | `rgb_callback`, `depth_callback`, `scan_callback`, `odom_callback` | **Yes** — nearly identical pattern |
| 2 | LiDAR obstacle detection | `scan_callback` sector processing | **Yes** — same sector approach, simpler (binary, no doorways) |
| 3 | Motor control | `move()`, `choose_direction()` | **Yes** — same open-loop pattern, simpler safety filter |
| 4 | Audio I/O | `listen()`, `transcribe()`, `speak()`, `_find_microphone()` | **Yes** — near-identical implementation |
| 5 | Vision/Observe | `observe()`, `image_to_base64()` | **Yes** — very similar (GPT-4o vision call) |
| 6 | LLM Brain | `think()`, `execute()`, `get_context()` | **Yes** — same JSON action pattern |
| 7 | Exploration | `exploration_loop()`, `start_exploration()`, `choose_direction()` | **Partial** — single simple mode vs VoiceMapper's 4 modes |
| 8 | Depth queries | `get_dist()`, `find_object_distance()`, `get_center_distance()` | **Partial** — VoiceMapper has similar `find_object_distance()`, `get_dist()` |

#### Unique Features (not in VoiceMapper)

1. **`follow_color()`** (lines 923–985) — HSV-based color tracking with LiDAR safety,
   publishes cmd_vel in a loop. VoiceMapper has no color-following capability.
2. **`DepthQuery` dataclass** — Structured depth query result with validity flag.
3. **Depth query caching** — `depth_query_cache` with 0.5s timeout to avoid redundant
   depth image processing.

#### Safety Comparison

| Aspect | VoiceMapper | YahboomExplorer |
|--------|-------------|-----------------|
| Safety filter | `_compute_safe_velocity()` — proportional slowdown | Binary check: `self.obstacles.get("front", False)` |
| Emergency stop | 5x zero publish with 20ms gaps | Single zero publish |
| Reverse safety | Backup-on-emergency (no rear check) | Checks `obstacles["back"]` |
| Sectors | front, front_wide, front_left, front_right, back | front, front_left, front_right, left, right, back |
| Doorway detection | Yes | No |

#### Threading Model

| Thread | Method | Purpose |
|--------|--------|---------|
| Main | `run()` → `rclpy.spin_once()` | ROS2 callbacks |
| voice_loop | `voice_loop()` | Blocking audio I/O + LLM |
| explore_thread | `exploration_loop()` | Autonomous exploration |

Same threading architecture as VoiceMapper, but with only 2 daemon threads vs 5+.
Same race condition: no mutex on `/cmd_vel` between voice_loop and explore_thread.

### 2.3 Script Analysis: `llm_robot_brain.py` (UNUSED PROTOTYPE)

**Class:** `LLMRobotBrain(Node)` — node name `llm_robot_brain`

This script has a fundamentally different architecture from the other two. Rather than
directly owning sensors, it is designed as a **topic-coupled orchestrator** that
communicates with external nodes via ROS topics.

#### ROS2 Interfaces

**Publishers (4):**

| Topic | Type | Line | Purpose |
|-------|------|------|---------|
| `/cmd_vel` | `Twist` | 417 | Motor commands |
| `/robot_action` | `String` | 418 | Action dispatch to executor |
| `/speech_output` | `String` | 419 | TTS output (external speech node) |
| `/llm_response` | `String` | 420 | Full JSON response broadcast |

**Subscribers (4):**

| Topic | Type | Line | Purpose |
|-------|------|------|---------|
| `/voice_text` | `String` | 423 | Transcribed speech input |
| `/text_input` | `String` | 426 | Text-based commands |
| `/camera/color/image_raw` | `Image` | 429 | Camera feed (generic topic) |
| `/robot_feedback` | `String` | 432 | Action completion feedback |

#### Key Architectural Differences from VoiceMapper

| Aspect | VoiceMapper / YahboomExplorer | LLMRobotBrain |
|--------|-------------------------------|---------------|
| Audio I/O | Direct pyaudio/Whisper/TTS | Via `/voice_text` and `/speech_output` topics |
| Sensor access | Direct subscription to all sensors | Only camera image; no LiDAR, no odometry |
| LLM provider | OpenAI only (hardcoded) | Multi-provider (OpenAI, Anthropic, Ollama, Gemini) |
| Safety | Reactive obstacle filter | **NONE** — publishes raw LLM-commanded velocities |
| Action execution | Direct method calls | Topic-based dispatch (`/robot_action`) |
| State management | 13+ boolean flags | 2 flags (`waiting_for_image`, `pending_actions`) |
| Threading | Multiple daemon threads | Single `_process_loop` thread + queue |

#### Unique Features

1. **Multi-provider LLM** (lines 210–325) — `OpenAIProvider`, `AnthropicProvider`,
   `OllamaProvider` with a common `LLMProvider` interface. Only script supporting
   non-OpenAI models.
2. **`RobotActionLibrary`** (lines 55–207) — Declarative action catalog with function
   signatures, descriptions, and examples. Includes actions not implemented anywhere
   (face_follow, color_follow, poseFollow, gestureFollow, servo control, mapping).
3. **Topic-based architecture** — Designed for a multi-node system where separate nodes
   handle speech, action execution, and feedback. VoiceMapper bundles all of these.

#### Critical Safety Gap

`_execute_velocity()` (lines 561–588) parses LLM output directly into a Twist and
publishes it with **zero safety checking**:

```python
twist.linear.x = linear_x  # Directly from LLM JSON parse
self.cmd_vel_pub.publish(twist)  # No obstacle check
```

The subsequent stop is a `time.sleep(duration)` in a daemon thread — if the main
thread crashes, the robot may continue moving indefinitely.

### 2.4 Script Analysis: `llm_navigator.py` (ACTIVE support module)

**Class:** `LLMNavigator` — **NOT a ROS2 Node**

This is a pure utility module with no ROS2 interface. It is imported and used by
`VoiceMapper.vlm_decision_loop()` as part of Plan 006 (LLM-driven exploration).

#### Design

| Aspect | Detail |
|--------|--------|
| Pattern | Stateless per call — fresh system+user message each invocation |
| LLM | OpenAI GPT-4o-mini with function calling (`tool_choice="required"`) |
| Input | `snapshot` (sensor data) + `memory` (exploration state) + optional image |
| Output | `NavigationDecision` dataclass (tool_name, parameters, reasoning, timestamp) |
| Temperature | 0.4 (lower than other scripts' 0.7–0.8) |
| Max tokens | 300 |
| Timeout | 8 seconds |

#### Tool Definitions (7 tools)

| Tool | Description | Maps to VoiceMapper |
|------|-------------|---------------------|
| `move_toward` | Direction + speed + duration move | `llm_control_loop()` → `cmd_vel` |
| `navigate_to_goal` | Nav2 path-planned navigation | `navigate_to()` |
| `rotate` | Ackerman-adapted arc turn | Direct `cmd_vel` publish |
| `stop_robot` | Intentional halt | Zero `cmd_vel` |
| `observe_scene` | Request detailed vision analysis | `_observe_in_background()` |
| `check_path_clear` | LiDAR direction query | Reads `obstacle_distances` |
| `report_discovery` | Log finding with significance | `DiscoveryLog` integration |

#### Key Design Strengths (vs other scripts)

1. **Stateless per call** — No growing conversation history. Each decision is independent,
   avoiding context window bloat and stale context.
2. **Structured output** — Uses OpenAI function calling with `strict: true`, eliminating
   the brittle JSON-in-text parsing used by `think()`/`execute()` in VoiceMapper and
   YahboomExplorer.
3. **Well-defined tool vocabulary** — 7 specific tools with typed parameters and
   constraints (speed enums, direction enums, degree quantization).
4. **Separation of concerns** — LLMNavigator only decides. It does not execute, subscribe,
   or publish. VoiceMapper's `llm_control_loop()` handles execution.

#### Limitations

1. **OpenAI-only** — Hardcoded to GPT-4o-mini. Cannot use Anthropic or local models.
2. **No safety in prompt enforcement** — Safety rules are in the system prompt but the
   robot relies entirely on VoiceMapper's `_compute_safe_velocity()` to enforce them.
   The LLM could output `speed: "fast"` next to an obstacle and only the safety filter
   would prevent collision.

### 2.5 Duplication Matrix

Cross-referencing functionality across all 4 scripts:

| Feature | VoiceMapper | YahboomExplorer | LLMRobotBrain | LLMNavigator |
|---------|:-----------:|:---------------:|:-------------:|:------------:|
| `/cmd_vel` publisher | **Yes** | **Yes** | **Yes** | No (not a node) |
| LiDAR obstacle detection | **Yes** (sectors + doorways) | **Yes** (sectors, simpler) | No | No |
| Audio I/O (mic/TTS) | **Yes** (pyaudio/ffmpeg) | **Yes** (near-identical) | No (topic-based) | No |
| Image → base64 | **Yes** | **Yes** (similar) | **Yes** (simple) | No |
| LLM think/execute | **Yes** (JSON parse) | **Yes** (JSON parse) | **Yes** (JSON parse) | **Yes** (tool calls) |
| Exploration loop | **Yes** (4 modes) | **Yes** (1 mode) | No | No |
| Odometry tracking | **Yes** | **Yes** (similar) | No | No |
| Object distance via vision | **Yes** | **Yes** (similar) | No | No |
| `observe()` vision | **Yes** | **Yes** (similar) | No | No |
| System prompt construction | **Yes** | **Yes** | **Yes** | **Yes** |
| JSON response parsing | **Yes** (fragile) | **Yes** (fragile) | **Yes** (fragile) | **Yes** (structured tool calls) |
| Safety filter | `_compute_safe_velocity()` | Binary obstacle check | **None** | Prompt-based only |
| Color following | No | **Yes** | No | No |
| Nav2 integration | **Yes** | No | No (action lib only) | **Yes** (tool definition) |
| SLAM/VSLAM management | **Yes** | No | No | No |
| Multi-provider LLM | No (OpenAI only) | No (OpenAI only) | **Yes** (4 providers) | No (OpenAI only) |
| Topic-based I/O | No (monolithic) | No (monolithic) | **Yes** | N/A (utility) |

### 2.6 Duplication Analysis: What's Actually Duplicated

#### Near-Identical Code (copy-paste with minor edits)

1. **Audio I/O pipeline** — `listen()`, `transcribe()`, `speak()`, `_find_microphone()`
   are nearly line-for-line identical between VoiceMapper and YahboomExplorer. Both use:
   - PyAudio with silence detection → WAV temp file → scipy resample → Whisper API
   - OpenAI TTS → MP3 temp file → ffmpeg → aplay
   - Same `speaking`/`listening` flags, same error handling

   **Estimated shared code:** ~200 lines

2. **Scan callback obstacle processing** — Both parse LaserScan into named sectors
   using the same index-range approach. VoiceMapper adds `front_wide` sector and
   doorway detection; YahboomExplorer is a strict subset.

   **Estimated shared code:** ~30 lines

3. **Odometry callback** — Both extract position and quaternion-to-yaw with the
   same `atan2(siny_cosp, cosy_cosp)` formula and distance accumulation.

   **Estimated shared code:** ~20 lines

4. **Image encoding** — Both have `image_to_base64()` with brightness enhancement
   for dark images, OpenCV + PIL fallback, resize to 640×480, JPEG quality 80.

   **Estimated shared code:** ~30 lines

5. **LLM JSON response parsing** — All three scripts that do LLM I/O use the same
   fragile pattern: `response_text.find('{')` / `rfind('}')` / `json.loads()`.

   **Estimated shared code:** ~15 lines per occurrence

#### Total Duplication Estimate

| Between | Duplicated Lines (approx) | % of Smaller Script |
|---------|--------------------------|---------------------|
| VoiceMapper ↔ YahboomExplorer | ~350 | ~31% of YahboomExplorer |
| VoiceMapper ↔ LLMRobotBrain | ~40 | ~6% of LLMRobotBrain |
| YahboomExplorer ↔ LLMRobotBrain | ~20 | ~3% of LLMRobotBrain |

### 2.7 Architectural Lineage

The scripts represent three evolutionary stages of the same project:

```
Stage 1: YahboomExplorer (DEPRECATED)
  └─ Simple voice-controlled explorer
  └─ HP60C camera, LiDAR, basic reactive exploration
  └─ Pattern: one monolithic class does everything

Stage 2: LLMRobotBrain (UNUSED PROTOTYPE)
  └─ Attempted to decouple LLM from direct sensor access
  └─ Multi-provider LLM support
  └─ Topic-based I/O architecture (never completed)
  └─ Pattern: ROS topics as inter-node interfaces

Stage 3: VoiceMapper (ACTIVE) + LLMNavigator (ACTIVE support)
  └─ VoiceMapper absorbed YahboomExplorer's functionality and expanded
  └─ Added SLAM, VSLAM, Nav2, 4 exploration modes, LLM-driven navigation
  └─ LLMNavigator extracted as a clean stateless decision module
  └─ Pattern: growing monolith + utility module
```

LLMRobotBrain's topic-based architecture was the **right direction architecturally**
(decoupled nodes communicating via ROS topics), but it was abandoned in favor of
growing VoiceMapper. The multi-provider LLM support and `RobotActionLibrary` from
LLMRobotBrain were never integrated.

### 2.8 Key Findings — Phase 2

1. **Only 2 of 4 scripts are active** — `yahboom_explorer.py` is explicitly deprecated
   (HP60C camera removed), `llm_robot_brain.py` was never deployed. Only `voice_mapper.py`
   and `llm_navigator.py` are in production.

2. **~350 lines of near-identical code** between VoiceMapper and YahboomExplorer — audio
   I/O, scan processing, odometry, image encoding, LLM interaction. This represents
   organic copy-paste evolution rather than shared libraries.

3. **LLMRobotBrain had the right architecture but was abandoned** — Its topic-based
   decoupled design (separate speech node, action executor, feedback loop) is closer
   to proper ROS2 practice than VoiceMapper's monolith. The multi-provider LLM support
   is a useful feature not present in VoiceMapper.

4. **LLMNavigator is the cleanest module** — Stateless, single-responsibility, structured
   output via tool calls. Its design should inform how the LLM decision layer is
   structured in a refactored architecture.

5. **Three different safety approaches, none complete:**
   - VoiceMapper: Proportional slowdown (best), but no rear safety
   - YahboomExplorer: Binary stop (simplest), but checks rear
   - LLMRobotBrain: No safety at all (dangerous)
   - LLMNavigator: Prompt-based only (unreliable)

6. **Three different LLM interaction patterns:**
   - VoiceMapper/YahboomExplorer: Free-form JSON in text (fragile parsing)
   - LLMRobotBrain: JSON with action dispatch via topics (partially decoupled)
   - LLMNavigator: OpenAI function calling with strict mode (most robust)

7. **No shared libraries** — Every script independently implements sensor processing,
   image encoding, audio I/O, and LLM interaction. There is no common `robot_utils`
   or `robot_interfaces` package.

8. **Feature loss from deprecation** — YahboomExplorer's `follow_color()` and
   depth query caching (`DepthQuery` dataclass) are not present in VoiceMapper.
   LLMRobotBrain's multi-provider LLM and declarative action library are also lost.

9. **The `rosmaster-a1-robot/scripts/` directory contains stale stubs** —
   `yahboom_explorer.py` (35 lines) and `llm_robot_brain.py` (75 lines) are early
   versions that bear no resemblance to the `scripts/` versions. These should be
   cleaned up to avoid confusion.

## Phase 3: Safety, Sensing & Decision Pipeline

**Scope:** Analyze the support modules: `safety_executor.py` (446 lines),
`sensor_snapshot.py` (552 lines), `exploration_memory.py` (344 lines). Trace how raw
sensor data is processed, packaged for LLM consumption, and how safety constraints are
enforced on the output.

**Key questions:**
- Is safety enforcement consistent or bypassable?
- How is sensor data aggregated and at what rate?
- What is the latency from sensor reading to motor command?
- Is the SafetyExecutor truly in the critical path?

### 3.1 Module Overview

All three modules were introduced by **Plan 006** (LLM-Driven Autonomous Exploration).
They are imported exclusively by `voice_mapper.py` and are only active during LLM
exploration mode (`explore_mode = "llm"`).

| Module | Lines | Classes | Role | Active During |
|--------|-------|---------|------|---------------|
| `safety_executor.py` | 446 | `SafetyExecutor`, `BlockedActionMemory`, `LLMWatchdog`, `NetworkMonitor` + 5 dataclasses | Validates VLM decisions against LiDAR, translates to velocity targets | LLM control loop |
| `sensor_snapshot.py` | 552 | `SensorSnapshotBuilder`, `EventTrigger` + `SensorSnapshot` dataclass | Captures immutable sensor snapshots for VLM consumption | VLM decision loop |
| `exploration_memory.py` | 344 | `ExplorationMemory` + `ExplorationState` dataclass | Tracks rooms, frontiers, dead ends, recent actions for VLM context | VLM decision loop + control loop |

### 3.2 Data Flow: Sense → Decide → Act Pipeline

The LLM exploration mode implements a three-stage pipeline that is architecturally
cleaner than the rest of VoiceMapper's control paths:

```
┌─── VLM Decision Thread (vlm_decision_loop) ──────────────────────────┐
│                                                                       │
│  EventTrigger.check_triggers() @ 10 Hz                               │
│       │ (fires on: doorway, intersection, dead_end, moved > 1.5m)    │
│       ▼                                                               │
│  SensorSnapshotBuilder.capture()                                      │
│       │ reads: vm.latest_image, vm.latest_depth, vm.latest_scan,     │
│       │        vm.latest_odom, vm.obstacle_distances, vm.detected_gaps│
│       ▼                                                               │
│  Frame Similarity Check (compute_frame_similarity)                    │
│       │ skip if similarity > 0.92 AND nearest > 2.0m AND no trigger  │
│       ▼                                                               │
│  LLMNavigator.decide(snapshot, memory)  ← GPT-4o-mini API call       │
│       │ returns: NavigationDecision (tool_name + parameters)          │
│       ▼                                                               │
│  vlm_decision_queue.put(decision)                                     │
│                                                                       │
└──────────────── queue ────────────────────────────────────────────────┘
                    │
                    ▼
┌─── LLM Control Thread (llm_control_loop) @ 20 Hz ───────────────────┐
│                                                                       │
│  vlm_decision_queue.get_nowait()                                      │
│       │                                                               │
│       ▼                                                               │
│  SafetyExecutor.execute_decision(decision, vm, memory)                │
│       │ 1. BlockedActionMemory.should_allow()                        │
│       │ 2. _pre_validate() — LiDAR clearance check                   │
│       │ 3. Translate tool → VelocityTarget/NavGoalTarget/...         │
│       ▼                                                               │
│  _compute_safe_velocity(raw_lin, raw_ang)   ← VoiceMapper method     │
│       │ Proportional slowdown from obstacle_distances                 │
│       ▼                                                               │
│  cmd_vel_pub.publish(twist)                                           │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘
```

### 3.3 `sensor_snapshot.py` — Deep Analysis

#### `SensorSnapshotBuilder` (lines 379–493)

**Pattern:** Observer that reads VoiceMapper's state directly via reference (`self.vm`).

The builder holds a direct reference to the VoiceMapper instance and reads its mutable
sensor fields (`latest_image`, `latest_depth`, `latest_scan`, `latest_odom`,
`obstacle_distances`, `detected_gaps`, `current_position`) without any synchronization.

**Snapshot contents assembled per `capture()` call:**

| Field | Source | Processing |
|-------|--------|------------|
| `annotated_image_b64` | `vm.latest_image` + `vm.latest_depth` | Resize to 640×480, overlay 7 depth samples + heading arrow, JPEG encode, base64 |
| `lidar_summary` | `vm.latest_scan` (raw LaserScan) | 12 sectors × 30°, 10th percentile per sector, qualitative labels (WALL/OBSTACLE/NEAR/CLEAR) |
| `doorway_gaps` | `vm.detected_gaps` | Direct copy with rounding |
| `robot_state` | `vm.current_position`, `vm.latest_odom` | x, y, heading (quaternion→deg), speed, distance from start, explore time |
| `affordance_scores` | `vm.obstacle_distances` | 6 directions, linear interpolation 0.1–1.0 from LiDAR distance |
| `previous_action_result` | Set by control loop via `set_previous_action_result()` | Last SafetyExecutor result |

**Key design strengths:**
1. **Immutable snapshots** — `SensorSnapshot` is a `@dataclass(frozen=True)`, preventing
   mutation after capture. This is the only place in the codebase that uses immutable data.
2. **Token-efficient** — The 12-sector LiDAR summary (~120 tokens) is much cheaper than
   raw LaserScan data (720+ floats). Affordance scores add only ~30 tokens.
3. **Depth annotation** — Overlaying depth values on the camera image gives the VLM spatial
   context without needing a separate depth channel in the API call.

**Key weaknesses:**
1. **No thread safety** — `capture()` reads `vm.latest_image`, `vm.latest_depth`, and
   `vm.latest_scan` in sequence. Between reads, the main thread's `spin_once()` could
   deliver new callbacks, producing a snapshot where the image is from time T but the
   LiDAR is from T+50ms. There is no lock or atomic snapshot mechanism.
2. **Coupled to VoiceMapper internals** — Direct attribute access (e.g. `vm.latest_odom`,
   `vm.obstacle_distances`) means the builder breaks if VoiceMapper renames or restructures
   any field. No interface contract.
3. **Redundant LiDAR processing** — `build_lidar_summary()` reprocesses the raw LaserScan
   into 12 sectors, duplicating logic from `scan_callback()` which already computes
   `obstacle_distances` and `detected_gaps` from the same data. Two independent sector
   decompositions exist: 6 named sectors in `scan_callback()` and 12 angular sectors in
   `build_lidar_summary()`.

#### `EventTrigger` (lines 272–372)

Monitors four conditions to fire early VLM calls:

| Trigger | Condition | Cooldown |
|---------|-----------|----------|
| Doorway | Any gap ≥ 0.7m width in `detected_gaps` | 2s global |
| T-intersection | ≥ 3 adjacent open directions (≥ 2.0m) | 2s global |
| Dead end | All forward sectors < 1.0m | 2s global |
| Position | Moved > 1.5m since last VLM call | 2s global |

The trigger polling happens at 10 Hz in the VLM decision loop. When a trigger fires, the
VLM call happens immediately rather than waiting for the 5s base interval. This is a
good adaptive frequency mechanism, but the triggers are polled from the same thread that
makes the VLM API call, meaning trigger response is delayed by the API call latency
(500–2000ms+).

#### `annotate_image()` (lines 108–159) and `build_lidar_summary()` (lines 162–212)

These are standalone functions that do the heavy lifting of preparing sensor data for
VLM consumption. `annotate_image()` runs OpenCV operations (resize, text overlay, arrow
drawing, JPEG encode) which could take 5–15ms per frame — acceptable at the VLM's
0.2–5 Hz call rate but would be problematic at 20 Hz.

#### `compute_frame_similarity()` (lines 231–257)

MAD-based similarity on 160×120 grayscale frames. Used in two places:
1. **Skip VLM call entirely** if similarity > 0.92 AND nearest obstacle > 2.0m (saves API cost)
2. **Text-only VLM call** if similarity > 0.95 AND nearest obstacle > 3.0m (sends LiDAR only, no image)

This is a cost-optimization mechanism, not a safety mechanism. The safety backstop
ensures a VLM call happens at least every 10 seconds regardless.

### 3.4 `safety_executor.py` — Deep Analysis

#### Architecture: Translation + Validation Layer

SafetyExecutor is designed as a **translation layer** between the LLM's abstract decisions
and the control loop's concrete velocity commands. It does NOT publish to `/cmd_vel`
directly. This is a critical architectural boundary:

```
LLMNavigator.decide() → NavigationDecision → SafetyExecutor → ExecutionResult → Control Loop → cmd_vel
                         (abstract)            (validates)       (concrete target)  (publishes)
```

#### Safety Enforcement: Two-Layer Model

**Layer 1: SafetyExecutor pre-validation** (`_pre_validate()`, lines 241–290)

- Checks LiDAR clearance in the commanded direction
- Maps direction → LiDAR sectors (e.g., "forward_left" → `["front", "front_left"]`)
- **Reject** if minimum clearance < 0.5m (`REJECT_DISTANCE`)
- **Downgrade speed to "slow"** if clearance < 1.0m (`DOWNGRADE_DISTANCE`)
- Records blocked actions in `BlockedActionMemory` to prevent retry loops

**Layer 2: VoiceMapper's `_compute_safe_velocity()`** (called in control loop)

- Proportional velocity scaling based on real-time obstacle distances
- Applied at 20 Hz on every cmd_vel publish
- Can override SafetyExecutor's approved velocity with emergency stop

**Relationship between layers:**

| Aspect | Layer 1 (SafetyExecutor) | Layer 2 (_compute_safe_velocity) |
|--------|--------------------------|----------------------------------|
| When | Once per VLM decision (~0.2–5 Hz) | Every control loop iteration (20 Hz) |
| Data | Snapshot of obstacle_distances at validation time | Live obstacle_distances at publish time |
| Action | Reject/downgrade/approve | Scale velocity / emergency stop |
| Scope | Only LLM exploration mode | LLM exploration + move() calls |
| Bypassable? | No (hardcoded in execute_decision) | No (hardcoded in llm_control_loop) |

**The two-layer model is sound in principle** — Layer 1 prevents obviously bad decisions
from reaching the control loop, while Layer 2 handles dynamic obstacles that appear
after the decision was validated. However, Layer 1 uses a point-in-time snapshot of
`obstacle_distances` that could be stale by the time the velocity target is actually
applied (decision validation → queue → dequeue → apply could span 100–500ms).

#### `BlockedActionMemory` (lines 96–144)

Prevents the VLM from retrying blocked directions more than 2 times within 15 seconds.
Uses direction-hashing: `"move_toward:forward"`, `"rotate:90"`, etc.

This solves a real problem: without it, the VLM might repeatedly command "move forward"
toward a wall, getting rejected each time but consuming API calls and time.

**Weakness:** The hash is coarse — blocking "move_toward:forward" blocks ALL forward
moves regardless of speed or duration. A slow, short forward move near an obstacle might
be safe but gets blocked because a previous fast forward was rejected.

#### `LLMWatchdog` (lines 149–175)

Implements degradation tiers based on time since last VLM response:

| Tier | Elapsed | Action |
|------|---------|--------|
| CONTINUE | 0–3s | Normal operation |
| STOP_WAIT | 3–10s | Stop robot, wait for VLM |
| LOCAL_NAV | 10–30s | Fall back to frontier exploration |
| RETURN_HOME | >30s | Navigate to (0,0), then frontier explore |

The watchdog is checked at 20 Hz in `llm_control_loop()`. The LOCAL_NAV and RETURN_HOME
tiers cause the control loop to **exit entirely** (via `return`), handing control to
`exploration_loop()`. This is a mode transition without formal state machine management.

#### `NetworkMonitor` (lines 180–221)

Tracks rolling API success rate and latency over a 20-call / 60-second window.
Reports 4 tiers: normal, elevated (avg > 3s), degraded (no success 10s), offline (30s).

Currently, the network monitor is **read but never acted upon** in the control loop.
The `current_tier` is logged in the 60-second metrics summary but does not affect behavior.
Only the watchdog (which uses wall-clock time, not the network monitor's tier) actually
degrades operation.

#### Target Dataclasses (lines 27–59)

| Target Type | Purpose | Fields |
|-------------|---------|--------|
| `VelocityTarget` | Direct velocity command | `twist: Twist`, `duration_s: float` |
| `NavGoalTarget` | Nav2 delegation | `x, y: float`, `reason: str` |
| `ObserveTarget` | Stop + observe | `focus: str` |
| `StopTarget` | Intentional stop | `reason: str` |

These form a **discriminated union** that the control loop pattern-matches on.
This is clean: the SafetyExecutor returns a sealed set of possible actions, and the
control loop handles each case explicitly.

#### Tool-to-Target Translation

| LLM Tool | SafetyExecutor Method | Target | Velocity |
|----------|----------------------|--------|----------|
| `move_toward` | `_execute_move()` | `VelocityTarget` | 0.08/0.12/0.18 × direction_factor, Ackerman constraint |
| `rotate` | `_execute_rotate()` | `VelocityTarget` | 0.05 linear + ±0.4 angular, duration from degrees |
| `navigate_to_goal` | `_execute_navigate()` | `NavGoalTarget` or `StopTarget` | Delegates to Nav2 |
| `stop_robot` | `_execute_stop()` | `StopTarget` | Zero velocity |
| `observe_scene` | `_execute_observe()` | `ObserveTarget` | Zero velocity + spawn observe thread |
| `check_path_clear` | `_execute_check_path()` | `StopTarget` (with check_result) | No movement, LiDAR query only |
| `report_discovery` | `_execute_report()` | `StopTarget` | No movement, TTS + memory update |

**Velocity constants:**

| Speed | m/s | Notes |
|-------|-----|-------|
| slow | 0.08 | ~0.3 km/h |
| medium | 0.12 | ~0.4 km/h — default |
| fast | 0.18 | ~0.6 km/h |
| Ackerman minimum | 0.05 | Forced when angular > 0.1 |
| Arc turn angular | 0.4 | rad/s for rotate commands |

Duration is clamped: moves 1–8s, rotations 0.5–6s.

### 3.5 `exploration_memory.py` — Deep Analysis

#### `ExplorationMemory` (lines 48–291)

**Pattern:** Session-scoped state store with prompt serialization and disk persistence.

The memory tracks:
1. **Rooms** — Named locations with exploration percentage, discovered objects, and
   entry linkage (which room was entered from). Initialized with "start" room.
2. **Frontiers** — Open exploration boundaries with direction, distance, type, width.
   Fully replaced on each update (no incremental).
3. **Dead ends** — Blocked directions with position-based deduplication (merge within 0.5m
   and 30° of existing dead end).
4. **Recent actions** — Last 5 actions with compact summaries (tool name + key params +
   result). This gives the VLM short-term memory of what it just did.
5. **Statistics** — Total discoveries, exploration duration.

#### Prompt Serialization (`to_prompt_text()`, lines 191–238)

The memory is serialized to ~200 tokens of structured text for inclusion in each VLM
prompt. Format:

```
EXPLORATION MEMORY:
  Rooms: start (0% explored), kitchen (30% explored)
  Current room: kitchen
  Frontiers: 2 open — #1 at 45° (doorway, 2.5m, w=0.9m); #2 at 270° (opening, 3.1m, w=1.2m)
  Dead ends: 180° (blocked)
  Recent:
    [5] move_toward(forward, medium) -> success, +1.23m
    [4] rotate(-90°) -> success
  Discoveries: 3 | Duration: 2.5min
```

This is an efficient design — the VLM gets spatial context, recent history, and available
options in a compact format that fits within a small fraction of the context window.

#### Persistence (`save_to_disk()` / `load_from_disk()`, lines 244–280)

YAML serialization to disk, allowing exploration state to survive between sessions. Uses
`yaml.safe_load()` for safe deserialization.

#### Integration Points

| Called From | Method | Purpose |
|-------------|--------|---------|
| `llm_control_loop()` (line 2652) | `update_after_action()` | Record action outcome after SafetyExecutor validates |
| `vlm_decision_loop()` (line 2549) | `update_after_action()` | Record action when VLM decides (before execution) |
| `safety_executor._execute_report()` (line 384) | `record_discovery()` | Log VLM-reported discovery |
| `vlm_decision_loop()` (line 2524) | passed to `llm_navigator.decide()` | Serialized to prompt text for VLM context |
| `start_llm_exploration()` (line 2770) | Constructor | Fresh memory for each exploration session |

**Issue: Double-counting actions.** Both `vlm_decision_loop()` (line 2549) and
`llm_control_loop()` (line 2652) call `update_after_action()` for the same decision.
The VLM loop records the decision immediately with `{'success': True}`, while the control
loop records it later with the actual safety executor result. This means every action
appears twice in `recent_actions` with potentially conflicting results. Since
`MAX_RECENT_ACTIONS = 5`, the window of visible history is effectively halved.

### 3.6 End-to-End Latency Analysis

Tracing latency from sensor reading to motor command through the full pipeline:

```
T+0ms     : LiDAR scan_callback delivers latest_scan + obstacle_distances
T+0ms     : Camera callback delivers latest_image/latest_depth
            (callbacks at ~10–30 Hz, processed in main thread via spin_once)

--- VLM Decision Thread ---
T+0ms     : EventTrigger.check_triggers() reads obstacle_distances (10 Hz poll)
T+0ms     : SensorSnapshotBuilder.capture() reads all sensor fields (~5–15ms for image annotation)
T+15ms    : LLMNavigator.decide() sends API call to GPT-4o-mini
T+500-2000ms : API response received, NavigationDecision enqueued

--- LLM Control Thread ---
T+500-2050ms : queue.get_nowait() picks up decision (up to 50ms delay for 20 Hz loop)
T+500-2050ms : SafetyExecutor._pre_validate() checks obstacle_distances
T+500-2050ms : SafetyExecutor translates to VelocityTarget
T+500-2050ms : _compute_safe_velocity() scales velocity
T+500-2050ms : cmd_vel_pub.publish()

Total end-to-end: 500–2050ms (dominated by API latency)
```

**For the safety path only** (no VLM, just control loop):
```
T+0ms     : scan_callback updates obstacle_distances
T+0-50ms  : llm_control_loop iteration reads obstacle_distances
T+0-50ms  : _compute_safe_velocity() applies safety scaling
T+0-50ms  : cmd_vel_pub.publish()

Total safety path: 0–50ms (bounded by 20 Hz loop rate)
```

The safety path latency is acceptable for obstacle avoidance at the robot's low speeds
(0.08–0.18 m/s). At max speed (0.18 m/s), the robot moves 9mm per 50ms interval.

### 3.7 Safety Enforcement Consistency Audit

**Question: Is safety enforcement consistent or bypassable?**

| Control Path | Layer 1 (SafetyExecutor) | Layer 2 (_compute_safe_velocity) | Assessment |
|-------------|--------------------------|----------------------------------|------------|
| LLM exploration (`llm_control_loop`) | **Yes** — every decision validated | **Yes** — every publish scaled | **Two-layer safety, strongest path** |
| `move()` (from voice/execute) | **No** — SafetyExecutor not in path | **Yes** — every publish scaled | **Single-layer safety** |
| `_execute_uturn()` | **No** | **No** — uses own obstacle check | **Weakest path** |
| `_reactive_exploration_step()` | **No** | **No** — direct publish | **No safety filter** |
| `_reactive_exploration_loop()` | **No** | **Partial** — delegates to `move()` for forward | **Inconsistent** |
| `_random_exploration_walk()` | **No** | **Partial** — delegates to `move()` for drive-through | **Inconsistent** |
| `exploration_loop()` via Nav2 | **No** | **No** — Nav2 handles its own safety | **Nav2's responsibility** |
| `emergency_stop()` | **No** | **No** — IS the safety mechanism | **Override/last resort** |

**Key finding: SafetyExecutor only protects the LLM exploration path.** The 9 other
`/cmd_vel` paths identified in Phase 1 do not use SafetyExecutor at all. This is by
design (SafetyExecutor was built for Plan 006), but it means the two-layer safety model
is not a system-wide guarantee.

**Paths with NO safety filter at all:**
- `_reactive_exploration_step()` — publishes directly based on obstacle_distances but with
  no proportional scaling or emergency stop check
- `_execute_uturn()` — checks obstacle distance per iteration but publishes at fixed
  velocity, no proportional scaling

### 3.8 Thread Safety Analysis

| Shared State | Written By | Read By | Synchronized? |
|-------------|------------|---------|---------------|
| `vm.obstacle_distances` | `scan_callback` (main thread) | VLM decision loop, control loop, EventTrigger | **No** — dict mutation is not atomic |
| `vm.detected_gaps` | `scan_callback` (main thread) | VLM decision loop, EventTrigger | **No** — list replacement |
| `vm.latest_image` | `camera_callback` (main thread) | SensorSnapshotBuilder | **No** — reference swap |
| `vm.latest_depth` | `depth_callback` (main thread) | SensorSnapshotBuilder | **No** — reference swap |
| `vm.latest_scan` | `scan_callback` (main thread) | SensorSnapshotBuilder | **No** — reference swap |
| `vm.latest_odom` | `odom_callback` (main thread) | SensorSnapshotBuilder | **No** — reference swap |
| `vm.current_position` | `odom_callback` (main thread) | SensorSnapshotBuilder, ExplorationMemory, EventTrigger | **No** — dict mutation |
| `vlm_decision_queue` | VLM decision loop | Control loop | **Yes** — `queue.Queue` is thread-safe |
| `ExplorationMemory.state` | VLM decision loop + control loop | VLM decision loop (serialization) | **No** — both threads mutate |

The `vlm_decision_queue` is the **only properly synchronized** data transfer in the
pipeline. All other shared state relies on Python's GIL for partial protection, but
dict mutations (e.g., updating `obstacle_distances` while another thread iterates it)
can produce inconsistent reads.

The `ExplorationMemory.state` is particularly concerning — both the VLM decision loop
(line 2549) and the control loop (line 2652) call `update_after_action()`, which appends
to and trims `recent_actions`. Concurrent list operations under the GIL are unlikely to
corrupt but can produce unexpected ordering.

### 3.9 Key Findings — Phase 3

1. **The Plan 006 modules implement a proper sense→decide→act pipeline** — This is the
   cleanest architecture in the codebase: immutable snapshots, a validation/translation
   layer, discriminated union targets, and a single cmd_vel publisher. It exists only for
   LLM exploration mode.

2. **Two-layer safety is sound but limited to one control path** — SafetyExecutor
   (pre-validation) + `_compute_safe_velocity()` (real-time scaling) together provide
   robust protection, but only for `llm_control_loop()`. The other 9 cmd_vel paths
   identified in Phase 1 have weaker or no safety filters.

3. **No thread synchronization on sensor state** — SensorSnapshotBuilder,
   SafetyExecutor, and EventTrigger all read VoiceMapper's mutable sensor fields without
   locks. Python's GIL prevents memory corruption but not logical inconsistencies (e.g.,
   snapshot where image and LiDAR are from different time points).

4. **ExplorationMemory is double-written** — Both the VLM decision loop and the control
   loop call `update_after_action()` for the same decision, with different result data.
   This halves the effective recent action window and can produce misleading VLM context.

5. **NetworkMonitor is read but never acted on** — The monitor tracks API success rate
   and latency tiers but no code path actually degrades behavior based on it. Only the
   watchdog (wall-clock based) triggers fallback. The monitor is vestigial.

6. **Redundant LiDAR processing** — Raw LaserScan data is processed into sectors twice:
   once in `scan_callback()` (6 named sectors + doorway detection) and again in
   `build_lidar_summary()` (12 angular sectors). These produce different sector
   definitions from the same data.

7. **BlockedActionMemory hash is too coarse** — Blocking `"move_toward:forward"` blocks
   all forward moves regardless of speed/duration. A slow cautious approach could be
   rejected because a fast approach was previously blocked.

8. **End-to-end latency is API-dominated** — The sensor→decision→motor path takes
   500–2050ms total, of which 500–2000ms is the GPT-4o-mini API call. The local safety
   path (obstacle_distances → _compute_safe_velocity → cmd_vel) runs at 0–50ms, which
   is adequate for the robot's low speeds.

9. **EventTrigger is a good adaptive frequency mechanism** — Firing VLM calls on
   doorways, intersections, dead ends, and movement thresholds is smarter than a fixed
   timer. However, trigger polling and VLM calling share a thread, so trigger response
   is delayed by the previous API call.

10. **The module design is a blueprint for system-wide refactoring** — The patterns used
    (immutable snapshots, validation layers, discriminated union targets, queue-based
    decoupling) could be applied to all control paths, not just LLM exploration. The
    current limitation is that these modules are tightly coupled to VoiceMapper internals
    via direct attribute access rather than ROS interfaces.

## Phase 4: Systemd Services, Launch Files & Runtime Topology

**Scope:** Examine all `.service` files under `scripts/`, all `.launch.py` files, and
the `robot.target` orchestration. Map which nodes run in which processes, what the
actual runtime topic graph looks like.

**Key questions:**
- Are ROS nodes properly isolated in separate processes?
- What is the startup/shutdown ordering?
- Are there single points of failure?
- How does the systemd layer map to the ROS node graph?

### 4.1 Evolution of Startup Approaches

The codebase contains **three distinct startup mechanisms**, reflecting the system's
organic evolution:

| Generation | Mechanism | Entry Point | Status |
|------------|-----------|-------------|--------|
| Gen 1 | Monolithic service | `voice_mapper.service` → `start_robot.sh` | **Deprecated** — prints warning, kept for backward compat |
| Gen 2 | Interactive script | `rosmaster_control.sh` → menu/CLI | **Manual use only** — for development/debugging |
| Gen 3 | Multi-service systemd | `robot.target` → 6 individual `.service` files | **Active** — production deployment |

**Key insight:** Gen 1 (`voice_mapper.service`) targets `multi-user.target` and bundles
everything in one process. Gen 3 (`robot.target`) also targets `multi-user.target`
but decomposes into isolated services. The `rosmaster_control.sh service install`
function explicitly removes Gen 1 when installing Gen 3 (line 341–346).

### 4.2 robot.target — The Orchestration Unit

**File:** `scripts/robot.target` (8 lines)

```
[Unit]
Description=Robot Complete System
Wants=robot-base robot-lidar robot-camera robot-tf robot-vslam robot-voice-mapper
After=network.target sound.target

[Install]
WantedBy=multi-user.target
```

**Analysis:**

- Uses `Wants=` (not `Requires=`) for all 6 services — **soft dependencies**. If any
  service fails to start, the target still activates. This is intentional: VSLAM is
  optional, and the robot should operate in degraded mode.
- `After=network.target sound.target` — waits for network and audio subsystems, but
  does NOT wait for individual robot services. There is **no ordering guarantee**
  between the 6 services at the target level.
- Ordering between services is defined by `After=`/`Requires=` within each service file.

### 4.3 Service Dependency Graph

```
                 robot.target
                 (Wants all 6)
                      │
    ┌─────┬─────┬─────┼──────┬───────────────────┐
    ▼     ▼     ▼     ▼      ▼                   ▼
  base  lidar camera   tf   vslam          voice-mapper
   │      │     │      │     │                    │
   │      │     │      │     │  After=base,lidar,camera,tf,vslam
   │      │     │      │     │  Requires=base,lidar,tf
   │      │     │      │     │  Wants=camera,vslam
   │      │     │      │     │                    │
   │      │     │      │     └─ After=camera      │
   │      │     │      │        Wants=camera      │
   │      │     │      │        ConditionPath=    │
   │      │     │      │        oakd_vslam.launch.py
   │      │     │      │                          │
   └──────┴─────┴──────┴──────────────────────────┘
         (no ordering between base/lidar/camera/tf)
```

**Ordering semantics:**

| Service | After | Requires (hard) | Wants (soft) | Restart | Rate Limit |
|---------|-------|-----------------|--------------|---------|------------|
| robot-base | — | — | — | always / 5s | 5 in 60s |
| robot-lidar | — | — | — | always / 5s | 5 in 60s |
| robot-camera | — | — | — | always / 10s | 5 in 120s |
| robot-tf | — | — | — | always / 3s | 5 in 60s |
| robot-vslam | camera | camera | — | on-failure / 10s | 3 in 60s |
| robot-voice-mapper | base,lidar,camera,tf,vslam | base,lidar,tf | camera,vslam | on-failure / 10s | — |

**Findings:**

1. **Hardware services (base, lidar, camera, tf) have no ordering between them** — they
   start in parallel. This is correct: they have no dependencies on each other.

2. **VSLAM correctly depends on camera** — uses `After=robot-camera` and
   `Wants=robot-camera`. Also has `ConditionPathExists=` guard so it skips cleanly
   if the launch file isn't deployed.

3. **voice-mapper waits for everything** — `After=` all 5 services, `Requires=` the
   essential three (base, lidar, tf), `Wants=` the optional two (camera, vslam).

4. **No readiness checks at the systemd level** — services are `Type=simple`, so
   systemd considers them "started" as soon as the process forks. There's no
   notification when ROS nodes are actually publishing topics. The old `start_robot.sh`
   had explicit topic readiness polling (lines 100–128), but this is lost in Gen 3.

5. **Camera has a wider restart window** — `StartLimitIntervalSec=120` and
   `RestartSec=10`, vs 60s/5s for others. This is appropriate because OAK-D Pro USB
   resets take longer.

### 4.4 Service-to-Process-to-Node Mapping

Each systemd service runs a `run_*.sh` wrapper that sources `ros2_env.sh` (shared
ROS2 workspace sourcing) and then `exec`s into a ROS2 launch or node.

#### Process Architecture at Runtime

| Systemd Service | Bash Wrapper | ROS2 Command | Resulting Nodes | Process Model |
|-----------------|-------------|--------------|-----------------|---------------|
| robot-base | `run_base.sh` | `ros2 launch yahboomcar_bringup yahboomcar_bringup_A1_launch.py` | `/driver_node`, `/base_node`, `/ekf_filter_node`, `/robot_state_publisher` | Multi-node launch (4+ nodes in child processes) |
| robot-lidar | `run_lidar.sh` | `ros2 launch sllidar_ros2 sllidar_c1_launch.py` | `/sllidar_node` | Single-node launch |
| robot-camera | `run_camera.sh` | `ros2 launch depthai_ros_driver camera.launch.py` | `/oak` (multi-pipeline) | Single-node launch (params from `oakd_params.yaml`) |
| robot-tf | `run_tf.sh` | 2x `static_transform_publisher` (background) | 2 anonymous TF nodes | 2 child processes, parent monitors both |
| robot-vslam | `run_vslam.sh` | `ros2 launch oakd_vslam.launch.py` | `/visual_slam_node` (ComposableNode in `component_container`) | Composable node container |
| robot-voice-mapper | `run_voice_mapper.sh` | `python3 voice_mapper.py` | `/voice_mapper` (monolith) + **dynamically launched subprocesses** | Single Python node + subprocess children |

**Total steady-state nodes: ~10-12 ROS2 nodes across 6 systemd services.**

#### voice-mapper's Dynamic Subprocess Children

The monolith (`voice_mapper.py`) launches additional ROS2 stacks **as subprocesses at
runtime**, creating nodes that are invisible to systemd:

| Subprocess | Trigger | ROS2 Command | Nodes Created | Lifecycle |
|------------|---------|-------------|---------------|-----------|
| Nav2 | `start_nav2()` (line 542) | `ros2 launch nav2_bringup navigation_launch.py` | `bt_navigator`, `controller_server`, `planner_server`, `behavior_server`, `smoother_server`, `velocity_smoother`, `waypoint_follower`, `lifecycle_manager` (~8 nodes) | Managed by voice_mapper PID tracking; killed on `stop_nav2()` |
| SLAM | `start_slam()` (line 1406) | `ros2 launch slam_toolbox online_sync_launch.py` | `slam_toolbox` | Managed by voice_mapper PID tracking |
| Isaac VSLAM | `start_isaac_vslam()` (line 1548) | `ros2 launch oakd_vslam.launch.py` | `visual_slam_node` | Can also be externally launched by systemd — voice_mapper detects this (line 1517–1521) |

**This means the actual runtime topology varies between ~10 and ~20+ nodes** depending
on which features the user activates through voice commands.

### 4.5 Process Isolation Analysis

#### What IS properly isolated

1. **Hardware drivers are in separate systemd services** — a camera crash doesn't take
   down the LiDAR or base driver. Each restarts independently.

2. **VSLAM is optional and guarded** — `ConditionPathExists` prevents startup if the
   launch file isn't deployed. Failure restarts only 3 times before giving up.

3. **The TF service monitors both publishers** — `run_tf.sh` uses `wait -n` to detect
   if either TF publisher dies, then kills both and exits non-zero, triggering systemd
   restart. This is a proper watchdog pattern.

4. **Environment sourcing is centralized** — `ros2_env.sh` is shared across all
   wrappers, preventing workspace sourcing drift.

#### What is NOT properly isolated

1. **Nav2, SLAM, and VSLAM are children of voice_mapper** — When voice_mapper restarts
   (or crashes), these subprocess trees are orphaned or killed. There is no systemd
   service for Nav2 or SLAM. If voice_mapper crashes:
   - `subprocess.Popen` children may become zombies (stdout/stderr pipes break)
   - No cleanup handler — `VoiceMapper.__init__` doesn't register signal handlers
   - `_kill_stale_nav2()` (line 492) exists to clean up **previous** voice_mapper
     instances' orphaned Nav2 processes, confirming this is a known problem

2. **Dual VSLAM launch paths create conflicts** — VSLAM can be started by:
   - `robot-vslam.service` (systemd, always-on)
   - `start_isaac_vslam()` in voice_mapper (on-demand subprocess)

   Voice_mapper checks `self.vslam_tracking` (line 1518) to detect externally-launched
   VSLAM and "adopts" it without spawning a duplicate. However, if voice_mapper starts
   VSLAM as a subprocess AND the systemd service is also running, there could be two
   VSLAM instances competing for GPU resources.

3. **voice_mapper is a single point of failure for the application** — All decision
   logic, LLM calls, speech I/O, exploration control, and motor command arbitration
   live in one process. If the Python interpreter crashes (OOM, segfault in numpy/cv2),
   everything above the hardware layer stops.

4. **No health monitoring of subprocess nodes** — After launching Nav2, voice_mapper
   checks `bt_navigator` lifecycle state during startup (line 557) but never again.
   If the controller_server crashes mid-navigation, voice_mapper won't detect it until
   a goal fails.

### 4.6 Startup Timing & Readiness

#### Systemd (Gen 3) — No Readiness Checks

All services are `Type=simple`. Systemd considers them started as soon as the shell
script's process is created. The `After=` directives ensure ordering of **process
creation**, not **topic availability**.

**Practical consequence:** `robot-voice-mapper` may start before camera or VSLAM topics
are actually publishing. voice_mapper.py handles this by checking `self.latest_image`,
`self.latest_scan`, etc. before using them, but there's no structured "wait for
readiness" phase.

#### start_robot.sh (Gen 1) — Had Readiness Checks

The deprecated `start_robot.sh` polled for topic availability (lines 100–128):
```bash
# Wait up to 30s for /scan, /odom, /oak/rgb/image_raw
while [ $ELAPSED -lt $TIMEOUT ]; do
    TOPICS=$(ros2 topic list)
    # check for required topics...
done
```

This readiness gate was **lost** in the Gen 3 migration. voice_mapper compensates with
its `_sensor_monitor_loop()` (line 3215) which runs in a daemon thread and checks
sensor staleness, but this is a **detection** mechanism, not a **wait-for-ready** gate.

#### Subprocess Launches — Sleep-Based Timing

voice_mapper's subprocess launches use sleep-based waiting:
- Nav2: polls `wait_for_server()` + lifecycle check for up to 60 seconds (line 553)
- SLAM: `time.sleep(3)` then assumes ready (line 1414)
- VSLAM: polls `self.vslam_tracking` for up to 30 seconds (line 1558)

Nav2's approach is the most robust (lifecycle-state-aware). SLAM's is the weakest
(fixed 3-second sleep with no verification).

### 4.7 The rosmaster-a1-robot Package — Unused ROS2 Package

The `rosmaster-a1-robot/` directory contains a formal ROS2 package structure:

```
rosmaster-a1-robot/
├── package.xml          (ament_cmake build, declares all deps)
├── setup.py             (entry_points: voice_mapper, yahboom_explorer, etc.)
├── launch/              (4 launch files: robot_bringup, slam, navigation, camera)
├── config/              (4 YAML configs: slam, vslam, camera, lidar)
├── scripts/             (copies of voice_mapper.py, yahboom_explorer.py, etc.)
├── msg/RobotStatus.msg  (custom message)
├── srv/VoiceCommand.srv (custom service)
└── test/                (3 test files)
```

**This package is NOT used at runtime.** The actual deployment uses `scripts/` directly.

**Evidence:**
- Launch files use placeholder paths (`/path/to/config/...`) — never parameterized
- `package.xml` declares `ament_cmake` build but `setup.py` uses `setuptools` — mixed
  build system conflict
- `setup.py` entry_points reference `scripts.voice_mapper:main` but the module path
  would require `rosmaster_a1_robot/scripts/` to be a Python package (no `__init__.py`)
- Custom `.msg` and `.srv` files are defined but never imported in any Python code
- The `scripts/` subdirectory contains older copies of the Python files

**Significance:** This package represents an abandoned attempt at proper ROS2 packaging.
The launch files in `rosmaster-a1-robot/launch/` are architecturally interesting as
they show a vision of running all nodes through a single `ros2 launch` command, but
they were never completed (placeholder paths, wrong executable names).

### 4.8 Configuration Management

#### Parameter Files

| File | Used By | Purpose |
|------|---------|---------|
| `scripts/oakd_params.yaml` | `run_camera.sh` | OAK-D Pro pipeline config (RGBD, stereo, USB speed, IMU) |
| `scripts/oakd_vslam.launch.py` | `run_vslam.sh` + voice_mapper | VSLAM params baked into launch file (not externalized) |
| `scripts/nav2_params.yaml` | voice_mapper `start_nav2()` | Full Nav2 config (AMCL, planners, costmaps, collision monitor) |
| `scripts/ros2_env.sh` | All `run_*.sh` wrappers | ROS2 workspace sourcing, `ROS_DOMAIN_ID=62` |
| `/home/jetson/.rosmaster_llm_config` | voice_mapper (source) | LLM API keys (shell `export` format) |
| `/home/jetson/.rosmaster_llm_env` | voice_mapper service (EnvironmentFile) | Same keys in `KEY=VALUE` format (created by `service install`) |

**Issues:**
- VSLAM parameters are embedded in the launch file (line 23–46 of `oakd_vslam.launch.py`)
  rather than in an external YAML. Changing IMU noise params requires editing Python.
- Two copies of LLM config in different formats (`.rosmaster_llm_config` with `export`
  prefix, `.rosmaster_llm_env` without) — the install script creates one from the other
  (`sed 's/export //'`) but they can drift if manually edited.
- Nav2 params are only used by voice_mapper's `start_nav2()` subprocess — not by any
  systemd service. If Nav2 were its own service, this file would need to be referenced
  in a `run_nav2.sh` wrapper.

### 4.9 KillMode & Shutdown Behavior

| Service | KillMode | TimeoutStopSec | Implication |
|---------|----------|----------------|-------------|
| robot-base | control-group | 10s | Kills all processes in the cgroup (base_node, ekf, driver, robot_state_publisher). Clean. |
| robot-lidar | control-group | 10s | Kills lidar node. Clean. |
| robot-camera | control-group | 10s | Kills camera node. Camera USB may need time to release. |
| robot-tf | control-group | 10s | Kills both TF publishers. Clean. |
| robot-vslam | control-group | 10s | Kills VSLAM container. GPU memory released. |
| robot-voice-mapper | control-group | 15s | **Problem:** kills voice_mapper AND any Nav2/SLAM subprocesses in the cgroup. This is correct for cleanup, but Nav2's lifecycle manager may not shut down cleanly in 15s. |
| voice_mapper (Gen 1) | mixed | 10s | Sends SIGTERM to main, then SIGKILL to cgroup — worse than control-group for child cleanup. |

**The Gen 3 `control-group` KillMode for voice-mapper is an improvement** over Gen 1's
`mixed` mode: it terminates all children simultaneously rather than hoping the main
process will clean up its subprocesses within the timeout.

### 4.10 Key Findings

1. **Gen 3 multi-service architecture is a significant improvement** — Hardware drivers
   are properly isolated in separate systemd services with independent restart policies.
   This is the right approach.

2. **Nav2 and SLAM should be systemd services, not voice_mapper subprocesses** — The
   biggest remaining isolation gap. Nav2 is a complex 8-node stack that should have its
   own lifecycle, independent of the application layer. SLAM similarly.

3. **Readiness gating was lost in the Gen 1→Gen 3 migration** — The old `start_robot.sh`
   polled for topic availability. Gen 3 services are fire-and-forget (`Type=simple`).
   Using `Type=notify` with `sd_notify` (via a wrapper) or `ExecStartPost=` with a
   readiness check script would restore this.

4. **Dual VSLAM launch path is a latent conflict** — Both systemd and voice_mapper can
   start VSLAM. The "adoption" logic (check `vslam_tracking`) works but is fragile.
   VSLAM should be exclusively systemd-managed.

5. **The rosmaster-a1-robot ROS2 package is dead code** — Placeholder paths, build
   system conflicts, unused custom messages. It should be either completed as the
   proper deployment mechanism or removed to reduce confusion.

6. **No subprocess health monitoring** — voice_mapper launches Nav2/SLAM as subprocesses
   but never checks if their internal nodes remain healthy after startup. The
   `_kill_stale_nav2()` function is a cleanup mechanism, not a health check.

7. **Configuration is split across formats** — Python launch files, YAML params, shell
   scripts, and systemd env files. No single source of truth for deployment config.

## Phase 5: Target Architecture & Recommendations

**Scope:** Synthesize findings from Phases 1–4. Propose a target architecture with:
- Decomposed ROS2 nodes with clear single responsibilities
- Well-defined topic/service/action interfaces between nodes
- Closed-loop control where hardware allows (heading PID via IMU; velocity PID not possible)
- Clean sense → decide → act pipeline
- Migration path from current to target state

### 5.1 Open Question Resolution

Before designing the target architecture, several open questions from earlier phases were
investigated:

| Question | Answer | Evidence |
|----------|--------|---------|
| Does the base driver report actual wheel velocity? | **No** | `/vel_raw` is a command echo from `/driver_node`, not encoder feedback. No wheel encoder topics exist in the Yahboom stack. |
| Is there an IMU available? | **Yes — two IMUs** | Built-in MPU: `/imu/data_raw` → Madgwick filter → `/imu/data` at 10 Hz. OAK-D Pro BMI270: `/oak/imu/data` (enabled in `oakd_params.yaml` but unused by voice_mapper). |
| Does Nav2's velocity smoother remap cmd_vel? | **Yes** | `velocity_smoother` reads from `/cmd_vel_smoothed` and outputs to `/cmd_vel`. Feedback mode: `OPEN_LOOP`. |
| Base driver cmd_vel subscription rate? | **Unknown, likely on-demand** | Not documented in Yahboom stack. voice_mapper publishes at 20 Hz; EKF outputs `/odom` at 20 Hz. |
| Can velocity PID be implemented? | **Not with current hardware** | No wheel speed feedback exists. Would require encoder hardware or motor driver firmware changes. |
| Can heading PID be implemented? | **Yes** | `/imu/data` provides orientation at 10 Hz. `/odom` from EKF (which fuses wheel odom + IMU) at 20 Hz provides heading. Both available for closed-loop heading control. |

**Implication for target architecture:** The target must work within the constraint that
velocity control remains open-loop. Heading control CAN be closed-loop using the existing
IMU. The primary gains come from architectural decomposition, cmd_vel arbitration, and
consistent safety enforcement — not from adding PID velocity loops.

### 5.2 Design Principles

Based on findings across all phases, the target architecture adheres to these principles:

1. **Single writer to `/cmd_vel`** — One node arbitrates all velocity commands. No other
   node publishes directly to `/cmd_vel`. (Fixes: 10 uncoordinated publishers from Phase 1)

2. **Safety is non-bypassable** — Every velocity command passes through the same safety
   filter before reaching the base driver. No code path can skip it. (Fixes: inconsistent
   safety from Phases 1 and 3)

3. **ROS2-native lifecycle** — Use ROS2 timers, not sleep-based threads. Use lifecycle
   nodes where appropriate. Use standard interfaces (topics, services, actions) instead of
   direct attribute access. (Fixes: zero ROS2 timers, coupled internals from Phases 1 and 3)

4. **Subprocess elimination** — Nav2, SLAM, and VSLAM are systemd services, not
   voice_mapper children. (Fixes: orphan processes, fragile subprocess management from
   Phases 1 and 4)

5. **Immutable data boundaries** — Sensor data crosses node boundaries via ROS messages
   (inherently immutable copies). Within nodes, follow the `SensorSnapshot` pattern from
   Plan 006. (Fixes: thread safety issues from Phase 3)

6. **Explicit state machine** — Mode transitions are governed by a finite state machine
   with defined transitions, not ad-hoc boolean flag combinations. (Fixes: 13 boolean
   flags from Phase 1)

7. **Preserve what works** — The Plan 006 sense→decide→act pipeline, SafetyExecutor's
   two-layer model, LLMNavigator's stateless design, and the Gen 3 systemd architecture
   are good. Extend them; don't replace them.

### 5.3 Target Node Graph

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              SYSTEMD SERVICES                                   │
│                                                                                 │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐             │
│  │robot-base│ │robot-lidar│ │robot-cam │ │robot-tf  │ │robot-vslam│             │
│  │(existing)│ │(existing) │ │(existing)│ │(existing)│ │(existing) │             │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘             │
│       │             │            │             │             │                   │
│  NEW SERVICES:                                                                  │
│  ┌──────────┐ ┌──────────┐ ┌────────────────┐                                  │
│  │robot-nav2│ │robot-slam│ │robot-app       │                                  │
│  │(new svc) │ │(new svc) │ │(replaces       │                                  │
│  │          │ │          │ │ voice-mapper)  │                                  │
│  └──────────┘ └──────────┘ └────────────────┘                                  │
└─────────────────────────────────────────────────────────────────────────────────┘

HARDWARE LAYER (unchanged)                    APPLICATION LAYER (redesigned)
┌────────────────────────────┐                ┌──────────────────────────────────┐
│ /driver_node               │                │                                  │
│ /base_node          /odom ─┼──────────┐     │  ┌─────────────────────┐         │
│ /ekf_filter_node          │          │     │  │   sensor_aggregator │         │
│ /imu_filter_madgwick      │          │     │  │   (NEW NODE)        │         │
│                  /cmd_vel ◄┼──┐       │     │  │                     │         │
└────────────────────────────┘  │       │     │  │ Sub: /scan, /odom,  │         │
                                │       ├─────┼──┤ /oak/rgb, /oak/depth│         │
┌────────────────────────────┐  │       │     │  │ /imu/data           │         │
│ /sllidar_node              │  │       │     │  │                     │         │
│                    /scan ──┼──┼───────┼─────┼──┤ Pub: /sensor_state  │         │
└────────────────────────────┘  │       │     │  │      (custom msg)   │         │
                                │       │     │  └──────────┬──────────┘         │
┌────────────────────────────┐  │       │     │             │                    │
│ /oak (depthai)             │  │       │     │             ▼                    │
│         /oak/rgb/image ────┼──┼───────┼─────┼──┌──────────────────────┐        │
│         /oak/stereo/image ─┼──┼───────┼─────┼──│   obstacle_detector  │        │
│         /oak/imu/data ─────┼──┼───────┼─────┼──│   (NEW NODE)         │        │
└────────────────────────────┘  │       │     │  │                      │        │
                                │       │     │  │ Sub: /scan, /odom    │        │
┌────────────────────────────┐  │       │     │  │ Pub: /obstacles      │        │
│ /visual_slam_node          │  │       │     │  │      (custom msg)    │        │
│    /visual_slam/odom ──────┼──┼───────┘     │  │      /detected_gaps  │        │
└────────────────────────────┘  │             │  │      /emergency_stop │        │
                                │             │  └──────────┬───────────┘        │
                                │             │             │                    │
                                │             │             ▼                    │
                                │             │  ┌──────────────────────┐        │
                                │             │  │   velocity_arbiter   │        │
                                │             │  │   (NEW NODE)         │        │
                                │             │  │                      │        │
                                │             │  │ Sub: /cmd_vel_request │       │
                                │             │  │      /obstacles       │       │
                                │             │  │      /emergency_stop  │       │
                                │             │  │      /odom, /imu/data │       │
                          ┌─────┘             │  │                       │       │
                          │                   │  │ Pub: /cmd_vel (SOLE)  │       │
                          │                   │  │                       │       │
                          │                   │  │ Contains:             │       │
                          │◄──────────────────┼──│  - Safety filter      │       │
                          │                   │  │  - Heading PID        │       │
                          │                   │  │  - Priority arbiter   │       │
                          │                   │  └──────────┬───────────┘       │
                          │                   │             ▲                    │
                          │                   │             │ /cmd_vel_request   │
                          │                   │  ┌──────────┴───────────┐        │
                          │                   │  │                      │        │
                          │                   │  │  ┌─────────────┐    │        │
                          │                   │  │  │ brain_node   │    │        │
                          │                   │  │  │ (orchestrator)│    │        │
                          │                   │  │  │              │    │        │
                          │                   │  │  │ Sub: /sensor │    │        │
                          │                   │  │  │ _state,      │    │        │
                          │                   │  │  │ /voice_text  │    │        │
                          │                   │  │  │              │    │        │
                          │                   │  │  │ Pub:         │    │        │
                          │                   │  │  │ /cmd_vel_req │    │        │
                          │                   │  │  │ /speech_out  │    │        │
                          │                   │  │  │ /explore_cmd │    │        │
                          │                   │  │  └──────┬──────┘    │        │
                          │                   │  │         │           │        │
                          │                   │  │  ┌──────┴──────┐    │        │
                          │                   │  │  │ audio_node   │    │        │
                          │                   │  │  │              │    │        │
                          │                   │  │  │ Pub:         │    │        │
                          │                   │  │  │ /voice_text  │    │        │
                          │                   │  │  │ Sub:         │    │        │
                          │                   │  │  │ /speech_out  │    │        │
                          │                   │  │  └─────────────┘    │        │
                          │                   │  └─────────────────────┘        │
                          │                   └────────────────────────────────┘
```

### 5.4 Node Specifications

#### Node 1: `sensor_aggregator`

**Responsibility:** Subscribe to all raw sensor topics, package them into a single
synchronized message, publish at a consistent rate.

| Aspect | Detail |
|--------|--------|
| **Replaces** | VoiceMapper's 8 subscription callbacks + `_sensor_monitor_loop()` + SensorSnapshotBuilder |
| **Subscribes to** | `/scan` (LaserScan), `/odom` (Odometry), `/oak/rgb/image_raw` (Image), `/oak/stereo/image_raw` (Image), `/oak/left/image_rect` (Image), `/oak/right/image_rect` (Image), `/imu/data` (Imu), `/visual_slam/tracking/odometry` (Odometry), `/map` (OccupancyGrid) |
| **Publishes** | `/sensor_state` (custom `SensorState.msg`) at 10 Hz |
| **ROS2 timer** | 10 Hz timer to assemble and publish snapshot |
| **Thread safety** | Uses `message_filters.ApproximateTimeSynchronizer` for camera+depth, simple latest-value storage for others with lock-protected reads |
| **Health monitoring** | Publishes `sensor_health` diagnostic with staleness per-topic (replaces `_sensor_monitor_loop()` clearing to `None`) |

**Custom message: `SensorState.msg`**
```
std_msgs/Header header
sensor_msgs/Image rgb_image
sensor_msgs/Image depth_image
sensor_msgs/LaserScan scan
nav_msgs/Odometry odom
nav_msgs/Odometry vslam_odom
sensor_msgs/Imu imu
bool rgb_valid
bool depth_valid
bool scan_valid
bool odom_valid
bool vslam_valid
bool imu_valid
```

**Key design decision:** This node does NOT process sensor data (no sector decomposition,
no doorway detection). It just aggregates and publishes. Processing belongs in
`obstacle_detector`.

#### Node 2: `obstacle_detector`

**Responsibility:** Process LiDAR data into obstacle distances, detect doorways and gaps,
publish emergency stop signals.

| Aspect | Detail |
|--------|--------|
| **Replaces** | VoiceMapper's `scan_callback()` obstacle processing (lines 1033–1176), `_detect_doorways()`, SensorSnapshotBuilder's `build_lidar_summary()` |
| **Subscribes to** | `/scan` (LaserScan), `/odom` (Odometry) |
| **Publishes** | `/obstacles` (custom `ObstacleMap.msg`) at scan rate (~10 Hz), `/detected_gaps` (custom `GapArray.msg`), `/emergency_stop` (`std_msgs/Bool`) |
| **ROS2 timer** | None — callback-driven from `/scan` |
| **Eliminates** | Redundant dual LiDAR processing (Phase 3 finding 6) |

**Custom message: `ObstacleMap.msg`**
```
std_msgs/Header header
# Named sector distances (meters, inf = clear)
float32 front
float32 front_wide
float32 front_left
float32 front_right
float32 left
float32 right
float32 back
# 12-sector angular summary for VLM consumption
float32[12] sector_distances
string[12] sector_labels    # WALL/OBSTACLE/NEAR/CLEAR
# Affordance scores (0.0-1.0 per direction)
float32[6] affordance_scores
bool emergency
```

**Key design decision:** A single LiDAR processing pipeline produces BOTH the named
sectors (for safety filter) and the 12-sector summary (for VLM prompt). This eliminates
the Phase 3 finding of redundant processing from two independent sector decompositions.

#### Node 3: `velocity_arbiter` — THE critical new node

**Responsibility:** The SOLE publisher to `/cmd_vel`. Receives velocity requests from
multiple sources, applies priority arbitration, enforces safety, and optionally applies
heading PID.

| Aspect | Detail |
|--------|--------|
| **Replaces** | All 10 `/cmd_vel` publish paths from Phase 1, `_compute_safe_velocity()`, `emergency_stop()`, `_execute_uturn()` heading logic |
| **Subscribes to** | `/cmd_vel_request` (custom `VelocityRequest.msg`), `/obstacles` (ObstacleMap), `/emergency_stop` (Bool), `/odom` (Odometry), `/imu/data` (Imu) |
| **Publishes** | `/cmd_vel` (Twist) — **the ONLY node that publishes this topic** |
| **ROS2 timer** | 20 Hz control loop |
| **Parameters** | `safety.min_distance`, `safety.slow_distance`, `safety.emergency_distance`, `heading_pid.kp`, `heading_pid.ki`, `heading_pid.kd`, `arbitration.timeout_ms` |

**Custom message: `VelocityRequest.msg`**
```
std_msgs/Header header
geometry_msgs/Twist twist
uint8 priority          # 0=EMERGENCY, 1=SAFETY, 2=NAVIGATION, 3=EXPLORATION, 4=MANUAL
string source           # "brain", "nav2", "exploration", "emergency"
float32 duration_s      # How long this request is valid (0 = single-shot)
float32 target_heading  # NaN = no heading control; otherwise desired heading in radians
```

**Priority arbitration logic:**
```
EMERGENCY (0)  — emergency_stop signal from obstacle_detector
SAFETY (1)     — backup maneuver, controlled stop
NAVIGATION (2) — Nav2 path execution
EXPLORATION (3) — LLM exploration, reactive exploration
MANUAL (4)     — Voice commands ("move forward")
```

Higher-priority requests preempt lower-priority ones. Each request has a timeout — if no
new request arrives within `duration_s`, the arbiter publishes zero velocity. This
eliminates the "robot keeps moving if a thread crashes" problem.

**Heading PID controller (optional, per-request):**
When `target_heading != NaN`, the arbiter runs a PID loop:
- **Input:** Current heading from `/odom` (EKF-fused, 20 Hz) or `/imu/data` (10 Hz)
- **Setpoint:** `target_heading` from the request
- **Output:** Angular velocity component of the Twist
- **Gains:** Configurable via ROS2 parameters (tunable at runtime)
- **Use cases:** U-turns (replace open-loop 3-phase timing), `rotate` commands from
  LLMNavigator, any "turn to heading X" operation

This directly addresses Phase 1 finding 8 (open-loop heading control) and leverages the
IMU that Phase 5 research confirmed is available at 10 Hz.

**Safety filter (always active, non-bypassable):**
The existing `_compute_safe_velocity()` logic moves into this node. Every Twist that
reaches `/cmd_vel` has been safety-filtered. This addresses the Phase 3 finding that
safety was inconsistent across control paths.

Enhancement over current: add rear obstacle check using `back` sector from ObstacleMap.
This addresses Phase 1 finding 10 (backup-on-emergency has no reverse safety).

#### Node 4: `brain_node` (orchestrator)

**Responsibility:** High-level decision-making. Manages exploration modes, processes voice
commands, calls LLM APIs, coordinates Nav2 goals.

| Aspect | Detail |
|--------|--------|
| **Replaces** | VoiceMapper's `think()`/`execute()`, `exploration_loop()`, `vlm_decision_loop()`, `llm_control_loop()`, `start_exploration()`/`stop_exploration()`, Nav2 goal management |
| **Subscribes to** | `/sensor_state` (SensorState), `/obstacles` (ObstacleMap), `/voice_text` (String), `/nav2_result` (action feedback) |
| **Publishes** | `/cmd_vel_request` (VelocityRequest), `/speech_output` (String), `/explore_cmd` (String) |
| **Action clients** | `navigate_to_pose` (NavigateToPose) |
| **Contains** | LLMNavigator (imported), SafetyExecutor (adapted), ExplorationMemory, SensorSnapshotBuilder (adapted to use `/sensor_state` instead of direct attribute access) |

**State machine (replaces 13 boolean flags):**

```
                    ┌─────────┐
                    │  IDLE   │
                    └────┬────┘
                         │ start_exploration / voice "explore"
                    ┌────▼────┐
              ┌─────┤EXPLORING├─────┐
              │     └────┬────┘     │
              │          │          │
         ┌────▼───┐ ┌───▼────┐ ┌──▼──────┐
         │LLM_NAV │ │NAV2_FTR│ │REACTIVE │
         │        │ │        │ │         │
         └───┬────┘ └───┬────┘ └────┬────┘
              │          │           │
              │  fallback │  fallback │
              └──────────┘───────────┘
                         │
                    ┌────▼────┐
              ┌─────┤LISTENING├─────┐
              │     └─────────┘     │
         (pause exploration)   (resume)
              │                     │
              └─────────────────────┘

         Any state ──emergency──► EMERGENCY_STOP ──clear──► previous state
         Any state ──"stop"────► IDLE
```

States:
| State | Entry Action | Active Behavior | Exit Action |
|-------|-------------|-----------------|-------------|
| `IDLE` | Publish zero velocity | Respond to voice commands only | — |
| `EXPLORING` | Meta-state, enters one of 3 sub-states | — | Stop all sub-state activity |
| `LLM_NAV` | Start VLM decision timer + control timer | VLM decisions → SafetyExecutor → /cmd_vel_request | Stop timers |
| `NAV2_FRONTIER` | Send Nav2 goal | Monitor goal progress | Cancel Nav2 goal |
| `REACTIVE` | Start reactive timer | Obstacle-based direction → /cmd_vel_request | Stop timer |
| `LISTENING` | Pause exploration, publish zero vel | Capture audio, transcribe, LLM think | Resume exploration |
| `EMERGENCY_STOP` | Publish zero vel (priority 0) | Wait for obstacle clearance | Resume previous state |

**Key design change:** The state machine uses ROS2 timers for periodic behavior instead
of daemon threads with sleep loops. The VLM decision cycle becomes a timer callback at
adaptive frequency (using EventTrigger logic). The control loop becomes a separate timer
at 20 Hz. Both run in the ROS2 executor, eliminating the threading model entirely.

#### Node 5: `audio_node`

**Responsibility:** Microphone input, speech-to-text, text-to-speech. Isolated from all
robot control logic.

| Aspect | Detail |
|--------|--------|
| **Replaces** | VoiceMapper's `listen()`, `transcribe()`, `speak()`, `_find_microphone()`, `beep()` (~200 lines duplicated between VoiceMapper and YahboomExplorer) |
| **Subscribes to** | `/speech_output` (String) |
| **Publishes** | `/voice_text` (String) |
| **Parameters** | `mic_device_index`, `silence_threshold`, `tts_voice`, `whisper_model` |

This node runs its blocking audio I/O in its own process. It does not need access to any
sensor data, motor commands, or LLM APIs. This is the same architecture that
`llm_robot_brain.py` (Phase 2) correctly envisioned with `/voice_text` and
`/speech_output` topics.

### 5.5 Closed-Loop Control Opportunities

| Control Type | Feasible? | Input | Output | Addresses |
|-------------|-----------|-------|--------|-----------|
| **Heading PID** | **Yes** | `/imu/data` (10 Hz) or `/odom` heading (20 Hz) | Angular velocity in Twist | Open-loop turns/U-turns (Phase 1 finding 8) |
| **Velocity PID** | **No** | No wheel speed feedback available | — | Cannot fix. Hardware limitation. |
| **Wall-following PID** | **Yes** | `/obstacles` left/right sectors | Lateral velocity adjustment | Corridor navigation improvement |
| **Distance-to-goal PID** | **Yes** | `/odom` position | Linear velocity | Precise "move 1 meter" commands |

**Recommended implementation priority:**

1. **Heading PID** (high value, low risk) — Enables accurate turns and U-turns. Replace
   the current open-loop 3-phase U-turn (`_execute_uturn()`, 9 seconds of timed arcs)
   with a simple "turn to heading X" command. The arbiter reads heading from `/odom` at
   20 Hz and adjusts angular velocity with PID gains.

2. **Distance-to-goal PID** (medium value, low risk) — When `move()` is called with a
   target distance, use odometry to track actual distance traveled and stop precisely.
   Currently, `move()` runs for a caller-specified duration regardless of actual distance.

3. **Wall-following PID** (medium value, medium risk) — Useful for corridor exploration
   but requires tuning. Not critical given the LLM exploration mode handles this
   reasonably well.

### 5.6 Systemd Service Changes

**New services to add:**

| Service | Launch | Depends On | Restart Policy |
|---------|--------|------------|----------------|
| `robot-nav2` | `ros2 launch nav2_bringup navigation_launch.py params_file:=nav2_params.yaml` | base, lidar, tf | on-failure / 10s |
| `robot-slam` | `ros2 launch slam_toolbox online_sync_launch.py` | base, lidar, tf | on-failure / 10s |

**Service modifications:**

| Service | Change | Reason |
|---------|--------|--------|
| `robot-voice-mapper` | Rename to `robot-app`, remove subprocess launches for Nav2/SLAM/VSLAM | These become independent services |
| `robot-vslam` | Remove voice_mapper's `start_isaac_vslam()` path — VSLAM is exclusively systemd-managed | Eliminate dual launch conflict (Phase 4 finding 4) |
| All services | Consider `ExecStartPost=` readiness script that polls `ros2 topic list` | Restore readiness gating lost in Gen 1→Gen 3 migration (Phase 4 finding 3) |

**Updated dependency graph:**

```
                    robot.target
                         │
    ┌──────┬──────┬──────┼───────┬────────┬────────┬─────────┐
    ▼      ▼      ▼      ▼       ▼        ▼        ▼         ▼
  base   lidar  camera   tf    vslam     nav2     slam      app
                                          │        │         │
                                     After=       After=    After=
                                     base,lidar,  base,     ALL
                                     tf           lidar,tf
```

Nav2 and SLAM become `Wants=` (soft) from `robot-app`, since the application should work
in degraded mode without them. Nav2 lifecycle can be controlled via ROS2 lifecycle service
calls from `brain_node` rather than process start/stop.

### 5.7 Interface Contract Summary

All node-to-node communication through standard ROS2 interfaces:

```
sensor_aggregator
  ├── pub: /sensor_state (SensorState, 10 Hz)
  └── pub: /diagnostics (DiagnosticArray)

obstacle_detector
  ├── pub: /obstacles (ObstacleMap, ~10 Hz)
  ├── pub: /detected_gaps (GapArray, ~10 Hz)
  └── pub: /emergency_stop (Bool, event-driven)

velocity_arbiter
  ├── sub: /cmd_vel_request (VelocityRequest)
  ├── sub: /obstacles (ObstacleMap)
  ├── sub: /emergency_stop (Bool)
  ├── sub: /odom (Odometry)
  ├── sub: /imu/data (Imu)
  └── pub: /cmd_vel (Twist, 20 Hz) — THE ONLY cmd_vel publisher

brain_node
  ├── sub: /sensor_state (SensorState)
  ├── sub: /obstacles (ObstacleMap)
  ├── sub: /voice_text (String)
  ├── pub: /cmd_vel_request (VelocityRequest)
  ├── pub: /speech_output (String)
  └── action: navigate_to_pose (NavigateToPose)

audio_node
  ├── sub: /speech_output (String)
  └── pub: /voice_text (String)
```

**No node has direct attribute access to another node.** All data flows through ROS2
messages. This eliminates the tight coupling found in Phase 3 (SensorSnapshotBuilder
reading `vm.latest_image`, `vm.obstacle_distances`, etc.).

### 5.8 What This Architecture Preserves

The target architecture is not a rewrite from scratch. It preserves and extends the best
patterns found during research:

| Preserved Pattern | Source | How Extended |
|-------------------|--------|-------------|
| Sense→decide→act pipeline | Plan 006 modules | Applied system-wide, not just LLM exploration |
| Immutable sensor snapshots | `SensorSnapshot` dataclass | Becomes ROS2 message (`SensorState.msg`) — inherently immutable across node boundaries |
| SafetyExecutor two-layer safety | `safety_executor.py` | Layer 1 stays in brain_node; Layer 2 moves to velocity_arbiter and becomes non-bypassable |
| LLMNavigator stateless decisions | `llm_navigator.py` | Used as-is within brain_node — no changes needed |
| EventTrigger adaptive VLM frequency | `sensor_snapshot.py` | Becomes a ROS2 timer in brain_node with dynamic period |
| Discriminated union targets | `VelocityTarget`, `NavGoalTarget`, etc. | Becomes `VelocityRequest.msg` with priority field |
| Gen 3 multi-service systemd | `robot.target` + 6 services | Extended with `robot-nav2`, `robot-slam`, `robot-app` |
| Topic-based I/O architecture | `llm_robot_brain.py` (abandoned) | Revived: `/voice_text`, `/speech_output` for decoupled audio |
| Multi-provider LLM support | `llm_robot_brain.py` (abandoned) | Could be integrated into brain_node's LLM layer |
| Structured tool calls | `llm_navigator.py` | Continue using OpenAI function calling with `strict: true` |

### 5.9 What This Architecture Eliminates

| Eliminated | Reason | Replaced By |
|-----------|--------|-------------|
| 10 uncoordinated `/cmd_vel` publishers | Safety, correctness | Single `/cmd_vel` publisher in velocity_arbiter |
| 13 boolean state flags | Race conditions, inconsistent states | Finite state machine in brain_node |
| Sleep-based thread loops | Bypasses ROS2 executor, no lifecycle management | ROS2 timers at 10/20 Hz |
| Subprocess Nav2/SLAM management | Orphaned processes, fragile lifecycle | Systemd services with lifecycle service calls |
| Direct attribute access (`vm.latest_scan`) | Tight coupling, thread unsafety | ROS2 topic messages |
| Dual LiDAR processing | Redundant work | Single pipeline in obstacle_detector |
| Sensor monitor clearing data to None | Race condition with readers | Diagnostic publisher with staleness timestamps |
| Double-write to ExplorationMemory | Halved action history | Single writer (brain_node's control timer callback) |
| NetworkMonitor (vestigial) | Never acted upon | Remove, or integrate into brain_node's watchdog tier logic |
| `yahboom_explorer.py` (deprecated) | Dead code | Delete |
| `llm_robot_brain.py` (unused prototype) | Dead code (patterns salvaged) | Delete |
| `rosmaster-a1-robot/` package (dead code) | Placeholder paths, never deployed | Delete or complete as proper ROS2 package |

### 5.10 Migration Path

The migration should be incremental — each step produces a working system. No big-bang
rewrite.

**Phase A: Velocity Arbiter (highest-impact, lowest-risk first)**

| Step | Description | Validates |
|------|-------------|-----------|
| A1 | Create `velocity_arbiter` node with safety filter and priority arbitration. Initially subscribes directly to `/scan` for obstacle data (no obstacle_detector yet). | Single cmd_vel publisher works |
| A2 | Modify `voice_mapper.py` to publish `/cmd_vel_request` instead of `/cmd_vel` in ALL 10 paths. The arbiter is the only cmd_vel publisher. | All existing functionality still works through the arbiter |
| A3 | Add heading PID to the arbiter. Modify `_execute_uturn()` and rotate commands to send `target_heading` in VelocityRequest. | Closed-loop heading control works |

**Phase B: Sensor & Obstacle Decomposition**

| Step | Description | Validates |
|------|-------------|-----------|
| B1 | Create `obstacle_detector` node that subscribes to `/scan` and publishes `/obstacles`, `/detected_gaps`, `/emergency_stop`. | Obstacle data available as ROS topics |
| B2 | Modify arbiter to subscribe to `/obstacles` instead of `/scan` directly. | Arbiter uses obstacle_detector output |
| B3 | Create `sensor_aggregator` node. Modify brain_node's SensorSnapshotBuilder to subscribe to `/sensor_state` instead of reading `vm.*` attributes. | Decoupled sensor access |

**Phase C: Application Decomposition**

| Step | Description | Validates |
|------|-------------|-----------|
| C1 | Extract `audio_node` from voice_mapper. voice_mapper subscribes to `/voice_text` and publishes `/speech_output`. | Audio works through topics |
| C2 | Promote Nav2 and SLAM to systemd services. Replace `start_nav2()` / `start_slam()` subprocess calls with ROS2 lifecycle service calls. | Nav2/SLAM survive voice_mapper restart |
| C3 | Replace 13 boolean flags with a state machine class inside voice_mapper. | Mode transitions are explicit and safe |
| C4 | Rename service to `robot-app`. Replace all sleep-based loops with ROS2 timers. | Full ROS2-native lifecycle |

**Phase D: Cleanup**

| Step | Description |
|------|-------------|
| D1 | Delete `yahboom_explorer.py` |
| D2 | Delete `llm_robot_brain.py` |
| D3 | Delete or complete `rosmaster-a1-robot/` package |
| D4 | Consolidate configuration into YAML param files (externalize VSLAM params from launch file, unify LLM config formats) |
| D5 | Add `ExecStartPost=` readiness scripts to systemd services |
| D6 | Extract shared utilities (image encoding, LLM JSON parsing) into a `robot_utils` Python package |

### 5.11 Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Latency from additional ROS hops (sensor → aggregator → brain) | Medium | Low | At robot's 0.08–0.18 m/s speeds, even 10ms additional latency adds <2mm of travel. Safety loop in arbiter reads `/scan` directly for real-time response. |
| Heading PID oscillation | Medium | Medium | Start with conservative gains (low P, no I, moderate D). The arbiter can be tuned via ROS2 parameters at runtime without restart. |
| Nav2 cmd_vel conflict with arbiter | High | Medium | Nav2's velocity_smoother already outputs to `/cmd_vel`. Must remap Nav2 output to `/cmd_vel_request` or have the arbiter sit downstream of Nav2's smoother. Alternatively, brain_node sends Nav2 goals and Nav2's output goes through arbiter. |
| Migration breaks existing functionality | Medium | High | Each migration phase produces a working system. Keep the old code path available behind a parameter toggle until the new path is validated. |
| Increased system complexity (more nodes, more messages) | Certain | Low-Medium | Complexity is shifted from hidden (interleaved code in one file) to visible (explicit ROS interfaces). Visible complexity is manageable; hidden complexity is not. |

## Overview

This research systematically audited the robot's control architecture across 5 phases,
examining 6 Python files (~6,500 lines), 8 systemd service configurations, and the full
runtime topology of 10–20+ ROS2 nodes.

**The core problem:** A 3,396-line monolithic node (`VoiceMapper`) owns 14 responsibilities,
publishes to `/cmd_vel` through 10 uncoordinated code paths across multiple threads, uses
13 boolean flags instead of a state machine, and manages Nav2/SLAM/VSLAM as fragile
subprocesses. Safety enforcement is inconsistent — only 1 of 10 cmd_vel paths has the
full two-layer safety model.

**The bright spots:** The Plan 006 modules (`safety_executor.py`, `sensor_snapshot.py`,
`exploration_memory.py`) implement a clean sense→decide→act pipeline with immutable
snapshots, a validation/translation layer, and discriminated union targets. The Gen 3
multi-service systemd architecture properly isolates hardware drivers. `LLMNavigator`
demonstrates clean stateless design with structured tool calls.

**The proposed solution:** Decompose VoiceMapper into 5 focused nodes communicating through
ROS2 topics: `sensor_aggregator` (data collection), `obstacle_detector` (LiDAR processing),
`velocity_arbiter` (sole `/cmd_vel` publisher with safety + heading PID), `brain_node`
(LLM orchestration + state machine), and `audio_node` (speech I/O). Nav2 and SLAM become
independent systemd services. The migration is incremental — each step yields a working
system, starting with the velocity arbiter (highest-impact, lowest-risk).

**Hardware constraint discovered:** Velocity PID is not feasible — the Yahboom base driver
provides no wheel speed feedback. However, heading PID IS feasible using the existing
IMU (10 Hz) and EKF-fused odometry (20 Hz), which enables closed-loop turns to replace
the current open-loop timed maneuvers.

## Key Findings

### From Phase 1 — Monolith Anatomy
- **14 responsibilities** in a single 3,396-line `VoiceMapper` class
- **10 uncoordinated cmd_vel publishers** with no arbitration layer
- **Zero ROS2 timers** — all periodic work is sleep-based threads bypassing the executor
- **No closed-loop control** — `_compute_safe_velocity()` is a reactive filter, not a PID
- **13 boolean flags instead of a state machine** — concurrent transitions are unsafe
- **Open-loop heading** — no IMU/gyro feedback for turns or U-turns
- **Fragile subprocess management** for SLAM/Nav2/VSLAM — no health monitoring after startup
- **Sensor monitor race condition** — clears `latest_scan` to detect staleness, but other threads may read None

### From Phase 2 — Parallel Scripts & Duplication Audit
- **Only 2 of 4 scripts are active** — `yahboom_explorer.py` (deprecated) and `llm_robot_brain.py` (unused prototype) are dead code
- **~350 lines duplicated** between VoiceMapper and YahboomExplorer (audio I/O, scan processing, odom, image encoding)
- **LLMRobotBrain's topic-based architecture was the right direction** but abandoned in favor of growing the monolith
- **LLMNavigator is the cleanest module** — stateless, structured tool calls, single responsibility
- **Three different safety approaches, none complete** — VoiceMapper (proportional, no rear), YahboomExplorer (binary, has rear), LLMRobotBrain (none)
- **No shared libraries** — each script independently reimplements common functionality
- **Lost features** — color following, depth query caching, multi-provider LLM, declarative action library

### From Phase 3 — Safety, Sensing & Decision Pipeline
- **Plan 006 modules implement a proper sense→decide→act pipeline** — Immutable snapshots, validation layer, discriminated union targets, single cmd_vel publisher. The cleanest architecture in the codebase.
- **Two-layer safety only protects LLM exploration path** — SafetyExecutor pre-validation + `_compute_safe_velocity()` together are robust, but the other 9 cmd_vel paths have weaker or no safety
- **No thread synchronization on sensor state** — SensorSnapshotBuilder, SafetyExecutor, EventTrigger all read mutable sensor fields without locks
- **ExplorationMemory double-written** — Both VLM and control loops call `update_after_action()` for same decision, halving the effective 5-action history window
- **NetworkMonitor is vestigial** — Tracks API health but no code acts on the tiers; only the wall-clock watchdog triggers degradation
- **Redundant LiDAR processing** — Raw scan processed into sectors twice (6 named sectors in scan_callback, 12 angular sectors in build_lidar_summary) from the same data
- **End-to-end latency 500–2050ms** — API-dominated; local safety path is 0–50ms, adequate for robot's 0.08–0.18 m/s speeds
- **EventTrigger adaptive frequency is a good pattern** — Fires VLM calls on doorways, intersections, dead ends, movement; but shares thread with API calls
- **Module design patterns are a blueprint** — Immutable snapshots, validation layers, discriminated unions, queue decoupling should be applied system-wide

### From Phase 4 — Systemd Services, Launch Files & Runtime Topology
- **Gen 3 multi-service architecture is a good foundation** — 6 systemd services with proper `Wants`/`Requires`/`After` ordering, independent restart policies, `control-group` KillMode
- **Nav2 and SLAM are subprocess children of voice_mapper, not systemd services** — The biggest isolation gap. When voice_mapper crashes, Nav2/SLAM are orphaned. `_kill_stale_nav2()` exists to clean up previous instances' orphans, confirming this is a known problem
- **Readiness gating lost in Gen 1→Gen 3 migration** — `start_robot.sh` polled for `/scan`, `/odom`, `/oak/rgb/image_raw` before proceeding. Gen 3 `Type=simple` services have no readiness verification
- **Dual VSLAM launch path** — Both systemd (`robot-vslam.service`) and voice_mapper (`start_isaac_vslam()`) can launch VSLAM. Adoption logic exists but is fragile
- **Runtime topology varies wildly: 10–20+ nodes** — 10 steady-state (hardware) + up to 10 more when Nav2/SLAM are dynamically launched by voice_mapper
- **rosmaster-a1-robot/ package is dead code** — Placeholder paths, build system conflicts, unused custom msg/srv. Never deployed
- **No subprocess health monitoring** — voice_mapper checks Nav2 lifecycle at startup but never again. SLAM uses a 3-second sleep with no verification at all

## Actionable Conclusions

1. **Implement a velocity arbiter as the single `/cmd_vel` publisher** — This is the
   highest-impact change. It enforces safety universally, eliminates race conditions
   between threads publishing to the same topic, and provides a natural insertion point
   for heading PID. Start here.

2. **Add heading PID using existing IMU** — The robot has two IMUs (built-in MPU at 10 Hz,
   OAK-D BMI270). EKF-fused `/odom` provides heading at 20 Hz. Use this to close the
   loop on turns and U-turns, replacing the open-loop 9-second timed arc maneuver.

3. **Promote Nav2 and SLAM to systemd services** — Stop spawning them as voice_mapper
   subprocesses. This eliminates orphaned processes, enables independent restart, and
   decouples the navigation stack lifecycle from the application. Use ROS2 lifecycle
   service calls for start/stop control.

4. **Replace boolean flags with a state machine** — The 13-flag system allows inconsistent
   states during concurrent transitions. A finite state machine with explicit transitions
   makes mode management safe and debuggable.

5. **Extract audio into a separate node** — Audio I/O is blocking and completely
   independent of robot control. Isolating it in its own process (with `/voice_text` and
   `/speech_output` topics) eliminates the threading complexity and deduplicates ~200
   lines shared with the deprecated YahboomExplorer.

6. **Unify LiDAR processing in one node** — The current dual processing (6 named sectors
   in `scan_callback` + 12 angular sectors in `build_lidar_summary`) wastes CPU and can
   diverge. A single `obstacle_detector` node produces both formats.

7. **Delete dead code** — `yahboom_explorer.py` (deprecated), `llm_robot_brain.py`
   (never deployed), and `rosmaster-a1-robot/` (dead package) add confusion without value.
   Salvage useful patterns (multi-provider LLM, topic-based architecture, color following)
   into the new architecture before deleting.

8. **Fix ExplorationMemory double-write** — Both VLM and control loops currently write to
   `recent_actions` for the same decision. Remove the premature write in `vlm_decision_loop`
   (which records `{'success': True}` before safety validation). Only the control loop
   should record action outcomes.

9. **Restore readiness gating** — Add `ExecStartPost=` scripts to systemd services that
   poll `ros2 topic list` for required topics before declaring ready. This restores the
   capability lost in the Gen 1→Gen 3 migration.

10. **Do NOT attempt velocity PID** — The hardware has no wheel speed feedback. Focus
    control improvements on heading PID (IMU-based) and distance-to-goal (odometry-based),
    which are both feasible with existing sensors.

## Open Questions

1. **Does the robot hardware support velocity feedback?** — If the motor controller reports actual wheel speed, a PID velocity loop becomes feasible. Need to check base driver topics.
2. **Is there an IMU available?** — The OAK-D Pro has an IMU (`/oak/imu/data` is subscribed but never used in VoiceMapper). Could enable closed-loop heading control.
3. **~~Are `yahboom_explorer.py` and `llm_robot_brain.py` still used?~~** — **Answered in Phase 2:** `yahboom_explorer.py` is explicitly deprecated (HP60C camera removed), `llm_robot_brain.py` was never deployed. Both are dead code.
4. **How does Nav2's cmd_vel interact with VoiceMapper's cmd_vel?** — When Nav2 is active, both may publish. Need to check if Nav2's velocity smoother is in the path.
5. **What is the actual cmd_vel subscription rate of the base driver?** — Determines whether 20 Hz publishing is sufficient or excessive.
6. **Should NetworkMonitor tiers drive actual behavior?** — The monitor exists but is never acted upon. Should degraded network tier reduce exploration speed or trigger more conservative fallbacks?
7. **Can the double-write to ExplorationMemory be resolved?** — Both VLM and control loops update `recent_actions` for the same decision. The VLM loop's write (with `{'success': True}`) is premature since safety hasn't validated yet.
8. **Should Nav2 and SLAM become independent systemd services?** — Discovered in Phase 4. This would require extracting the start/stop logic from voice_mapper and creating `run_nav2.sh` / `run_slam.sh` wrappers. The application layer would then use ROS2 lifecycle or action interfaces instead of subprocess management.
9. **Should `Type=notify` or readiness scripts replace `Type=simple`?** — Gen 3 lost the readiness gating that Gen 1 had. `ExecStartPost=` with a topic-check script could restore it without requiring `sd_notify` integration.

## Handoff

**Research Status:** ✅ Complete — all 5 phases finished.

**Recommended next step:** Use `/pch-planner` to create an implementation plan for the
velocity arbiter (Phase A of the migration path in Section 5.10). This is the
highest-impact, lowest-risk change and produces immediate safety and correctness benefits.

**Scope for initial planning:**
- A1: Create `velocity_arbiter` node with safety filter and priority arbitration
- A2: Modify voice_mapper.py to publish `/cmd_vel_request` in all 10 paths
- A3: Add heading PID with IMU feedback for turns/U-turns

**Dependencies:** None — the arbiter can be added alongside the existing system and
progressively take over `/cmd_vel` publishing.

**Research document:** [008-control-loop-architecture-review.md](docs/research/008-control-loop-architecture-review.md)
