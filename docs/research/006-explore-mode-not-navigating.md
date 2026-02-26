---
id: "006"
type: research
title: "Explore Mode Not Navigating — Robot Stationary or Moves Backwards"
status: ✅ Complete
created: "2026-02-25"
current_phase: "5 of 5"
---

## Introduction

The robot's explore mode (triggered by `{"action": "explore"}`) exhibits three symptoms:
1. **Robot doesn't move** — stays stationary despite being told to explore
2. **Moves backwards only** — when it does move, it reverses a few cm then stops
3. **Loops on descriptions** — the LLM repeatedly describes what it sees without issuing navigation commands

The explore mode implementation lives primarily in `scripts/voice_mapper.py` (~2893 lines).
The deprecated `scripts/yahboom_explorer.py` is kept for reference only.

## Objectives

1. Identify **why the robot fails to issue forward movement commands** during exploration
2. Determine **why backwards movement occurs** (reversed velocity signs? safety fallback?)
3. Understand **why the LLM loop describes but doesn't navigate** (prompt issue? action parsing? feedback loop?)
4. Map the complete **explore data flow**: user command → LLM → action parsing → Nav2/cmd_vel → motion
5. Document **all potential root causes** ranked by likelihood
6. Provide actionable conclusions for a `/pch-planner` handoff

## Research Phases

| Phase | Name | Status | Scope | Session |
|-------|------|--------|-------|---------|
| 1 | Exploration Loop & State Machine | ✅ complete | `exploration_loop()`, `start_exploration()`, `stop_exploration()`, `_random_exploration_walk()`, exploration state flags | 1 |
| 2 | Navigation & Movement Pipeline | ✅ complete | `navigate_to()`, `find_frontiers()`, `choose_frontier()`, `choose_exploration_direction()`, Nav2 integration, `cmd_vel` publishing | 1 |
| 3 | LLM Decision Loop & Action Parsing | ✅ complete | LLM system prompt, `process_llm_response()`, action parsing for "explore", LLM context/history management, observation loop timing | 1 |
| 4 | Safety Systems & Obstacle Handling | ✅ complete | Emergency stop logic, LiDAR callbacks, `min_front_distance`, obstacle avoidance during exploration, costmap/Nav2 params | 1 |
| 5 | Synthesis & Root Cause Analysis | ✅ complete | Cross-phase correlation, root cause ranking, data flow diagram, actionable fix recommendations | 1 |

## Phase 1: Exploration Loop & State Machine

**Status**: ✅ Complete

### 1.1 Entry Point — How Explore Mode Starts

When the user says "explore", the LLM returns `{"action": "explore"}`. The `execute()` method (L2179) handles it at L2244-2251:

```python
# voice_mapper.py:2244-2251
elif action == "explore":
    if speech:
        self.speak(speech)
    if not self.mapping:
        self.speak("Starting mapping first...")
        self.start_slam()
        time.sleep(2)
    self.start_exploration()
```

`start_exploration()` at L2674-2680:
```python
def start_exploration(self):
    if self.exploring:
        return             # <-- guard: no-op if already exploring
    self.exploring = True
    self.stuck_counter = 0
    self.explore_thread = threading.Thread(target=self.exploration_loop, daemon=True)
    self.explore_thread.start()
```

**Key finding**: Exploration runs on a **daemon thread** separate from the voice loop.

### 1.2 State Flags

| Flag | Init Value | Set True | Set False | Purpose |
|------|-----------|----------|-----------|---------|
| `self.exploring` (L328) | `False` | `start_exploration()` L2677 | `stop_exploration()` L2684, shutdown L2858 | Controls explore while-loop |
| `self.nav2_available` (L427) | `False` | `start_nav2()` L524 (when bt_navigator is `active`) | `stop_nav2()` L553 | Gates Nav2 vs reactive mode |
| `self.navigating` (L429) | `False` | `navigate_to()` L574 | nav goal complete L590, fail L629, cancel L638 | Tracks if Nav2 goal is active |
| `self.nav_feedback` (L432) | `None` | nav feedback callback L601 | — | Distance/time for stuck detection |
| `self.mapping` (L329) | `False` | `start_slam()` | `stop_slam()` | SLAM active flag |

### 1.3 The Exploration Loop — Three Distinct Paths

`exploration_loop()` at L2325 is the primary loop. It has a critical **branching decision** at the top:

```
exploration_loop() called
├── Nav2 NOT available? (L2339)
│   ├── start_nav2() succeeds? → continue to main while-loop
│   └── start_nav2() fails? → _try_nav2_with_reactive_fallback() → RETURN (exits loop!)
│       ├── Nav2 becomes available within 30s → re-enters exploration_loop() (RECURSION!)
│       └── Nav2 never available → _reactive_exploration_loop()
│
└── Nav2 IS available (or just became so) → main while-loop
    ├── Every 15s: spawn _observe_in_background() thread
    ├── If not self.navigating:
    │   ├── choose_frontier() returns a frontier → navigate_to(fx, fy)
    │   │   ├── success → consecutive_failures = 0
    │   │   └── fail → consecutive_failures++, if >=5: clear costmaps + random walk
    │   └── choose_frontier() returns None → no_frontier_count++
    │       └── if >=3: _random_exploration_walk(), reset counter
    ├── sleep(0.5)
    └── If navigating: check stuck (>30s, still >0.5m away) → cancel + clear costmaps
```

### 1.4 Critical Issue #1: Nav2 Failure Cascades to Reactive Mode (Most Likely "No Movement" Path)

If Nav2 fails to start (L2341), the code calls `_try_nav2_with_reactive_fallback()` (L2528) and **returns immediately** (L2345). This means the main `exploration_loop()` **exits**.

`_try_nav2_with_reactive_fallback()` (L2528-2550):
1. Starts `start_nav2()` in a background thread
2. Runs `_reactive_exploration_step()` in a 30-second polling loop
3. If Nav2 comes online → **recursively calls `exploration_loop()`** (L2539) — potential stack issue
4. If Nav2 never starts → calls `_reactive_exploration_loop()` (L2550)

**The reactive path is where the robot likely ends up.** Nav2 requires a map (SLAM), bt_navigator lifecycle to reach `active` state, and has a 60-second timeout.

### 1.5 Critical Issue #2: Reactive Exploration — Backwards-Only Movement

`_reactive_exploration_step()` (L2552-2574) is the **single-step reactive fallback**:

```python
def _reactive_exploration_step(self):
    min_front = min(front, front_wide, fl, fr)
    twist = Twist()
    if min_front < self.min_obstacle_dist:    # < 0.5m
        twist.linear.x = -0.05               # ← BACKWARDS
        twist.angular.z = 0.4 if left > right else -0.4
    else:
        twist.linear.x = 0.1                 # forward 0.1 m/s
        twist.angular.z = random.uniform(-0.1, 0.1)
    self.cmd_vel_pub.publish(twist)
```

**If LiDAR reports any front sector < 0.5m, the robot goes backwards at -0.05 m/s.** Given that `min_obstacle_dist = 0.5m` and the LiDAR scans a wide 90° arc for the "front" zone (±45°), **side-of-robot obstacles within 0.5m will trigger backward movement even if the path ahead is clear.**

### 1.6 Critical Issue #3: Reactive Loop Also Has Backwards Movement

`_reactive_exploration_loop()` (L2576-2661) — the full reactive fallback:
- At L2606: if `min_front < self.min_obstacle_dist` → enters obstacle avoidance
- At L2616-2623: if stuck_count > 5 → **backs up at -0.1 m/s for 1.5 seconds**
- At L2627-2628: normal obstacle turn uses `twist.linear.x = -0.05` — **backwards again**
- Comment says "Ackerman can't turn in place, use gentle reverse so steering actually changes heading"

**The Ackerman reverse-to-turn pattern is the source of the "backwards a few cm" behavior.** Every time an obstacle is within 0.5m of any front sector, the robot reverses.

### 1.7 The Observation Loop — Describe-Without-Moving

During exploration, `_observe_in_background()` runs every 15 seconds (L2349-2351):

```python
if time.time() - last_observation >= self.look_interval:
    last_observation = time.time()
    threading.Thread(target=self._observe_in_background, daemon=True).start()
```

`_observe_in_background()` (L2316-2323):
```python
def _observe_in_background(self):
    observation, _ = self.observe()
    if observation:
        self.speak(observation)
```

`observe()` (L1885) makes a GPT-4o vision API call and speaks the result. This **does not issue any movement commands**. The observation itself is purely descriptive — it says what the robot sees but does not trigger navigation.

**However**, the voice loop (L2691-2708) runs **concurrently** on another thread. If the user speaks during exploration, `think()` → `execute()` is called. If the LLM responds with `{"action": "look"}` or `{"action": "speak"}`, no movement happens. The exploration thread is the **only** thing moving the robot.

### 1.8 The `_random_exploration_walk()` — Actually Forward-Moving

When frontiers aren't found (no_frontier_count >= 3), the exploration loop calls `_random_exploration_walk()` (L2417). This function:

1. **Priority 1**: If `self.detected_gaps` exists, drives toward the widest gap at 0.15 m/s (L2445)
2. **Fallback**: Finds direction with most open space, drives forward at 0.12 m/s (L2509)

This is one of the few paths that actually drives **forward**. But it only runs when frontier detection fails 3+ times consecutively.

### 1.9 Timing Analysis

| Operation | Timing | Impact |
|-----------|--------|--------|
| Main loop sleep | 0.5s per iteration (L2392) | Reasonable |
| Observation interval | Every 15s (L2359, `look_interval`) | Each GPT-4o call + TTS takes ~3-5s |
| No-frontier retry | 2s sleep (L2369) | 3 retries × 2s = 6s before random walk |
| Nav2 startup timeout | 60s (L2514 in start_nav2) | Robot sits still for up to 60s waiting |
| Reactive fallback wait | 30s max (L2536) | During this, only `_reactive_exploration_step()` runs |
| Stuck detection threshold | 30s navigating with >0.5m remaining (L2400) | Long time before detecting stuck |

### 1.10 Phase 1 Summary — Root Cause Hypotheses

| # | Hypothesis | Evidence | Confidence |
|---|-----------|----------|------------|
| H1 | **Nav2 never becomes `active`** → robot falls through to reactive mode which barely moves | Nav2 requires SLAM map, bt_navigator lifecycle, 60s timeout at L514 | ⭐⭐⭐⭐⭐ |
| H2 | **Wide front LiDAR cone (±45°) triggers obstacle avoidance** even when path ahead is clear → robot reverses | `min_front = min(front, front_wide, fl, fr)` uses ±45° sectors + ±30° front_wide; obstacle threshold is 0.5m | ⭐⭐⭐⭐ |
| H3 | **Ackerman reverse-to-turn** design means every obstacle avoidance step moves backwards | `twist.linear.x = -0.05` in both `_reactive_exploration_step` and `_reactive_exploration_loop` | ⭐⭐⭐⭐ |
| H4 | **Observation loop speaks every 15s** making it appear the robot "loops describing" while exploration thread struggles silently | `_observe_in_background` runs on schedule regardless of movement | ⭐⭐⭐⭐ |
| H5 | **No forward movement in reactive step** when surrounded by objects within 0.5m (furniture, walls) | Typical indoor environment has walls/objects within 0.5m on sides | ⭐⭐⭐ |

### Files Examined
- `scripts/voice_mapper.py`: L320-370 (state init), L425-555 (Nav2 init/start/stop), L993-1082 (scan callback), L1885-1934 (observe), L2022-2084 (move), L2085-2124 (choose_exploration_direction), L2128-2237 (think/execute/context), L2244-2265 (explore action handler), L2316-2415 (exploration_loop), L2417-2527 (random walk), L2528-2673 (reactive fallback), L2674-2688 (start/stop exploration), L2691-2708 (voice loop)

## Phase 2: Navigation & Movement Pipeline

**Status**: ✅ Complete

### 2.1 `navigate_to()` — Async Fire-and-Forget (L556-583)

```python
def navigate_to(self, x, y, theta=0.0):
    if not self.nav2_available:
        if not self.start_nav2():
            return False
    # ... build goal_msg with frame_id="map" ...
    self.navigating = True
    send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=...)
    send_goal_future.add_done_callback(self._nav_goal_response_callback)
    return True   # ← returns IMMEDIATELY, doesn't wait for completion
```

**Critical finding**: `navigate_to()` is **non-blocking**. It sends the goal and returns `True` right away. The exploration loop then relies on `self.navigating` being flipped to `False` by the async callbacks.

**Problem**: `navigate_to()` always returns `True` if Nav2 is available — even if the goal is later **rejected** by bt_navigator. The exploration loop at L2379 treats `return True` as "navigation succeeded" (resets `consecutive_failures`), but the goal may actually be rejected asynchronously at L2588-2590 (which only sets `self.navigating = False`).

### 2.2 Nav2 Goal Lifecycle and Failure Modes

The async callback chain:
1. `_nav_goal_response_callback` (L585) — checks `goal_handle.accepted`
   - If **rejected**: `self.navigating = False`, no notification to exploration loop
   - If **accepted**: starts waiting for result
2. `_nav_feedback_callback` (L598) — stores `distance_remaining` and `time_elapsed`
3. `_nav_result_callback` (L606) — handles SUCCESS, CANCELED, ABORTED, REJECTED
   - Sets `self.navigating = False` in all cases

**Problem**: There is no mechanism to report goal failure back to the exploration loop other than `self.navigating` becoming `False`. The loop checks `not self.navigating` (L2354) and picks a new frontier, but doesn't know *why* the last goal failed. If goals are repeatedly rejected (e.g., bt_navigator not fully configured), the loop just keeps trying new frontiers with no backoff or mode change.

### 2.3 cmd_vel Topic Chain — Potential Conflict

The voice_mapper publishes directly to `/cmd_vel` (L243):
```python
self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
```

Nav2's velocity chain (from `nav2_params.yaml`):
```
controller_server → /cmd_vel_smoothed → velocity_smoother → /cmd_vel_smoothed
collision_monitor: cmd_vel_in_topic: "cmd_vel_smoothed" → cmd_vel_out_topic: "cmd_vel"
```

**Both Nav2 (via collision_monitor) and voice_mapper publish to `/cmd_vel`.** During Nav2 exploration, if the reactive fallback or any voice_mapper code publishes a stop/reverse command to `/cmd_vel`, it **conflicts with Nav2's own commands**. The last writer wins on ROS2 topics.

However, during the main exploration loop (L2347-2409), voice_mapper does NOT publish directly to cmd_vel — it only calls `navigate_to()` which lets Nav2 handle motion. The conflict only occurs if:
- The reactive fallback runs concurrently with Nav2
- `_random_exploration_walk()` runs while Nav2 is still commanding

### 2.4 Frontier Detection — Requires SLAM Map

`find_frontiers()` (L641-692) is algorithmically sound:
- Vectorized numpy operations on occupancy grid
- Finds free cells (value=0) adjacent to unknown cells (value=-1)
- Clusters using spatial hashing, sorts by size (larger = likely doorways)

**But it has a hard dependency on `self.latest_map`** (L647):
```python
if self.latest_map is None:
    self.get_logger().warning("No map available for frontier detection")
    return []
```

`self.latest_map` is populated by `map_callback` (L1156) subscribing to `/map` topic. This only produces data when SLAM is running. If SLAM startup failed or is delayed, `find_frontiers()` returns `[]` → `choose_frontier()` returns `None` → exploration loop increments `no_frontier_count`.

After 3 `None` returns, it falls through to `_random_exploration_walk()`, which is the only forward-motion path.

### 2.5 `choose_frontier()` — Filtering May Discard All Candidates

`choose_frontier()` (L755-836) applies multiple filters:
1. **Visited filter**: Skips frontiers within 0.8m of `self.visited_goals` (L772-776)
2. **Distance filter**: Skips if dist < 0.5m or dist > 10.0m (L797-800)
3. **Scoring**: Prefers 2-5m distance, larger clusters

**Potential issue**: If the robot's position tracking is wrong (odometry drift, SLAM not running), `self.current_position` may be stale at `(0, 0)` (initial value from L336). This means:
- All frontiers might appear at incorrect distances
- The 0.5-10m filter may reject valid frontiers
- Scored frontiers may be ranked incorrectly

### 2.6 Nav2 Configuration Analysis

**AMCL (L4-48)**: Uses `DifferentialMotionModel` — **incorrect for Ackerman steering**. AMCL's differential model assumes the robot can rotate in place. For an Ackerman robot, this means AMCL's particle filter will produce poor localization, especially during turns where the motion model diverges from actual motion.

**Controller (L109-155)**: `RegulatedPurePursuitController` with:
- `desired_linear_vel: 0.15` — reasonable
- `use_rotate_to_heading: false` — correct for Ackerman
- `allow_reversing: true` — allows backward motion
- `min_approach_linear_velocity: 0.05` — very slow approach speed

**Planner (L270-300)**: `SmacPlannerHybrid` with:
- `motion_model_for_search: "REEDS_SHEPP"` — allows forward + reverse, correct for Ackerman
- `minimum_turning_radius: 0.30` — matches Ackerman constraint
- `reverse_penalty: 2.1` — prefers forward but allows reverse
- `allow_unknown: true` — good for exploration

**Behavior server (L312-334)**:
- **Spin removed** (correct for Ackerman)
- **Backup included** — Nav2's recovery behavior will back up the robot
- This is another source of backward movement during recovery

**Velocity smoother (L336-349)**:
- `max_velocity: [0.15, 0.0, 0.5]` — reasonable
- `min_velocity: [-0.15, 0.0, -0.5]` — allows reverse at full speed

**Collision monitor (L362-397)**:
- Uses `FootprintApproach` with `time_before_collision: 1.2`
- Monitors both LiDAR and point cloud
- This is an **additional** safety layer on top of the costmap

### 2.7 Critical Issue: AMCL Motion Model Mismatch

`nav2_params.yaml:32`:
```yaml
robot_model_type: "nav2_amcl::DifferentialMotionModel"
```

The robot is Ackerman, not differential. Nav2's AMCL differential motion model predicts that the robot can rotate in place, but Ackerman robots cannot. This causes:
1. **Poor localization** — particle filter predictions diverge from actual motion
2. **Navigation failures** — Nav2 thinks it's somewhere it's not
3. **Path planning conflicts** — planned paths assume differential kinematics while the planner uses Ackerman

Note: Nav2 AMCL doesn't have a native Ackerman motion model. The closest option is `"nav2_amcl::OmniMotionModel"` which is actually worse. The differential model is the "least bad" option for AMCL with Ackerman, but it still introduces localization error.

### 2.8 Costmap Configuration — Depth Camera Dependency

Both local and global costmaps configure two observation sources:
```yaml
observation_sources: scan depth_camera
depth_camera:
    topic: /oak/points
    data_type: "PointCloud2"
```

If the OAK-D camera is not publishing `/oak/points` (e.g., USB issues documented in research 002-005), the costmap **may stall or produce incomplete obstacle data**. The costmap won't crash, but missing point cloud data means obstacles visible only to the camera (not LiDAR) won't be in the costmap.

### 2.9 Phase 2 Summary — Additional Root Causes

| # | Hypothesis | Evidence | Confidence |
|---|-----------|----------|------------|
| H6 | **`navigate_to()` fire-and-forget** — returns True immediately but goals may be rejected async; exploration loop doesn't detect this properly | L574 sets `navigating=True`, L578-583 sends async, returns True; rejection only caught in callback L588-590 | ⭐⭐⭐⭐ |
| H7 | **AMCL uses DifferentialMotionModel for Ackerman robot** — localization drift causes Nav2 to reject/abort goals | `nav2_params.yaml:32`, A1 is Ackerman, not differential | ⭐⭐⭐ |
| H8 | **No SLAM map → no frontiers → no navigation goals** — robot falls to random walk only | `find_frontiers()` returns `[]` if `latest_map is None` (L647); SLAM startup is async with 3s sleep | ⭐⭐⭐⭐ |
| H9 | **Nav2 backup recovery behavior** contributes to backward movement | `behavior_server` includes `backup` plugin (L319-320); recovery BT includes `nav2_back_up_action_bt_node` | ⭐⭐⭐ |
| H10 | **Position starts at (0,0)** until first odom callback — frontier distance filter may reject valid frontiers early on | `current_position` init at L336 is `{'x': 0, 'y': 0}`; frontier filter rejects dist < 0.5m or > 10m | ⭐⭐ |

### Files Examined
- `scripts/voice_mapper.py`: L243 (cmd_vel pub), L299-311 (subscriptions), L556-640 (navigate_to + callbacks), L641-753 (frontier detection + clustering), L755-836 (choose_frontier), L968-992 (VSLAM odom), L1137-1164 (odom + map callbacks), L1352-1431 (SLAM start/stop)
- `scripts/nav2_params.yaml`: Full file (L1-397) — AMCL, bt_navigator, controller, planner, costmaps, behavior server, velocity smoother, collision monitor

## Phase 3: LLM Decision Loop & Action Parsing

**Status**: ✅ Complete

### 3.1 Two Separate LLM Systems — Only One Is Active

There are **two independent LLM implementations** in the codebase:

1. **`voice_mapper.py`** — The active system. Contains its own GPT-4o integration via `self.client` (OpenAI SDK). This is what runs during exploration.
2. **`llm_robot_brain.py`** — A **standalone, unused** ROS2 node (`LLMRobotBrain`). It has its own system prompt, action library, and ROS2 topic-based architecture (`/voice_text`, `/robot_action`, `/speech_output`). It is **not imported or referenced** by `voice_mapper.py`.

**Conclusion**: `llm_robot_brain.py` is irrelevant to the explore mode bug. All LLM logic runs inside `voice_mapper.py`.

### 3.2 The LLM Is NOT Called During Autonomous Exploration

This is the most important finding for this phase. The exploration loop (L2325-2415) **does not call the LLM for navigation decisions**. The control flow is:

```
exploration_loop() → choose_frontier() → navigate_to()  [NO LLM involved]
                   → _random_exploration_walk()          [NO LLM involved]
```

The LLM is only called in two contexts:
1. **Voice loop** (L2691-2708): User speaks → `think()` → GPT-4o → `execute()` — this is user-initiated
2. **Observation background thread** (L2316-2323): Timer fires → `observe()` → GPT-4o vision → `speak()` — this is **observe-only**, no actions

Neither of these feeds back into the exploration loop's navigation decisions. The exploration thread is purely algorithmic (frontier detection + Nav2 goals + reactive fallback).

### 3.3 The "Loops Describing" Symptom Explained

The user reports the robot "loops and describes what it sees but does not navigate." Here's exactly why:

1. **Observation timer** fires every 15s (`self.look_interval = 15.0`, L359)
2. `_observe_in_background()` calls `observe()` which sends the camera image to GPT-4o with prompt: *"What do you see? Note anything useful for mapping."* (L1910)
3. GPT-4o returns a description (max 100 tokens, L1915)
4. `speak()` is called to narrate the observation via TTS (L2321)
5. **Meanwhile**, the exploration thread is either:
   - Waiting for Nav2 to start (up to 60s of silence)
   - In reactive mode, reversing a few cm and stopping
   - Waiting for frontiers that don't exist

The user hears continuous descriptions every 15s because the observation loop runs on a timer, but the exploration thread is stuck or barely moving. **The LLM is not looping on actions — it's looping on observations.**

### 3.4 LLM System Prompt Analysis (L838-945)

The system prompt (`_build_system_prompt()`) defines 14 JSON actions including `"explore"`. Key observations:

- **Action 9**: `{"action": "explore", "speech": "Beginning exploration!"}` — triggers `start_exploration()`, which is a one-shot call that spawns the explore thread
- The prompt does NOT instruct the LLM to issue sequential move commands for exploration
- The prompt does NOT give the LLM feedback about exploration progress
- There is no "continue exploring" or "explore next area" action — the LLM fires one `"explore"` action and the explore thread takes over

**This is correct design** — the LLM shouldn't be in the movement loop. But it means the LLM has no ability to recover when the explore thread fails silently.

### 3.5 The `think()` Function — Context During Exploration (L2146-2177)

```python
def think(self, user_input):
    context = self.get_context()  # Includes: mapping status, exploring flag, position, LiDAR distances
    messages = [
        {"role": "system", "content": self.system_prompt + "\n\n" + context}
    ]
    for msg in self.conversation[-4:]:  # Last 4 conversation entries
        messages.append(msg)
    messages.append({"role": "user", "content": user_input})
    response = self.client.chat.completions.create(
        model="gpt-4o", messages=messages, max_tokens=150, temperature=0.7
    )
```

The context tells the LLM `"Exploring: yes"` or `"Exploring: no"` (L2135). But it does **not** tell the LLM:
- Whether the exploration is making progress
- Whether Nav2 started successfully
- Whether the robot is stuck
- What the frontier detection is finding (or not finding)

If the user asks "are you exploring?" during a stuck exploration, the LLM would say "yes" even though no movement is happening.

### 3.6 The `execute()` Function — Action Parsing (L2179-2312)

JSON parsing is robust with fallback extraction (L2184-2190). The `"explore"` action handler:

```python
elif action == "explore":
    if speech:
        self.speak(speech)
    if not self.mapping:
        self.speak("Starting mapping first...")
        self.start_slam()
        time.sleep(2)
    self.start_exploration()
```

**Key issue**: If the user says "explore" while `self.exploring` is already `True`, `start_exploration()` returns immediately (guard at L2675). The user gets no feedback that exploration is already running (or stuck).

### 3.7 `speak()` Blocks Audio (L1815-1855)

`speak()` is synchronous and blocking:
1. Calls OpenAI TTS API (`tts-1`, voice `nova`)
2. Writes MP3 to temp file
3. Converts to WAV with ffmpeg (volume boost 3x)
4. Plays with `aplay`

Duration: ~2-5 seconds per utterance. While `self.speaking = True`, the voice loop (L2694) sleeps in a polling loop, and **no new voice input is processed**.

**Impact on exploration**: The observation loop fires `speak()` every 15s. Each TTS call takes ~3-5s. During that window, the voice loop is paused. But the exploration thread is NOT blocked by speaking — it continues running (or failing) independently.

However, in `_random_exploration_walk()` (L2449) and `_reactive_exploration_loop()` (L2594), `self.speak()` IS called inline, which **blocks the exploration thread** for 3-5 seconds:
```python
self.speak("I see an opening, going through!")  # L2449 — blocks exploration!
```

### 3.8 Observation vs. Navigation — Architectural Gap

The observation system and the navigation system are **completely disconnected**:

| System | Thread | Calls LLM? | Issues Movement? | Knows About Other? |
|--------|--------|------------|-----------------|-------------------|
| Exploration loop | `explore_thread` | No | Yes (Nav2/cmd_vel) | Does not read observations |
| Observation timer | Background threads | Yes (GPT-4o vision) | No | Does not know exploration state |
| Voice loop | `voice_thread` | Yes (GPT-4o) | Via `execute()` | Knows `self.exploring` flag only |

Observations like "I see a doorway" are spoken aloud but **never fed to the exploration algorithm**. The frontier detection uses only the occupancy grid map, not camera observations.

### 3.9 Phase 3 Summary — Additional Root Causes

| # | Hypothesis | Evidence | Confidence |
|---|-----------|----------|------------|
| H11 | **LLM is not in the navigation loop** — exploration is purely algorithmic; when the algorithm fails, there's no intelligent fallback | `exploration_loop` never calls `think()`; observation output is not used for navigation | ⭐⭐⭐ (design issue, not a bug) |
| H12 | **No exploration progress feedback to user** — LLM context only has `"Exploring: yes/no"`, not progress/stuck state | `get_context()` L2128-2144 has no frontier count, nav2 status, or movement metrics | ⭐⭐⭐ |
| H13 | **`speak()` blocks exploration thread** when called inline from `_random_exploration_walk()` and `_reactive_exploration_loop()` | L2449, L2594 call `self.speak()` synchronously, blocking for 3-5s | ⭐⭐ |
| H14 | **Observation narration every 15s creates illusion of activity** while exploration is stuck | Timer at L2349 + GPT-4o vision at L1903 runs regardless of movement | ⭐⭐⭐⭐ (explains user symptom) |

### Files Examined
- `scripts/voice_mapper.py`: L838-945 (system prompt), L1815-1855 (speak/TTS), L1885-1934 (observe/vision), L2128-2177 (get_context + think), L2179-2312 (execute + action parsing), L2316-2323 (_observe_in_background), L2691-2708 (voice_loop), L2712-2748 (sensor monitor)
- `scripts/llm_robot_brain.py`: Full file L1-618 (confirmed standalone/unused)

## Phase 4: Safety Systems & Obstacle Handling

**Status**: ✅ Complete

### 4.1 Safety Layer Stack — Four Independent Systems

The robot has **four overlapping safety systems**, each with different thresholds and behaviors:

| Layer | Where | Threshold | Action | Active During Exploration? |
|-------|-------|-----------|--------|---------------------------|
| 1. Emergency stop | `scan_callback` L1075 | `front_wide < 0.3m` | Sets `emergency_stop_triggered` flag | **NO** — disabled when `self.exploring` (L1076) |
| 2. voice_mapper obstacle avoidance | `move()`, reactive loops | `min_front < 0.5m` | Stops forward / reverses | **YES** — primary obstacle system |
| 3. Nav2 costmap + controller | costmap inflation + RPP controller | `inflation_radius: 0.35m` from obstacle edge | Replans, slows, or aborts goal | **YES** (when Nav2 is running) |
| 4. Nav2 collision monitor | `collision_monitor` in nav2_params | `time_before_collision: 1.2s` | Modifies cmd_vel | **YES** (when Nav2 is running) |

**Critical: Layer 1 (emergency stop) is intentionally disabled during exploration.** The comment at L1073-1074 says: *"Emergency stop check — only for manual movement, not during exploration (exploration has its own obstacle handling via Nav2 and reactive fallback)"*. This is correct in principle, but it means exploration relies entirely on Layer 2 (the wide-cone obstacle avoidance that triggers too aggressively).

### 4.2 The 135° Detection Cone Problem — Quantified

The reactive exploration functions compute `min_front` as:
```python
min_front = min(front, front_wide, fl, fr)
```

Where (with 720-point LiDAR at 0.5°/point):
- `front` = 0° to +45° (90 points, 10th percentile)
- `front_right` = -45° to 0° (90 points, 10th percentile)
- `front_left` = +45° to **+90°** (90 points, 10th percentile)
- `front_wide` = -30° to +30° (120 points, 5th percentile)

**The combined `min_front` covers -45° to +90° = 135° total.** The `front_left` sector extends all the way to 90° (directly left of the robot). An object at 80° — almost perpendicular to the robot's heading — triggers `min_front < 0.5m`, causing backward movement.

In contrast, the emergency stop correctly uses only `front_wide` (±30° = 60° narrow cone). This shows the developers understood the problem for emergency stops but didn't apply the same logic to exploration obstacle avoidance.

### 4.3 Asymmetric Sector Coverage

The sector definitions have an asymmetry:

```
front_left:  +45° to +90°  (included in min_front)
front_right: -45° to  0°   (included in min_front)
→ Left side covered to 90°, right side only to 45°
```

This means an object at 80° on the **left** triggers avoidance, but the same object at 80° on the **right** (280° = in the `right` sector) does **not**. The robot is more likely to reverse when objects are on its left side.

### 4.4 10th Percentile Distance — Conservative But Reasonable

Each sector's obstacle distance uses `np.percentile(sector_ranges, 10)` (L1038). This means the distance reported is the value where 10% of readings are closer. This is:
- Good for filtering noise (dust, sensor artifacts)
- But conservative — a single wall segment at the edge of the sector will pull the percentile down

For a sector with 90 points covering 45°, a wall spanning just 9 points (4.5°) at close range would make the 10th percentile reflect that wall's distance rather than the open space in the rest of the sector.

### 4.5 Nav2 Costmap — Triple Safety

When Nav2 is running, the robot's safety stack becomes:
1. **Obstacle layer** marks cells occupied based on LiDAR + depth camera
2. **Inflation layer** inflates obstacles by 0.35m (`inflation_radius: 0.35`)
3. **Collision monitor** checks `time_before_collision: 1.2s` before forwarding cmd_vel

The robot footprint is 30cm × 20cm (`[[0.15, 0.10], ...]`). With 0.35m inflation, the effective clearance is 0.35m from any obstacle edge. In a narrow hallway or doorway (standard door = 0.8m wide), the inflated obstacles leave only:

```
0.8m (door) - 0.20m (robot width) - 2 × 0.35m (inflation) = -0.1m
```

**The inflation radius makes standard doorways appear IMPASSABLE to Nav2.** The `inflation_radius: 0.35` combined with the 0.20m robot width leaves negative clearance through a 0.8m doorway. Nav2 cannot plan a path through doorways.

**Wait** — let me reconsider. The inflation radius is from the obstacle cell edge, not from the robot center. The planner checks if the robot footprint overlaps with inflated cells. So the actual clearance check is:
- Door opening = 0.8m
- Robot width = 0.20m, so needs 0.3m on each side minimum (robot center to wall)
- Inflation extends 0.35m from wall → cost is elevated but not necessarily lethal cost at 0.35m

The `cost_scaling_factor: 2.5` determines how quickly cost drops with distance. At factor 2.5, the lethal cost (253) drops to near-zero within about 0.35m. The robot CAN plan through doorways, but the controller may slow significantly or abort if the path gets too close to inflated zones.

### 4.6 Collision Monitor — Additional cmd_vel Filtering

```yaml
collision_monitor:
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    FootprintApproach:
        time_before_collision: 1.2
        simulation_time_step: 0.1
        min_points: 6
```

The collision monitor simulates the robot's trajectory 1.2 seconds forward. If collision is predicted, it scales down or zeroes cmd_vel. At 0.15 m/s, 1.2s = 0.18m lookahead. This could cause the robot to stop well before reaching an obstacle.

Combined with the voice_mapper's own 0.5m threshold, the robot may never get close enough to a doorway to pass through it — Nav2 slows it down, the collision monitor restricts velocity, and the reactive fallback reverses.

### 4.7 Self-Body Reflection Filtering

```python
min_range = max(msg.range_min, 0.12) if msg.range_min > 0 else 0.12
ranges_clean = np.where(
    np.isinf(ranges) | np.isnan(ranges) | (ranges < min_range),
    max_range, ranges)
```

Readings below 0.12m are filtered as self-body reflections (L1005-1008). This is good — the LiDAR can see the robot's own chassis. But if `range_min` from the sensor is > 0.12m, that sensor-reported minimum is used instead. If the sensor's `range_min` is misconfigured (e.g., 0.3m), legitimate close-range readings would be filtered out as body reflections.

### 4.8 Emergency Stop During Exploration — Disabled But Logged

```python
if self.min_front_distance < self.emergency_dist:  # < 0.3m
    if not self.exploring:
        self.emergency_stop_triggered = True        # Flag for move()
    elif ...:
        self.get_logger().warning(...)              # Log only
```

During exploration, the emergency stop flag is NOT set. The `move()` function checks `self.emergency_stop_triggered` (L2030), but since the flag is never set during exploration, `move()` won't emergency-stop either.

**However**, `move()` is only called from `_reactive_exploration_loop()` → `choose_exploration_direction()` → `self.move(linear, angular, 2.0)` at L2655. The reactive step functions publish to cmd_vel directly, bypassing `move()` entirely. So the `move()` safety checks (emergency stop, obstacle distance, speed scaling) are **only active in the full reactive loop, not the step-based reactive fallback**.

### 4.9 Phase 4 Summary — Additional Root Causes

| # | Hypothesis | Evidence | Confidence |
|---|-----------|----------|------------|
| H15 | **135° detection cone** — `min_front` includes `front_left` (+45° to +90°) making side objects trigger backward movement | Sector geometry confirmed: `front_left` covers 45-90°; `min_front = min(front, front_wide, fl, fr)` | ⭐⭐⭐⭐⭐ (confirmed) |
| H16 | **Nav2 inflation radius** may make doorways appear too costly for path planning | `inflation_radius: 0.35m`, door = 0.8m, robot = 0.2m wide; tight but passable with `cost_scaling_factor: 2.5` | ⭐⭐ |
| H17 | **Multiple safety layers compound** — voice_mapper (0.5m), Nav2 costmap (0.35m inflation), collision monitor (1.2s lookahead) all restrict movement independently | Each layer documented above; combined effect is overly conservative | ⭐⭐⭐ |
| H18 | **`_reactive_exploration_step()` bypasses `move()` safety** — publishes raw cmd_vel without the speed scaling and Ackerman constraints in `move()` | L2574 publishes directly; `move()` at L2022 has richer obstacle handling | ⭐⭐ |

### Files Examined
- `scripts/voice_mapper.py`: L993-1082 (scan_callback full analysis), L1083-1136 (doorway detection), L1938-1944 (emergency_stop), L1946-2020 (_execute_uturn), L2022-2084 (move with safety), L2085-2124 (choose_exploration_direction), L2552-2574 (_reactive_exploration_step), L2576-2661 (_reactive_exploration_loop)
- `scripts/nav2_params.yaml`: L157-205 (local costmap), L207-255 (global costmap), L312-334 (behavior server), L362-397 (collision monitor)

## Phase 5: Synthesis & Root Cause Analysis

**Status**: ✅ Complete

### 5.1 Complete Data Flow — Explore Mode

```
User says "explore"
    │
    ▼
voice_loop() → listen() → transcribe() → think() [GPT-4o]
    │                                        │
    │                              Returns: {"action": "explore", "speech": "..."}
    │                                        │
    ▼                                        ▼
execute()                              speak(speech)  [3-5s TTS block]
    │
    ├── if not mapping: start_slam() + sleep(2)
    │
    ▼
start_exploration()
    │
    ├── self.exploring = True
    └── Thread(exploration_loop).start()
         │
         ▼
    ┌─ Nav2 available? ──────────────────────────────────────┐
    │  NO                                              YES   │
    │  ▼                                                ▼    │
    │  start_nav2()                              MAIN LOOP   │
    │  ├─ SUCCESS → MAIN LOOP ──────────────────────►│       │
    │  └─ FAIL → _try_nav2_with_reactive_fallback()  │       │
    │       ├─ Nav2 up in 30s → exploration_loop() ──┤       │
    │       └─ Nav2 never → _reactive_exploration_loop()     │
    │            │                                    │      │
    │            ▼                                    │      │
    │    [STUCK HERE: reverse/stop loop]              │      │
    │    Every iteration:                             │      │
    │      min_front = min(F,FW,FL,FR) [135° cone]    │      │
    │      if < 0.5m → reverse at -0.05 m/s           │      │
    │      else → forward at 0.1 m/s                  │      │
    │                                                 │      │
    ├─────────────────────────────────────────────────┘      │
    │                                                         │
    │  MAIN LOOP (Nav2 mode):                                │
    │    while exploring:                                     │
    │      Every 15s: _observe_in_background() [GPT-4o→TTS]  │
    │      if not navigating:                                 │
    │        frontier = choose_frontier()                     │
    │        if frontier: navigate_to(fx, fy)  ──► Nav2 goal │
    │        if None x3: _random_exploration_walk()           │
    │      sleep(0.5)                                        │
    │      if stuck 30s: cancel + clear costmaps             │
    └────────────────────────────────────────────────────────┘

CONCURRENT: _observe_in_background() every 15s
    observe() → GPT-4o vision → speak() [description only, NO movement]
```

### 5.2 Ranked Root Causes — Explaining Each Symptom

#### Symptom 1: "Robot doesn't move"

| Rank | Root Cause | Mechanism | Fix Difficulty |
|------|-----------|-----------|---------------|
| 1 | **Nav2 fails to start** → reactive fallback barely moves (H1) | `start_nav2()` times out (60s), bt_navigator never reaches `active`; robot enters reactive mode which reverses on any obstacle within 135° × 0.5m | Medium |
| 2 | **No SLAM map → no frontiers** (H8) | `find_frontiers()` returns `[]` without a map; exploration loops on `choose_frontier() → None` with 2s sleeps | Low |
| 3 | **135° obstacle cone triggers on side objects** (H2, H15) | `min_front = min(front, front_wide, front_left, front_right)` includes the +45° to +90° sector; walls at 80° cause avoidance | Low |
| 4 | **`speak()` blocks exploration thread** (H13) | Inline `speak()` calls in reactive loop pause movement 3-5s per TTS call | Low |

#### Symptom 2: "Moves backwards only, a few cm"

| Rank | Root Cause | Mechanism | Fix Difficulty |
|------|-----------|-----------|---------------|
| 1 | **Ackerman reverse-to-turn in reactive mode** (H3) | `twist.linear.x = -0.05` used for turning because "Ackerman can't turn in place"; any obstacle < 0.5m in the 135° cone triggers this | Low |
| 2 | **Nav2 BackUp recovery behavior** (H9) | When Nav2 is running and gets stuck, the behavior tree triggers `BackUp` plugin which drives backward | Low (config) |
| 3 | **Overly conservative 0.5m threshold** on wide cone (H5, H17) | `min_obstacle_dist = 0.5m` applied to 135° arc means typical indoor furniture/walls constantly trigger reverse | Low |

#### Symptom 3: "Loops describing what it sees"

| Rank | Root Cause | Mechanism | Fix Difficulty |
|------|-----------|-----------|---------------|
| 1 | **Observation timer fires regardless of movement** (H4, H14) | `_observe_in_background()` every 15s → GPT-4o vision → TTS; decoupled from exploration progress | Low |
| 2 | **No progress feedback to user/LLM** (H12) | `get_context()` only reports `"Exploring: yes"`, not whether it's stuck or making progress | Low |

### 5.3 The Primary Failure Cascade

The most likely end-to-end failure sequence:

```
1. User says "explore"
2. SLAM starts (or was already running)
3. exploration_loop() checks: Nav2 available? → NO
4. start_nav2() launches Nav2 stack
5. bt_navigator takes too long to reach 'active' (common with lifecycle conflicts)
6. 60s timeout → start_nav2() returns False
7. _try_nav2_with_reactive_fallback() runs for 30s:
   - _reactive_exploration_step() fires every 0.1s
   - In typical indoor room: walls/furniture within 0.5m of 135° cone
   - Robot reverses 0.05 m/s per step → moves backwards a few cm
8. 30s expires, Nav2 still not ready
9. Falls through to _reactive_exploration_loop():
   - Same 0.5m / 135° cone issue
   - Reverse-to-turn on every obstacle detection
   - speak() calls block movement 3-5s each time
10. Meanwhile, every 15s: GPT-4o describes surroundings and speaks
11. User hears descriptions, sees minimal/backward movement
12. User concludes: "it loops describing but doesn't navigate"
```

### 5.4 Fix Recommendations — Prioritized

#### Priority 1: Fix the 135° detection cone (HIGH IMPACT, LOW EFFORT)

**Problem**: `min_front` includes `front_left` (45°-90°) which is the robot's side, not its front.

**Fix**: Use a narrower cone for exploration obstacle avoidance — match the emergency stop's ±30° `front_wide` sector, or use only `front` + `front_right` (±45°) without `front_left`.

```python
# BEFORE (voice_mapper.py, multiple locations):
min_front = min(front, front_wide, fl, fr)

# AFTER — use only the actual forward-facing sectors:
min_front = min(front, front_wide, fr)  # ±45° max, not +90°
# Or even narrower for exploration:
min_front = front_wide  # ±30° like emergency stop
```

**Files**: `_reactive_exploration_step()` L2558, `_reactive_exploration_loop()` L2600-2604, `choose_exploration_direction()` L2095

#### Priority 2: Improve reactive exploration — forward arc turns instead of reverse (HIGH IMPACT, LOW EFFORT)

**Problem**: Ackerman reverse-to-turn always moves backwards. But the robot CAN turn with forward motion — it's already done in `_random_exploration_walk()` (L2431: `twist.linear.x = 0.08` with angular).

**Fix**: Replace `twist.linear.x = -0.05` with `twist.linear.x = 0.08` (minimum forward speed for Ackerman turning), the same pattern used in `_random_exploration_walk()`.

```python
# BEFORE (_reactive_exploration_step, L2567):
twist.linear.x = -0.05
twist.angular.z = 0.4 if left > right else -0.4

# AFTER — forward arc turn (same pattern as _random_exploration_walk L2431):
twist.linear.x = 0.08  # Minimum forward speed for Ackerman steering
twist.angular.z = 0.4 if left > right else -0.4
```

**Files**: `_reactive_exploration_step()` L2567, `_reactive_exploration_loop()` L2628

#### Priority 3: Reduce min_obstacle_dist for exploration (MEDIUM IMPACT, LOW EFFORT)

**Problem**: 0.5m is too conservative for the 135° cone (or even a narrower cone in indoor environments).

**Fix**: Use a separate, smaller threshold for exploration (e.g., 0.3m), distinct from the manual movement threshold.

```python
self.min_obstacle_dist = 0.5       # For manual/voice commands
self.explore_obstacle_dist = 0.3   # For autonomous exploration
```

#### Priority 4: Make reactive mode the robust default (MEDIUM IMPACT, MEDIUM EFFORT)

**Problem**: If Nav2 fails, the reactive fallback is a poor experience. But `_random_exploration_walk()` actually works well — it drives forward, follows gaps, and finds open space.

**Fix**: Restructure exploration to use `_random_exploration_walk()` as the primary exploration strategy, with Nav2 as an enhancement rather than a requirement. The random walk already has doorway detection and open-space navigation.

#### Priority 5: Don't block exploration with speak() (LOW IMPACT, LOW EFFORT)

**Problem**: Inline `speak()` calls in `_random_exploration_walk()` and `_reactive_exploration_loop()` pause movement.

**Fix**: Run speech in a background thread from exploration code:
```python
# BEFORE:
self.speak("I see an opening, going through!")

# AFTER:
threading.Thread(target=self.speak, args=("I see an opening, going through!",), daemon=True).start()
```

#### Priority 6: Add exploration progress to LLM context (LOW IMPACT, LOW EFFORT)

**Fix**: Enhance `get_context()` to include Nav2 status, frontier count, and movement progress so the LLM can report stuck state to the user.

### 5.5 Open Questions — Resolved

| Question | Answer | Phase |
|----------|--------|-------|
| Is Nav2 actually running and healthy? | Likely NO — bt_navigator lifecycle must reach `active`, which requires SLAM map + time; 60s timeout is often insufficient | Phase 2 |
| Is VSLAM providing localization? | VSLAM is independent of Nav2 exploration; Nav2 uses AMCL (with wrong motion model) for localization, not VSLAM | Phase 2/4 |
| Is the LLM called in a tight loop? | NO — LLM is only called for voice input and 15s observation timer; exploration is purely algorithmic | Phase 3 |

---

## Overview

The robot's explore mode fails due to a **cascade of compounding issues**, not a single bug. The primary failure path is:

1. **Nav2 fails to initialize** within 60s (common due to lifecycle timing), dropping the robot into reactive mode
2. **Reactive mode's 135° obstacle detection cone** (including the +45° to +90° left sector) triggers obstacle avoidance on side objects that pose no forward-movement risk
3. **Ackerman reverse-to-turn** strategy means every obstacle detection moves the robot backwards instead of finding a forward path
4. **The observation timer** (every 15s) keeps narrating what the robot sees, independent of movement, creating the impression of a loop that "describes but doesn't navigate"

The fix is straightforward: narrow the obstacle detection cone, use forward arc turns instead of reverse, and make the reactive exploration more aggressive about moving forward. These are all low-effort changes in `voice_mapper.py` that don't require architectural changes.

## Key Findings

### Phase 1 Findings
1. **Nav2 failure = reactive fallback** — If Nav2 doesn't reach `active` within 60s, exploration degrades to reactive mode which has severe movement limitations
2. **Reactive mode goes backwards** — Both `_reactive_exploration_step()` and `_reactive_exploration_loop()` use `linear.x = -0.05` (reverse) for obstacle avoidance, based on Ackerman steering constraint
3. **Wide obstacle detection cone** — `min_front` combines 4 LiDAR sectors spanning ±45° plus a ±30° emergency zone; anything within 0.5m in this wide arc triggers backward movement
4. **Observation loop is decoupled from movement** — GPT-4o vision calls happen every 15s on background threads regardless of whether the robot is moving, creating the "describes but doesn't navigate" impression
5. **Recursive `exploration_loop()` call** from `_try_nav2_with_reactive_fallback()` (L2539) is a potential stack issue

### Phase 2 Findings
6. **`navigate_to()` is fire-and-forget** — returns `True` immediately; async goal rejection not propagated to exploration loop, so it keeps trying new frontiers without knowing why they fail
7. **AMCL motion model mismatch** — uses `DifferentialMotionModel` for an Ackerman robot, causing localization drift and Nav2 goal failures
8. **No SLAM map = no frontiers** — `find_frontiers()` returns `[]` if `latest_map is None`; entire exploration falls to `_random_exploration_walk()` fallback
9. **cmd_vel conflict** — both voice_mapper and Nav2 (via collision_monitor) publish to `/cmd_vel`; reactive fallback code can override Nav2 commands
10. **Nav2 backup recovery** — behavior server includes `BackUp` plugin; when Nav2 recovery triggers, the robot physically backs up, contributing to "backwards movement"
11. **Costmap depends on `/oak/points`** — if OAK-D point cloud is unavailable, costmap has incomplete obstacle data

### Phase 3 Findings
12. **LLM is NOT in the exploration loop** — exploration is purely algorithmic (frontier + Nav2 + reactive); observations are spoken but never fed back to navigation
13. **`llm_robot_brain.py` is standalone/unused** — `voice_mapper.py` has its own self-contained GPT-4o integration
14. **"Loops describing" is the observation timer** — `_observe_in_background()` fires every 15s, calls GPT-4o vision, speaks result regardless of whether robot is moving
15. **No exploration progress in LLM context** — `get_context()` only has `"Exploring: yes/no"`, not stuck state, frontier counts, or Nav2 health
16. **`speak()` blocks inline callers** — `_random_exploration_walk()` and `_reactive_exploration_loop()` call `speak()` synchronously, pausing movement for 3-5s per TTS call

### Phase 4 Findings
17. **135° detection cone confirmed** — `min_front` includes `front_left` (45-90°), so a wall/object at 80° (nearly perpendicular) triggers obstacle avoidance and backward movement
18. **Asymmetric coverage** — left side detected to 90°, right side only to 45°; robot more likely to reverse when objects on left
19. **Four overlapping safety layers** — emergency stop (disabled during exploration), voice_mapper 0.5m threshold, Nav2 costmap inflation (0.35m), collision monitor (1.2s lookahead); combined effect is overly conservative
20. **Nav2 inflation + doorway geometry** — `inflation_radius: 0.35m` makes doorways expensive (but not impossible) for the planner; combined with collision monitor, robot likely slows or aborts near doorways
21. **Reactive step bypasses `move()` safety** — `_reactive_exploration_step()` publishes raw cmd_vel without the speed scaling, Ackerman constraints, or proximity-based slowdown in `move()`

## Actionable Conclusions

### Must Fix (directly cause reported symptoms)
1. **Narrow the obstacle detection cone** — change `min_front` in reactive exploration from `min(front, front_wide, fl, fr)` (135°) to `front_wide` only (60°) or `min(front, front_wide, fr)` (90°). Files: L2558, L2600-2604, L2095.
2. **Replace reverse-to-turn with forward arc turn** — change `twist.linear.x = -0.05` to `twist.linear.x = 0.08` in reactive exploration, matching the pattern already used in `_random_exploration_walk()`. Files: L2567, L2628.
3. **Lower obstacle threshold for exploration** — add `self.explore_obstacle_dist = 0.3` and use it in exploration code instead of `self.min_obstacle_dist = 0.5`. File: L350, plus reactive functions.

### Should Fix (improve reliability)
4. **Make `_random_exploration_walk()` the primary reactive strategy** — it already works well (drives forward, detects gaps, finds open space); currently only runs after 3 failed frontier attempts.
5. **Run `speak()` in background threads from exploration code** — prevent TTS from blocking movement. Files: L2449, L2594.
6. **Add exploration progress to `get_context()`** — include Nav2 status, frontier count, stuck state so the LLM can inform the user. File: L2128-2144.

### Nice to Have (long-term quality)
7. **Propagate Nav2 goal failure** from async callbacks to exploration loop — add a `self.last_nav_failure_reason` field.
8. **Fix AMCL motion model** — while `DifferentialMotionModel` is the "least bad" option, the localization impact should be monitored.
9. **Eliminate recursive `exploration_loop()` call** from `_try_nav2_with_reactive_fallback()` — restructure as a loop.

## Open Questions (Resolved)

| Question | Answer | Phase |
|----------|--------|-------|
| Is Nav2 actually running and healthy? | Likely NO — bt_navigator lifecycle timeout is the primary failure point | 2 |
| Is VSLAM providing localization? | VSLAM is separate from Nav2; Nav2 uses AMCL with incorrect motion model | 2/4 |
| Is the LLM called in a tight loop? | NO — LLM is only called for voice and 15s observation; exploration is algorithmic | 3 |

## Standards Applied
- Source priority: codebase first, then config files, then external docs
- All findings include file paths and line numbers
- Uncertainties flagged explicitly
- Root causes ranked by likelihood and cross-referenced across phases

## Handoff

This research is complete. The three reported symptoms (no movement, backwards movement, description loop) all trace to identifiable code patterns with clear fixes.

**Recommended next step**: `/pch-planner` to create an implementation plan addressing Priority 1-3 fixes (narrow cone, forward arc turns, lower threshold) as a single focused change to `scripts/voice_mapper.py`. These three changes alone should resolve the primary symptoms.
