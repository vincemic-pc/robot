---
id: "007"
type: research
title: "LLM-Driven Autonomous Exploration: Real-Time Navigation Decisions"
status: ✅ Complete
created: "2026-02-25"
current_phase: "6 of 6 (All phases complete)"
---

## Introduction

This research investigates how to give an LLM (such as Claude or GPT-4o) direct
decision-making authority over the robot's exploration behavior — choosing direction,
speed, and exploration strategy based on camera imagery, LiDAR scans, depth data,
IMU orientation, and odometry. Rather than following pre-programmed frontier-based
exploration, the LLM would act as the "cognitive layer" that perceives the environment,
reasons about where to go, and issues motor commands.

### Current State

The robot (ROSMASTER A1, Jetson Orin Nano) already has:
- **GPT-4o as brain** in `voice_mapper.py` — processes voice commands + camera images
- **Action vocabulary**: `move`, `look`, `explore`, `get_dist`, `stop`, etc.
- **Nav2 frontier exploration** — autonomous but algorithm-driven (no LLM reasoning)
- **OAK-D Pro** — RGB 1080p + stereo depth + IMU
- **SLLidar C1** — 2D LiDAR, 720 points @ 10Hz
- **Odometry** — wheel encoder + optional Isaac VSLAM

The gap: the LLM currently receives occasional snapshots and responds to voice commands.
It does NOT continuously perceive the environment and make real-time navigation decisions.
This research explores how to close that gap.

## Objectives

1. **Architecture**: Design a perception → reasoning → action loop where the LLM
   decides direction and speed at each decision point
2. **Sensor Fusion for LLM**: Determine what sensor data to present to the LLM
   and in what format (images, text summaries, structured data)
3. **Prompt Engineering**: Design the system prompt, tool definitions, and
   structured output format for navigation decisions
4. **Safety Layer**: How to maintain obstacle avoidance guarantees when the LLM
   controls movement (LLM is slow; obstacles are fast)
5. **Latency & Loop Timing**: Analyze the decision loop timing constraints and
   strategies (LLM API call latency vs. real-time requirements)
6. **Model Selection**: Evaluate Claude vs. GPT-4o vs. local models for this task
7. **Implementation Path**: Identify concrete steps to build this on the existing
   `voice_mapper.py` architecture

## Research Phases

| Phase | Name | Status | Scope | Session |
|-------|------|--------|-------|---------|
| 1 | Existing Architecture & Decision Points | ✅ Complete | How the current LLM brain works, action loop, timing | 2026-02-25 |
| 2 | Sensor Data Representation for LLMs | ✅ Complete | What to feed the LLM: image formats, LiDAR summaries, depth maps, structured state | 2026-02-25 |
| 3 | LLM Navigation Prompt & Tool Design | ✅ Complete | System prompt, tool/function calling schema, structured output for motor commands | 2026-02-25 |
| 4 | Safety Architecture & Reactive Layer | ✅ Complete | Guaranteeing obstacle avoidance when LLM controls movement, fallback behaviors | 2026-02-25 |
| 5 | Latency, Loop Timing & Streaming Strategies | ✅ Complete | API call timing, streaming responses, local vs. cloud models, decision frequency | 2026-02-25 |
| 6 | Prior Art, Frameworks & Implementation Path | ✅ Complete | Existing projects (SayCan, VoxPoser, RT-2, LM-Nav), frameworks, concrete build plan | 2026-02-25 |

---

## Phase 1: Existing Architecture & Decision Points

**Scope**: Deep-dive into `voice_mapper.py` to understand:
- How the current GPT-4o brain receives sensor data and makes decisions
- The current action loop timing (how often does the LLM get called?)
- What sensor data is currently sent to the LLM (images? text summaries?)
- How exploration mode works today (frontier-based vs. LLM-directed)
- The gap between current "LLM as command interpreter" and desired "LLM as navigator"
- Threading model and how LLM calls interact with ROS2 spin

**Key Questions**:
- How often is GPT-4o called during exploration today?
- What context window is sent with each call?
- What actions can the LLM currently issue?
- Where would the "continuous perception loop" be inserted?

**Status**: ✅ Complete

### Findings

#### 1.1 Two LLM Brain Implementations Exist

The codebase contains **two separate LLM brain files**:

| File | Lines | Status | LLM Provider | Used By |
|------|-------|--------|-------------|---------|
| `scripts/voice_mapper.py` | ~2900 | **Active** — main application | Hardcoded GPT-4o via `openai` SDK | `run_voice_mapper.sh` → systemd service |
| `scripts/llm_robot_brain.py` | ~618 | **Standalone** — not used in production | Multi-provider (OpenAI, Anthropic, Ollama, Google) | Not launched by start_robot.sh |

**`llm_robot_brain.py`** (lines 1–618) is architecturally cleaner for LLM-switching:
- Uses a `LLMProvider` base class with `OpenAIProvider`, `AnthropicProvider`, `OllamaProvider`
- Topic-based: listens on `/voice_text`, `/text_input`, publishes to `/robot_action`, `/speech_output`
- Has its own `RobotActionLibrary` with 25+ defined actions
- JSON-based structured output: `{"thinking": "...", "actions": [...], "response": "..."}`
- Max tokens: 1000 per call
- **However**: not integrated with Nav2, frontier exploration, SLAM, or any of the advanced features in `voice_mapper.py`

**`voice_mapper.py`** (lines 1–~2900) is the production system and the focus of this research.

#### 1.2 GPT-4o Call Sites in voice_mapper.py

There are **3 distinct GPT-4o call sites**, each with different purposes:

**Call Site 1: Object Localization** — `find_object_distance()` (line 1281)
- **Purpose**: Locate a named object in camera image, return pixel coordinates for depth query
- **System prompt**: "You are a robot vision system. When asked to find an object, respond with JSON..."
- **Input**: RGB image (640×480, JPEG quality 80, base64, `detail: "low"`) + target name
- **Output**: JSON → `{found, x, y, description}`
- **Max tokens**: 100 | **Temperature**: 0.3
- **Conversation history**: None — standalone call
- **Trigger**: User says something like "how far is the door?"

**Call Site 2: Environment Description** — `observe()` (line 1908)
- **Purpose**: Describe what the camera sees for mapping/exploration logging
- **System prompt**: "You are a mapping robot. Describe what you see concisely, noting landmarks, obstacles, open spaces, doors..."
- **Input**: RGB image (640×480, JPEG quality 80, base64, `detail: "low"`) + "What do you see?"
- **Output**: Free-text observation (spoken aloud + logged to `DiscoveryLog`)
- **Max tokens**: 100 | **Temperature**: 0.7
- **Conversation history**: None — standalone call
- **Trigger**: `look` voice command OR every 15s during exploration (`_observe_in_background`)

**Call Site 3: Main Brain / Command Processor** — `think()` (line 2173)
- **Purpose**: Process user voice commands and decide what action to take
- **System prompt**: ~100 lines defining capabilities, JSON action format, personality (~950 tokens)
- **Context injected**: Dynamic `get_context()` string with:
  - Mapping status (active/not started)
  - Exploring status (yes/no)
  - Current position (x, y)
  - Distance traveled
  - Discovery count
  - LiDAR distances: front, left, right (3 floats)
  - Map size (width × height cells)
  - Nav2 status, explore mode, frontier count, stuck status (when exploring)
- **Input**: system_prompt + context + last 4 conversation messages + user text
- **Output**: JSON action → `{"action": "move", "linear": 0.15, "angular": 0.0, "duration": 2.0, "speech": "..."}`
- **Max tokens**: 150 | **Temperature**: 0.7
- **Conversation history**: Rolling window of last 4 messages (trimmed to 6 total)
- **CRITICAL**: No camera image is sent with the brain call! The brain reasons from text context only.
- **Trigger**: Every voice command from the user

#### 1.3 Action Vocabulary

The `execute()` method (line 2192) dispatches on the `action` field from `think()`:

| Action | Parameters | Implementation | LLM Decides? |
|--------|-----------|----------------|-------------|
| `move` | linear (-0.2..0.2), angular (-0.5..0.5), duration (max 15s) | Direct cmd_vel with obstacle avoidance | Yes — LLM chooses speed/direction |
| `turn_around` | — | 3-point U-turn (forward arc, reverse arc, forward arc) | Yes — LLM triggers it |
| `look` | — | Calls `observe()` → GPT-4o vision → speak result | Yes |
| `get_dist` | target name | `find_object_distance()` → GPT-4o locates → depth query | Yes |
| `explore` | — | Starts `exploration_loop()` (algorithmic, NOT LLM-driven) | Yes to start, No for decisions |
| `navigate` | x, y | Nav2 `NavigateToPose` action goal | Yes — LLM picks coordinates |
| `start_mapping` | — | Launches `slam_toolbox` subprocess | Yes |
| `stop_mapping` | map_name | Saves map, kills SLAM process | Yes |
| `start_vslam` | — | Launches Isaac VSLAM | Yes |
| `stop` | — | Stops exploration + cancels navigation | Yes |
| `status` | — | Reports mapping/exploration progress | Yes |
| `speak` | speech text | TTS only | Yes |

**Key insight**: The LLM can issue `move` commands with specific velocities, but during `explore` mode, all navigation decisions are **purely algorithmic**. The LLM is sidelined to a passive observer role.

#### 1.4 Exploration Architecture

Exploration runs in 3 modes with cascading fallback:

```
User says "explore"
  └─→ exploration_loop() [dedicated thread]
       ├── Mode: "nav2-frontier" (primary)
       │   ├── find_frontiers() — numpy vectorized occupancy grid analysis
       │   ├── choose_frontier() — score by size × distance, weighted random selection
       │   ├── navigate_to(fx, fy) — Nav2 NavigateToPose action
       │   ├── Stuck detection: 30s timeout → cancel + clear costmaps
       │   └── Failure escalation: 5 consecutive failures → random walk
       │
       ├── Mode: "random-walk" (fallback when no frontiers)
       │   ├── PRIORITY: Check detected_gaps (doorway detection from LiDAR)
       │   │   └── Turn toward gap → drive through at 0.15 m/s
       │   └── FALLBACK: Find direction with most open space → drive forward
       │
       └── Mode: "reactive" (fallback when Nav2 unavailable)
           ├── Doorway priority: detected_gaps with width ≥ 0.7m
           ├── Obstacle avoidance: narrow ±30° cone check
           ├── Stuck escalation: 3 attempts → smart walk
           └── choose_exploration_direction() — proportional steering
```

**LLM involvement during exploration**: Only through `_observe_in_background()`:
- Fires every **15 seconds** (`look_interval = 15.0`) — line 2364
- Spawns a sub-thread so it doesn't block navigation
- Calls `observe()` → GPT-4o vision → speaks observation aloud → logs to DiscoveryLog
- **The observation result is NEVER fed back into navigation decisions**
- The LLM literally describes what it sees and then the description is discarded (from a nav perspective)

**Frontier detection** (`find_frontiers()`, line 645):
- Reads the SLAM occupancy grid (`/map` topic)
- Finds cells that are free (value=0) adjacent to unknown (value=-1)
- Clusters frontier cells using grid-cell grouping + flood-fill (O(n))
- Returns clusters sorted by size (larger = likely doorways)
- `choose_frontier()` scores by: `size_score × distance_score`, prefers 2-5m range

**Doorway detection** (`_detect_doorways()`, line 1088):
- Analyzes LiDAR front 180° arc for sudden distance jumps (>1m)
- Identifies gaps ≥ 0.6m wide as potential doorways
- Stores in `self.detected_gaps` for use by random walk and reactive modes

#### 1.5 Threading Model

```
┌─────────────────────────────────────────────────────────────────┐
│ Main Thread: rclpy.spin_once(timeout=0.1s)  [10 Hz]            │
│   ROS2 callbacks execute here:                                  │
│   - camera_callback → self.latest_image                         │
│   - depth_callback → self.latest_depth                          │
│   - scan_callback → self.latest_scan + obstacle_distances       │
│   - odom_callback → self.latest_odom + current_position         │
│   - map_callback → self.latest_map + map_info                   │
│   - Nav2 feedback/result callbacks (via action client)          │
├─────────────────────────────────────────────────────────────────┤
│ Voice Thread: voice_loop()  [blocks on listen()]                │
│   listen() → transcribe(Whisper) → think(GPT-4o) → execute()   │
│   - Synchronous pipeline: one command at a time                 │
│   - Paused while speaking (self.speaking flag)                  │
│   - GPT-4o call is blocking (~1-5s)                            │
├─────────────────────────────────────────────────────────────────┤
│ Exploration Thread: exploration_loop()  [when exploring]        │
│   Nav2 frontier loop with 0.5s poll interval                    │
│   - choose_frontier() reads self.latest_map (written by main)   │
│   - navigate_to() sends Nav2 goal (async callback on main)      │
│   - _observe_in_background() → sub-thread every 15s             │
│     └── observe(GPT-4o vision) → speak()                       │
├─────────────────────────────────────────────────────────────────┤
│ Sensor Monitor Thread: _sensor_monitor_loop()  [every 2s]       │
│   Checks for LiDAR/odom/camera loss → stops exploration if lost │
└─────────────────────────────────────────────────────────────────┘
```

**Concurrency risks for LLM navigation**:
- `self.latest_image`, `self.latest_scan`, `self.obstacle_distances` are **written by main thread** and **read by exploration thread** without locks
- This works today because reads/writes are atomic at the Python level (GIL) and exploration only reads
- A continuous LLM decision loop would need to coordinate with the voice thread (both want to call GPT-4o)
- The OpenAI client is thread-safe for concurrent calls, but conversation history is not protected

#### 1.6 Sensor Data Currently Sent to LLM

| Sensor | Sent to LLM? | Format | When | Recipient |
|--------|-------------|--------|------|-----------|
| RGB Camera | **Yes** | Base64 JPEG, 640×480, quality 80, `detail: "low"` | `observe()`, `find_object_distance()` | GPT-4o vision calls |
| Depth Camera | **No** — used for pixel queries only | Raw uint16 values at (x,y) coords | On-demand via `get_dist` action | Local processing only |
| LiDAR (720 pts) | **Indirectly** | 3 floats in text: "front=X.Xm, left=X.Xm, right=X.Xm" | Every `think()` call via `get_context()` | Brain text prompt |
| Odometry | **Indirectly** | Position text: "(X.X, Y.Y)" | Every `think()` call via `get_context()` | Brain text prompt |
| IMU | **No** | Not used anywhere in current code | Never | — |
| SLAM Map | **Indirectly** | Text: "Map size WxH cells" | Every `think()` call via `get_context()` | Brain text prompt |
| Doorway gaps | **No** | Internal list of angle/width/distance dicts | Never sent to LLM | Reactive exploration only |

**Image processing pipeline** (`image_to_base64()`, line 1876):
1. ROS Image → OpenCV BGR via `cv_bridge`
2. Optional brightness enhancement (`convertScaleAbs(alpha=15.0, beta=30)` — only for cameras that need it)
3. Resize to 640×480
4. JPEG encode at quality 80
5. Base64 encode → ~50-80KB per image → ~70-110K base64 characters

**Token cost per image**: With `detail: "low"`, OpenAI charges 85 tokens per image. At `detail: "high"`, a 640×480 image would cost ~765 tokens.

#### 1.7 The Gap: LLM as Command Interpreter vs. LLM as Navigator

| Dimension | Current State ("Command Interpreter") | Desired State ("Cognitive Navigator") |
|-----------|---------------------------------------|---------------------------------------|
| **When LLM is called** | On voice command + 15s observation timer | Continuously at decision frequency (0.2-1 Hz) |
| **What LLM receives** | Text context + optional image (separate calls) | Fused sensor summary: camera + LiDAR + depth + pose |
| **What LLM decides** | Which action to execute for user's request | Where to go, how fast, what strategy to use |
| **Navigation authority** | None during exploration (algorithmic frontier) | Full — LLM replaces frontier selection |
| **Camera usage in brain** | Never (brain gets text only) | Every decision cycle (visual reasoning) |
| **LiDAR usage** | 3 floats in text prompt | Structured sector summary or visual representation |
| **Depth usage** | On-demand pixel queries only | Integrated into spatial awareness |
| **Memory** | 4-message conversation window | Exploration history, discovered rooms, spatial map |
| **Observation → Action link** | `observe()` result is spoken, not acted on | Observations directly inform next movement |

#### 1.8 Insertion Points for Continuous Perception Loop

The **exploration_loop** (line 2338) is the natural insertion point:

```python
# CURRENT (algorithmic):
while exploring:
    if not navigating:
        frontier = choose_frontier()        # ← Algorithm decides
        navigate_to(frontier)               # ← Nav2 executes
    if time_for_observation:
        _observe_in_background()            # ← LLM describes (output discarded)
    sleep(0.5)

# PROPOSED (LLM-driven):
while exploring:
    sensor_summary = prepare_sensor_summary()     # ← Fuse camera+LiDAR+depth+pose
    decision = llm_decide_navigation(summary)     # ← LLM reasons about where to go
    validated = safety_layer.validate(decision)    # ← Reactive layer checks/modifies
    execute_navigation_decision(validated)         # ← Move or set Nav2 goal
    update_exploration_memory(decision, outcome)   # ← Track what was tried
    sleep(decision_interval)                       # ← 1-5 seconds
```

**Alternative insertion**: The `_observe_in_background()` call (line 2366) already runs every 15s and has access to the camera image. It could be extended to return a navigation decision instead of just an observation string. This would be the minimal-change approach.

#### 1.9 Key Findings Summary

1. **The LLM brain (`think()`) never sees camera images** — it reasons from text context only, yet the robot has a perfectly good camera that's already being sent to GPT-4o in other calls
2. **Exploration observations are wasted** — `observe()` produces rich environment descriptions every 15s, but they're spoken aloud and never fed back into navigation
3. **Three separate GPT-4o calls exist** with no coordination — object finding, observation, and brain each have independent system prompts and no shared context
4. **The frontier algorithm is competent but blind** — it finds map-based frontiers efficiently but cannot reason about what it sees (e.g., "that doorway looks interesting" or "this room seems like a kitchen")
5. **`llm_robot_brain.py` has the right provider abstraction** but isn't used — its multi-provider support (Anthropic, Ollama) would be needed for model switching
6. **The threading model is compatible** with adding an LLM decision loop — a new thread or modification of the exploration thread would work without architectural changes
7. **Safety infrastructure exists** — emergency stop, obstacle distances, speed limiting in `move()`, sensor monitoring — all separable from navigation logic
8. **Image size is small** — 640×480 JPEG at quality 80 with `detail: "low"` is only 85 tokens per OpenAI call, making frequent image submissions feasible

---

## Phase 2: Sensor Data Representation for LLMs

**Scope**: Research how to represent each sensor modality for LLM consumption:

- **Camera RGB**: Raw image vs. annotated image (with grid overlay, compass, distance markers)
- **Depth Map**: Heatmap image vs. text summary ("obstacle 0.5m ahead at 2 o'clock")
- **LiDAR Scan**: Polar plot image vs. sector summary text vs. ASCII top-down map
- **Odometry/Pose**: Text summary of position, heading, velocity
- **IMU**: Pitch/roll for slope detection
- **Battery**: Remaining capacity for return-to-base planning
- **Map/SLAM**: Occupancy grid image vs. text description of explored areas

**Key Questions**:
- What's the optimal balance between visual and textual sensor representation?
- How much data can we send per decision cycle without blowing token budgets?
- Should we pre-process sensor data into a "situation report" before sending to LLM?
- What image resolution is sufficient for navigation decisions (downsample from 1080p)?

**Status**: ✅ Complete

### Findings

#### 2.1 Current Sensor Data Inventory

The robot has 7 sensor sources. Here is the complete inventory with current LLM usage:

| Sensor | ROS2 Topic | Format | Rate | Currently Sent to LLM? | Tokens Used |
|--------|-----------|--------|------|----------------------|-------------|
| RGB Camera | `/oak/rgb/image_raw` | 640×480 JPEG q80, base64, `detail: "low"` | On-demand | **Yes** — `observe()` and `find_object_distance()` only | 85 (OpenAI) |
| Depth Camera | `/oak/stereo/image_raw` | uint16 mm, 5×5 median queries | On-demand | **No** — pixel queries only via `get_dist()` | 0 |
| LiDAR (720 pts) | `/scan` | 6 sectors + doorway gaps | 10 Hz | **Indirectly** — 3 floats in `get_context()` | ~15 |
| Wheel Odometry | `/odom` | x, y, theta | 50+ Hz | **Indirectly** — position text in `get_context()` | ~10 |
| VSLAM Odometry | `/visual_slam/tracking/odometry` | 6-DOF pose, 200+ Hz | When active | **No** | 0 |
| Occupancy Map | `/map` | OccupancyGrid (width×height cells) | On SLAM update | **Indirectly** — "Map size WxH cells" in `get_context()` | ~8 |
| IMU | `/oak/imu/data` | Configured but **never subscribed** | N/A | **No** | 0 |

**Key gap**: The brain (`think()`, line 2173) never receives camera images — it reasons purely from text. The camera is sent only to separate `observe()` and `find_object_distance()` calls, whose results are discarded by navigation logic.

#### 2.2 Camera RGB Representation

##### Current State
- `image_to_base64()` (line 1876): CvBridge → resize 640×480 → JPEG q80 → base64
- Staleness check: rejects frames >5 seconds old
- Enhancement: `cv2.convertScaleAbs(alpha=15.0, beta=30)` available but disabled for OAK-D Pro
- Sent with OpenAI `detail: "low"` = **85 tokens flat** (internally rescaled to 512×512)

##### Resolution Analysis

| Resolution | OpenAI `detail:low` | OpenAI `detail:high` | Claude (w×h/750) | Navigation Quality |
|-----------|---------------------|---------------------|------------------|-------------------|
| 320×240 | 85 tokens | 255 tokens (1 tile) | ~102 tokens | Adequate for coarse direction |
| 640×480 | 85 tokens | 765 tokens (4 tiles) | ~410 tokens | **Sweet spot** — doorways, furniture, floor |
| 1280×720 | 85 tokens | ~1105 tokens (6 tiles) | ~1228 tokens | Overkill for navigation |

**Recommendation**: Keep 640×480 at `detail: "low"` for routine navigation. This is already near-optimal. The trade-off: `detail: "low"` loses some spatial resolution that helps with depth-from-perspective cues, but the LiDAR text summary compensates.

##### Image Annotation Strategies (Pre-Processing Before Sending to VLM)

Research from VLN (Vision-Language Navigation) projects (2024–2025) consistently shows that **annotated images outperform raw images** for navigation decisions. Annotation cost is negligible (2–5ms of OpenCV drawing).

**a) Depth Value Overlays** (Highest value, zero extra token cost)
- Sample depth at 5–7 key points using existing `get_dist()` function
- Overlay distance text on the RGB image before encoding:
  ```
  Points: center(320,240), left(160,240), right(480,240),
          floor-ahead(320,400), upper-left(160,120), upper-right(480,120)
  Rendered as: "2.3m" in white text with black outline
  ```
- Gives the VLM both visual scene AND distance information in a single 85-token image
- **This is the single highest-impact improvement** — currently depth is unused by navigation

**b) Compass / Heading Indicator** (High value)
- Draw current heading as arrow + text in corner: "Facing: NE 47°"
- Helps VLM reason about cardinal directions ("turn left to face north")
- ~1ms to render

**c) Sector Labels** (Medium value)
- Divide image into 3–5 vertical sectors labeled "left", "center", "right"
- Creates shared vocabulary between prompt and visual data
- Enables prompts like "which sector has the most open space?"

**d) LiDAR Distance Arcs** (Medium value, requires calibration)
- Project LiDAR ranges as colored arcs on ground plane (red=1m, yellow=2m, green=3m+)
- Requires LiDAR-to-camera extrinsic calibration (OAK-D + SLLidar C1 have offset mounts)
- Most information-dense representation but highest implementation effort

**e) Bounding Boxes / Object Labels** (Low priority for navigation)
- Run lightweight detector (YOLO/MobileSSD) first, annotate image
- Useful for object-goal navigation ("go to the chair") but overkill for exploration
- Adds inference latency on Jetson (20–50ms for MobileSSD)

##### Recommended Camera Strategy

**Tier 1 (implement first)**: Annotated camera image with depth overlays + heading indicator
- Base: 640×480 RGB, JPEG q80
- Overlay: depth at 5–7 points, heading arrow/text
- Cost: 85 tokens (OpenAI `detail:low`) or ~410 tokens (Claude)
- Processing: <5ms additional on Jetson Orin Nano

**Tier 2 (consider later)**: Sector labels + LiDAR arcs
- Adds spatial grounding but requires calibration work

#### 2.3 LiDAR Data Representation

##### Current State
The 720-point LiDAR scan is processed in `scan_callback()` (line 998) into:
- **6 sectors**: front, front_left, front_right, left, right, back — with 10th percentile distances
- **Emergency zone**: ±30° front arc, 5th percentile
- **Doorway detection**: `_detect_doorways()` finds gaps ≥0.6m wide in front 180°
- **LLM receives**: Only 3 floats — `"front=X.Xm, left=X.Xm, right=X.Xm"` (~15 tokens)

This is a massive information loss: 720 data points → 3 numbers. Doorway detections, blocked sectors, and environmental shape are all discarded before reaching the LLM.

##### Representation Options Analyzed

| Approach | Token Cost | Info Density | VLM Accuracy | Best For |
|----------|-----------|-------------|-------------|---------|
| **3-float summary** (current) | ~15 | Very low | Poor spatial reasoning | Status display only |
| **12-sector text + gaps** | ~100–120 | High | Good — VLMs reason well about structured text | **Recommended default** |
| **Polar plot image** | 85 (detail:low) | Medium | Medium — VLMs struggle with precise distances from plots | Gestalt room shape |
| **ASCII top-down map** | ~400–600 | Medium | Surprisingly good for topology | Room-level reasoning |
| **BEV occupancy image** | 85 (detail:low) | High | Medium | "Where haven't I explored?" |

##### Recommended: Enhanced 12-Sector Text Summary

Expand from current 3-float summary to structured 12-sector report with qualitative labels:

```
LIDAR (12 sectors, 30° each, clockwise from front):
  000° front:       2.9m CLEAR
  030° front-right: 3.2m CLEAR
  060° right-front: 1.8m CLEAR
  090° right:       0.5m WALL
  120° right-back:  0.7m WALL
  150° back-right:  2.1m CLEAR
  180° back:        4.5m CLEAR
  210° back-left:   3.8m CLEAR
  240° left-back:   2.3m CLEAR
  270° left:        1.2m OBSTACLE
  300° left-front:  2.8m CLEAR
  330° front-left:  3.5m CLEAR
Nearest: 0.5m at 90° (right)
Gaps: doorway at 315°, width 0.9m, dist 2.1m
```

**Token cost**: ~100–120 tokens (text only, no image needed)
**Implementation**: Extend existing `scan_callback()` sector analysis from 6 → 12 sectors
**Qualitative labels**: CLEAR (>2m), NEAR (<2m), OBSTACLE (<1m), WALL (<0.5m)

##### Optional: Polar Plot Image as Second Image

For complex environments, a polar plot adds gestalt spatial understanding:
- Render with OpenCV (5–10ms) not matplotlib (20–50ms)
- Robot at center, obstacles as colored dots, gaps highlighted
- Scale rings at 1m, 2m, 3m
- Send at `detail: "low"` = 85 additional tokens
- **Use case**: When the LLM needs to understand room shape, not just obstacle distances

#### 2.4 Depth Camera Representation

##### Current State
- Depth (`/oak/stereo/image_raw`): uint16 millimeter values
- `get_dist(x, y)`: 5×5 region median query with 0.5s cache
- Valid range: 0.1m–10.0m
- **Never sent to LLM** — used only for on-demand pixel queries via `find_object_distance()`

##### Options Analyzed

| Approach | Extra Tokens | Info Value | Recommendation |
|----------|-------------|-----------|----------------|
| **Depth annotations on RGB** | 0 | **High** | **Best** — overlay distance text on camera image |
| Text region summary | ~30–50 | Medium | Good alternative if annotation is too complex |
| Jet colormap heatmap image | 85 | Medium | VLM sees "close=red, far=blue" but imprecise |
| Raw grayscale depth | 85 | Low | VLM can't interpret without scale reference |
| Don't send depth | 0 | N/A | Acceptable if LiDAR covers ground plane |

##### Recommended: Depth-Annotated RGB (Zero Extra Tokens)

The optimal approach is to NOT send a separate depth image. Instead, sample depth at key points and overlay as text on the RGB image before encoding:

```
Sample points (on 640×480 image):
  (320, 240) = center         → "2.3m"
  (160, 240) = left-center    → "1.1m"
  (480, 240) = right-center   → "4.2m"
  (320, 400) = floor-ahead    → "0.8m"
  (160, 120) = upper-left     → "3.5m"
  (480, 120) = upper-right    → "2.9m"
  (320, 100) = upper-center   → "5.1m" (corridor depth)
```

Rendering: white text with black outline, ~14px font, using OpenCV `putText()`.
Processing time: <2ms (7 calls to existing `get_dist()` with warm cache).

**When to add a full depth heatmap image** (85 extra tokens):
- Complex 3D geometry: stairs, ramps, overhanging obstacles
- Transparent obstacles: glass walls, thin poles invisible in RGB
- In these cases, jet-colormap at `detail: "low"` is worth the extra 85 tokens

#### 2.5 Odometry / Pose Representation

##### Current State
- `get_context()` sends: `"Position: (X.X, Y.Y)"` — no heading, no velocity
- Heading (theta) is tracked in `self.current_position['theta']` but not included
- VSLAM provides 6-DOF pose + 3D path history (up to 10,000 poses) but is never sent to LLM

##### Recommended: Enhanced State Block (~80 tokens)

```
ROBOT STATE:
  Position: (2.3, -1.5) meters from start
  Heading: 47° (NE) | facing northeast
  Speed: 0.15 m/s forward
  Distance traveled: 23.4m
  Time exploring: 4m 32s
  VSLAM: tracking (quality: good)
  Battery: 72% (~45min remaining)
```

**New data to include**:
- **Heading in degrees + cardinal**: Convert theta radians to degrees + compass direction — helps LLM reason about turns ("turn 90° left to face north")
- **Current velocity**: From odometry twist, helps LLM understand if robot is moving or stopped
- **VSLAM status**: tracking/lost/relocating — critical for knowing if position is reliable
- **Exploration time**: Helps LLM pace exploration and plan return

**Implementation**: Extend `get_context()` (line 2132) with additional fields from `self.current_position['theta']`, `self.latest_odom.twist`, and `self.vslam_tracking`.

#### 2.6 Map / SLAM Representation

##### Current State
- `get_context()` sends: `"Map size: WxH cells"` — no coverage, no frontier info
- `map_callback()` computes `self.map_coverage` (fraction of known cells) but doesn't send it
- Frontier count is sent during exploration: `"Frontiers found: N"`

##### Options

| Approach | Tokens | When Useful |
|----------|--------|------------|
| **Enhanced text** | ~40 | Always — coverage %, area in m², room count estimate |
| **Minimap image** (occupancy grid rendered) | 85 | Strategic decisions — "where haven't I been?" |
| **Frontier overlay on minimap** | 85 | Choosing which area to explore next |

##### Recommended: Enhanced Text + Optional Minimap

**Always send** (text, ~40 tokens):
```
MAP STATUS:
  Area mapped: 8.2m × 6.4m (52.5 m²)
  Coverage: 34% explored
  Frontiers: 3 open (largest at 315°, 2.1m away)
  Rooms discovered: ~2 (based on boundary analysis)
```

**Optionally send** (every 5th decision cycle, 85 tokens):
- Render occupancy grid as small image with robot position + heading arrow + frontier markers
- Color scheme: white=free, black=wall, gray=unknown, green dot=robot, red dots=frontiers
- Send at `detail: "low"` as second image
- **Most useful** when LLM is choosing between exploration directions

#### 2.7 IMU Representation

##### Current State
- IMU topic configured (`/oak/imu/data`) but **never subscribed** — no callback exists
- OAK-D Pro has a BNO086 IMU providing accelerometer + gyroscope + magnetometer

##### Recommendation: Subscribe and Include Pitch/Roll

For navigation safety, pitch and roll from IMU would detect:
- **Slopes/ramps**: pitch > 5° indicates incline
- **Uneven terrain**: roll > 3° indicates tilt
- **Stairs approach**: rapid pitch change = stair edge

Include in state block as: `"Tilt: pitch 2° (level), roll 1° (level)"` (~10 tokens)

**Priority**: Low — the robot operates on flat indoor floors. Implement only if multi-floor or ramp navigation is needed.

#### 2.8 Token Budget Per Decision Cycle

Using verified pricing from official API documentation (February 2026):

##### Image Token Costs by Provider

| Provider | Image Cost Formula | 640×480 Image Cost |
|----------|-------------------|-------------------|
| **OpenAI** (`detail:low`) | Fixed 85 tokens | **85 tokens** |
| **OpenAI** (`detail:high`) | 85 + (tiles × 170) | **765 tokens** (4 tiles) |
| **Anthropic Claude** | (width × height) / 750 | **~410 tokens** |
| **Google Gemini** | ~258 tokens per image | **~258 tokens** |

##### Complete Decision Cycle Token Budget

| Component | Tokens | Notes |
|-----------|--------|-------|
| System prompt (nav rules, safety, action schema) | 400–600 | One-time per conversation turn |
| State context (position, heading, velocity, map) | 80–120 | Enhanced `get_context()` |
| LiDAR text (12 sectors + gaps + qualitative) | 100–120 | Enhanced sector summary |
| Camera image | 85–410 | 85 (OpenAI low) / 410 (Claude) |
| Decision history (last 2 decisions + outcomes) | 150–300 | Rolling window |
| **Total input** | **815–1550** | Varies by provider |
| Output (JSON action + brief reasoning) | 80–150 | `{"action":"move","linear":0.15,...}` |
| **Total per cycle** | **895–1700** | |

##### Per-Call and Hourly Costs (Verified Feb 2026 Pricing)

**API Pricing Sources** (verified via official docs):

| Model | Input $/1M | Output $/1M | Source |
|-------|-----------|-------------|--------|
| GPT-4o | $2.50 | $10.00 | [OpenAI Pricing](https://openai.com/api/pricing/) |
| GPT-4o-mini | $0.15 | $0.60 | [OpenAI Pricing](https://openai.com/api/pricing/) |
| Claude Sonnet 4.5 | $3.00 | $15.00 | [Anthropic Pricing](https://platform.claude.com/docs/en/about-claude/pricing) |
| Claude Haiku 4.5 | $1.00 | $5.00 | [Anthropic Pricing](https://platform.claude.com/docs/en/about-claude/pricing) |
| Gemini 2.0 Flash | $0.10 | $0.40 | [Google AI Pricing](https://ai.google.dev/gemini-api/docs/pricing) |

**Cost per decision call** (estimated with ~1000 input tokens + image + 100 output tokens):

| Model | Image Tokens | Total Input | Cost/Call | Rank |
|-------|-------------|-------------|-----------|------|
| **Gemini 2.0 Flash** | ~258 | ~1,158 | **$0.000156** | 1st (cheapest) |
| **GPT-4o-mini** (detail:low) | 85 | ~985 | **$0.000208** | 2nd |
| **Claude Haiku 4.5** | ~410 | ~1,310 | **$0.001810** | 3rd |
| **GPT-4o** (detail:low) | 85 | ~985 | **$0.003463** | 4th |
| **Claude Sonnet 4.5** | ~410 | ~1,310 | **$0.005430** | 5th |

**Hourly costs by decision frequency**:

| Model | 0.2 Hz (720/hr) | 0.5 Hz (1,800/hr) | 1.0 Hz (3,600/hr) |
|-------|-----------------|-------------------|-------------------|
| Gemini 2.0 Flash | **$0.11** | $0.28 | $0.56 |
| GPT-4o-mini | **$0.15** | $0.37 | $0.75 |
| Claude Haiku 4.5 | **$1.30** | $3.26 | $6.52 |
| GPT-4o | **$2.49** | $6.23 | $12.47 |
| Claude Sonnet 4.5 | **$3.91** | $9.77 | $19.55 |

**Cost per 8-hour exploration day**:

| Model | 0.2 Hz | 0.5 Hz |
|-------|--------|--------|
| Gemini 2.0 Flash | **$0.90** | $2.24 |
| GPT-4o-mini | **$1.20** | $2.99 |
| Claude Haiku 4.5 | **$10.43** | $26.06 |
| GPT-4o | **$19.95** | $49.87 |
| Claude Sonnet 4.5 | **$31.27** | $78.17 |

##### Prompt Caching Impact

Both OpenAI and Anthropic offer cached input pricing:
- **OpenAI**: Cached input at 50% off ($1.25/1M for GPT-4o)
- **Anthropic**: Cache read at 10% of base price ($0.10/1M for Haiku 4.5, $0.30/1M for Sonnet 4.5)

The system prompt (~500 tokens) is identical every call and should be cached. With caching, the effective cost drops ~10–20% for OpenAI models and ~15–25% for Claude models since the system prompt is a significant fraction of total input.

#### 2.9 Latency Budget per Decision Cycle

| Phase | Duration | Notes |
|-------|---------|-------|
| Image capture + OpenCV resize + JPEG encode | 5–15ms | On Orin Nano |
| Depth sampling (7 points with cache) | 1–3ms | Warm cache from `depth_callback` |
| OpenCV annotation (depth text + heading) | 2–5ms | `putText()` calls |
| Base64 encoding | 1–3ms | ~80KB JPEG → ~110KB base64 |
| LiDAR sector summary generation | 1–2ms | NumPy vectorized |
| Network upload to API | 10–50ms | WiFi, ~150KB payload |
| **API inference** | **300–4,000ms** | **Dominant cost** |
| JSON parse + validation | 1–2ms | |
| ROS2 cmd_vel publish | <1ms | |
| **Total** | **320–4,080ms** | |

**API latency by model** (estimated median, vision calls):

| Model | Median Latency | Max Practical Frequency |
|-------|---------------|------------------------|
| Claude Haiku 4.5 | ~500–800ms | ~1.5 Hz |
| GPT-4o-mini | ~600–900ms | ~1.2 Hz |
| Claude Sonnet 4.5 | ~1,000–2,000ms | ~0.6 Hz |
| GPT-4o | ~1,500–3,000ms | ~0.4 Hz |

**Conclusion**: **0.2–0.5 Hz is the practical range** for cloud VLMs. The robot continues executing the last decision between VLM calls, with the LiDAR-based safety layer running at 10 Hz.

#### 2.10 Local VLM Feasibility on Jetson Orin Nano 8GB

Constraints: 8GB unified RAM shared between CPU, GPU, ROS2, SLAM, Nav2, and OAK-D driver. Estimated ~3–4GB available for VLM after other processes.

| Model | Parameters | VRAM | Speed (tok/s) | Nav Quality | Feasible? |
|-------|-----------|------|---------------|-------------|----------|
| **Moondream2** | 1.86B | ~1.5GB | 10–18 | Good for basic directions | **Yes** — best candidate |
| **Florence-2-base** | 0.23B | ~1GB | 15–30 | Detection/captioning only | **Yes** — good pre-filter |
| **SmolVLM** | ~1B | ~1GB | 12–20 | Basic visual Q&A | **Yes** — new option |
| **Phi-3.5-Vision** (4-bit) | 3.8B | ~3–4GB | 5–12 | Best reasoning | **Tight** — may compete with SLAM |
| LLaVA-7B (4-bit) | 7B | ~4–5GB | 3–6 | Good | **No** — too large alongside SLAM |

**Best local strategy**: **Moondream2 or Florence-2** as a continuous pre-filter (1–2 Hz) that produces structured text descriptions, which are then included in the cloud VLM prompt instead of (or alongside) the camera image. This reduces cloud VLM cognitive load and provides continuous awareness even during API call latency gaps.

#### 2.11 Composite / Fused Sensor Image Strategy

Instead of sending multiple images (camera + LiDAR plot + depth map), composite into fewer images:

##### Option A: Single Annotated Camera Image (85 tokens — Recommended Default)
- Base: 640×480 RGB
- Overlay: depth values at 7 points, heading indicator, sector labels
- Send at `detail: "low"` = 85 tokens (OpenAI) / ~410 tokens (Claude)
- **Best cost/info ratio** — one image contains camera + depth + heading

##### Option B: Camera + LiDAR Polar Plot (170 tokens)
- Image 1: annotated camera (as above)
- Image 2: LiDAR polar plot with scale rings, gap markers, heading arrow
- Both at `detail: "low"` = 170 tokens total
- **Use when**: complex room geometry, multiple doorways, T-intersections

##### Option C: Dashboard Composite (85 tokens, lower quality)
- Single 960×480 image tiling camera (left) + polar plot (right)
- At `detail: "low"`, each sub-panel gets only ~256×256 effective resolution
- **Not recommended**: sub-panels too small for VLM to interpret reliably

##### Anti-Pattern: Too Many Sub-Panels
Tiling 4+ views into one image degrades all panels below VLM readability threshold. At `detail: "low"` (512×512 internal), a 2×2 grid gives each panel only ~256×256 — insufficient for navigation reasoning.

#### 2.12 Prior Art on Sensor Representation for LLM Robotics

Three architectural patterns emerge from surveying major projects (SayCan, PaLM-E, RT-2, LM-Nav, NaVid, VoxPoser, CoW):

| Pattern | Description | Sensor Input | Decision Rate | Cost | Example |
|---------|------------|-------------|---------------|------|---------|
| **LLM as Planner** | Text descriptions → goals/subgoals | Text only | 0.05–0.2 Hz | Low | SayCan, LM-Nav |
| **VLM as Controller** | Camera images → motor commands | Images + text | 0.2–3 Hz | Moderate–High | RT-2, NaVid, GPT-4V demos |
| **LLM as Code Generator** | LLM writes policy code that runs locally | Images for LLM, code runs on sensor data | LLM: 0.01 Hz, code: 10+ Hz | Very low | VoxPoser |

**Best fit for this robot**: **VLM as Controller** with a safety wrapper. The VLM sees annotated camera + LiDAR text at 0.2–0.5 Hz, outputs direction/speed decisions, and the existing LiDAR-based safety layer (10 Hz) handles reactive obstacle avoidance.

**Key gap in open-source projects**: Almost no projects fuse LiDAR + camera for VLM navigation. Most use either vision-only (unreliable distance estimation) or text-only (no visual reasoning). The annotated-image-with-LiDAR-text approach is underexplored and high-potential.

**NaVid (2024) insight**: Sending 2–3 sequential frames (not just current frame) improves navigation decisions by providing temporal/motion context. At 0.2 Hz, include the last 2 annotated images (current + 5 seconds ago) for ~170 tokens total.

#### 2.13 Key Findings Summary

1. **Annotated camera images dramatically outperform raw images** for VLM navigation — overlaying depth values, heading, and sector labels costs <5ms and zero extra tokens
2. **The current 3-float LiDAR summary is severely inadequate** — expanding to 12 sectors with qualitative labels and doorway gaps (+85 tokens) provides 10× more spatial information
3. **Depth-on-RGB is the optimal depth representation** — sampling 7 depth points and overlaying on the camera image gives the VLM distance grounding at zero extra token cost
4. **Gemini 2.0 Flash and GPT-4o-mini are the practical choices** for continuous navigation — $0.11–$0.15/hour at 0.2 Hz, compared to $2.49–$3.91/hour for GPT-4o/Claude Sonnet
5. **0.2–0.5 Hz is the realistic decision frequency** for cloud VLMs — API latency is the bottleneck, not image processing
6. **Claude's image tokenization is 5× more expensive than OpenAI `detail:low`** — 410 vs 85 tokens for 640×480, making Claude models disproportionately expensive for vision-heavy workloads
7. **Moondream2 on Jetson Orin Nano** is feasible (~1.5GB, 10–18 tok/s) as a local continuous pre-filter, reducing cloud VLM dependency
8. **Prompt caching should be used** — system prompt is identical every call, saving 10–25% on input costs
9. **The total token budget per decision is ~900–1,700 tokens** — well within all model context windows and economically viable at 0.2 Hz

---

## Phase 3: LLM Navigation Prompt & Tool Design

**Scope**: Design the complete prompt and tool/function-call interface:

- **System Prompt**: Robot identity, capabilities, navigation philosophy, safety rules
- **Tool Definitions**: What "tools" does the LLM call to act?
  - `set_velocity(linear_x, angular_z, duration_s)` -- direct motor control
  - `navigate_to(x, y, theta)` -- Nav2 goal delegation
  - `rotate(degrees)` -- turn in place
  - `look_direction(direction)` -- orient camera/body
  - `speak(message)` -- announce findings
  - `mark_interest(label, x, y)` -- annotate map
  - `request_depth(pixel_x, pixel_y)` -- query specific depth
- **Structured Output**: JSON schema for decisions
- **Memory/Context**: How to maintain exploration history across calls
- **Reasoning Format**: Chain-of-thought vs. direct action

**Key Questions**:
- Should the LLM output a single action or a short plan (next 3 actions)?
- How to encode spatial reasoning in the prompt (cardinal directions? clock positions? grid cells?)
- What's the right abstraction level -- raw velocities vs. semantic movements ("go through the doorway")?
- How to handle the LLM wanting to "see more" before deciding?

**Status**: ✅ Complete

### Findings: Prior Art Survey — How Leading Projects Prompt VLMs/LLMs for Robot Navigation

This section surveys 10 major research projects (2022-2025) to extract concrete prompt
designs, action output formats, and architectural patterns for LLM-driven robot navigation.
These findings directly inform the prompt and tool design for our ROSMASTER A1 robot.

Sources consulted: arXiv papers, GitHub repositories, project websites, and NeurIPS/RSS/ICML
proceedings. All URLs are cited inline.

---

#### 3.1 NaVid (RSS 2024) -- Video-Based VLM for Vision-and-Language Navigation

**Paper**: "NaVid: Video-based VLM Plans the Next Step for Vision-and-Language Navigation"
([arXiv 2402.15852](https://arxiv.org/abs/2402.15852),
[RSS 2024 proceedings](https://www.roboticsproceedings.org/rss20/p079.pdf),
[project page](https://pku-epic.github.io/NaVid/))

**Architecture**: End-to-end VLM built on Vicuna-7B (LLaMA-based) with EVA-CLIP vision
encoder. Takes raw video frames + natural language instruction as input, outputs next-step
action in linguistic form.

**Prompt/Input Structure**:
- Uses the Vicuna v1.1 conversation format (`USER:` / `ASSISTANT:` turns)
- Special tokens split input modalities:
  - `[HIS]` -- history observation frames (4 instruction-agnostic tokens per frame)
  - `[OBS]` -- current observation frame (64 instruction-agnostic tokens)
  - `[NAV]` -- navigation instruction text
- Video frames are encoded via EVA-CLIP vision encoder into visual embeddings (256 patches
  per frame), then projected into language token space via a cross-modality projector
- The LLM receives concatenated observation tokens + instruction tokens

**Action Output Format**:
- Natural language text, parsed by regex into structured actions
- Action set: `{FORWARD, TURN-LEFT, TURN-RIGHT, STOP}`
- Each action includes quantitative arguments:
  - `FORWARD` -- specific distance (e.g., "Move forward 75 cm")
  - `TURN-LEFT` / `TURN-RIGHT` -- specific degrees (e.g., "Turn left 30 degrees")
- Regular expression parser extracts action type + argument with 100% parse success rate
- **Single-step output** -- one action per inference call

**Chain-of-Thought**: Two training data types:
- **Action planning samples** (500k): Direct action prediction from video context
- **Instruction reasoning samples** (10k): The model reasons about how the instruction
  relates to what it sees before choosing an action

**Spatial Information Encoding**:
- Purely visual -- no explicit spatial coordinates, maps, or depth values in prompt
- Spatial reasoning is learned implicitly from video frame sequences
- History frames provide implicit odometry (the model infers movement from visual change)
- No LiDAR, no occupancy grid, no explicit pose data

**Key Takeaway for Our Design**: NaVid demonstrates that a VLM can learn navigation purely
from video frames + language, without explicit spatial data. However, it requires training
a custom VLM end-to-end (510k navigation samples + 763k web data). For our zero-shot
approach with GPT-4o/Claude, we need explicit spatial data in the prompt since we are not
fine-tuning the model.

---

#### 3.2 RT-2 (Google DeepMind, 2023) -- Robotic Transformer 2

**Paper**: "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control"
([arXiv 2307.15818](https://arxiv.org/abs/2307.15818),
[project page](https://robotics-transformer2.github.io/))

**Architecture**: Takes an existing VLM (PaLI-X 55B or PaLM-E 12B) and co-fine-tunes it
on robot action data alongside web-scale vision-language data. The key innovation is
treating robot actions as just another language that can be tokenized and predicted.

**Prompt/Input Structure**:
- Input: single RGB image + natural language instruction (e.g., "Pick up the orange")
- Standard VLM prompt format -- image + text instruction
- No explicit spatial coordinates, depth, or LiDAR in the prompt

**Action Tokenization Scheme** (the critical innovation):
- 7 action dimensions: end-effector delta position (x, y, z), delta rotation (roll, pitch,
  yaw), and gripper extension
- Each continuous dimension is **discretized into 256 bins**
- Bin boundaries are set at 1st and 99th percentiles of training data distributions
  (not min-max, to mitigate outlier effects)
- Each bin maps to an integer token
- **Action string format**: `"1 128 91 241 5 101 127"` -- 7 space-separated integers
  - Token 0: terminate flag (1 = continue, 0 = terminate episode)
  - Tokens 1-3: delta position (x, y, z)
  - Tokens 4-6: delta rotation (roll, pitch, yaw)
  - Token 7: gripper extension
- For **PaLI-X**: numeric tokens up to 1000 are naturally part of the vocabulary, so
  action integers are directly represented
- For **PaLM-E**: 256 least-frequent tokens are overwritten with action vocabulary
  (symbol tuning)

**Output Format**:
- Autoregressive token generation -- the VLM generates action tokens one at a time,
  exactly like generating text
- Tokens are decoded back to continuous action values by mapping bin index to value
- **Single-step output** -- one 7-DoF action per inference call (runs at ~1-3 Hz)

**Chain-of-Thought**: Not used. Direct action token prediction from image + instruction.

**Spatial Information Encoding**: Entirely implicit through the visual input. No explicit
coordinates or spatial annotations.

**Key Takeaway for Our Design**: RT-2's action tokenization pattern is brilliant but
requires fine-tuning (we cannot fine-tune GPT-4o). However, the concept of mapping actions
to a discrete vocabulary is directly applicable -- we can define our action space as a
discrete set of named tools/functions rather than continuous velocities, and let the LLM
select from them. The 256-bin discretization also suggests that for our robot, we should
avoid asking the LLM for precise continuous values (e.g., "move at 0.147 m/s") and instead
provide discrete choices (e.g., "slow/medium/fast" or "15/30/45/90 degree turn").

---

#### 3.3 SayCan (Google, 2022) -- Grounding Language in Robotic Affordances

**Paper**: "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances"
([arXiv 2204.01691](https://arxiv.org/abs/2204.01691),
[project page](https://say-can.github.io/),
[code](https://github.com/google-research/google-research/blob/master/saycan/SayCan-Robot-Pick-Place.ipynb))

**Architecture**: LLM (PaLM 540B) proposes candidate actions; an affordance model
(learned value function) scores feasibility. The combined score selects the action.
No vision in the LLM prompt -- visual grounding comes from the value function.

**Prompt Structure**:
- **Dialog format** between human and robot:
  ```
  Human: How would you bring me a coke can?
  Robot: I would: 1. Find a coke can, 2. Pick up the coke can,
         3. Bring it to you, 4. Done.
  ```
- **17 few-shot examples** precede the actual user request (chain-of-thought prompting)
- Each example shows a task decomposition into a sequence of primitive skills
- The LLM is prompted to generate each step one at a time, and at each step, all possible
  skills are scored

**Action Format**:
- Pre-defined set of **skill primitives** with natural language descriptions
  - Examples: "find a coke can", "pick up the coke can", "go to the table",
    "put down the coke can", "done"
- Each skill has: a language description, a trained policy, and a value function
- The LLM never generates free-form actions -- it scores/selects from the fixed skill set

**Scoring Formula**:
```
score(skill_i) = P_LLM(skill_i | instruction, history) * V(skill_i | current_state)
```
Where:
- `P_LLM(skill_i | instruction, history)` = LLM's log-likelihood that this skill
  helps complete the instruction (text-only, from the language model's probability
  distribution over next tokens)
- `V(skill_i | current_state)` = value function output representing probability of
  successfully executing the skill from the current robot/world state (visual affordance)
- The highest-scoring skill is selected and executed
- After execution, the completed skill is appended to the history and the process repeats

**Chain-of-Thought**: Yes -- the 17 few-shot examples explicitly demonstrate step-by-step
task decomposition. PaLM-SayCan additionally uses chain-of-thought to interpret instructions
before scoring.

**Multi-step**: The overall plan is multi-step (iterative skill selection), but each LLM
call produces a **single next skill** selection.

**Spatial Information Encoding**: None in the LLM prompt. All spatial/visual grounding
is in the affordance model (value function), not in the language model. The LLM reasons
purely about task semantics.

**Key Takeaway for Our Design**: SayCan's architecture of "LLM proposes, affordance model
disposes" is highly relevant. For our robot, we could: (1) have the LLM score candidate
navigation actions, (2) multiply by a LiDAR-based feasibility score (is the path clear?),
and (3) select the winner. This would let the LLM reason about *what* to explore while
the safety layer ensures *how* is safe. The fixed skill set concept also maps well to our
tool/function calling approach.

---

#### 3.4 LM-Nav (CoRL 2023) -- Robotic Navigation with Pre-Trained Models

**Paper**: "LM-Nav: Robotic Navigation with Large Pre-Trained Models of Language, Vision,
and Action"
([arXiv 2207.04429](https://arxiv.org/abs/2207.04429),
[CoRL proceedings](https://proceedings.mlr.press/v205/shah23b.html),
[project page](https://sites.google.com/view/lmnav))

**Architecture**: Three pre-trained models, zero fine-tuning:
1. **GPT-3** -- parses instructions into landmark sequences
2. **CLIP** -- grounds landmarks in visual observations (image-text similarity)
3. **ViNG** -- pre-trained visual navigation model that navigates between images

**GPT-3 Prompt Structure**:
- **3 few-shot examples** for in-context learning, followed by the instruction to parse
- Example format:
  ```
  Instruction: "Look for a library, after taking a right turn next to a statue."
  Landmarks: 1. a statue  2. a library

  Instruction: "Look for a statue. Then look for a library. Then go towards a pink house."
  Landmarks: 1. a statue  2. a library  3. a pink house

  Instruction: [complex reordering example]
  Landmarks: [correctly reordered sequence]
  ```
- A third example handles temporal reordering (e.g., "look for X after passing Y")
- The LLM output is a numbered list: `1. [landmark]  2. [landmark]  ...`

**Action Output Format**:
- GPT-3 does NOT output motor commands or navigation actions
- GPT-3 outputs **landmark names** only (text strings)
- CLIP matches landmarks to pre-collected images in a topological graph
- ViNG (visual navigation model) handles actual motor control to reach each landmark
- **The LLM is a high-level planner only, not a low-level controller**

**Chain-of-Thought**: Not explicitly. The few-shot examples serve as implicit reasoning
templates for instruction parsing.

**Spatial Information Encoding**: None. GPT-3 receives only the text instruction. All
spatial reasoning happens in the CLIP + ViNG visual models. The topological graph of
pre-collected images provides the spatial structure.

**Key Takeaway for Our Design**: LM-Nav shows that an LLM can be effective with a very
narrow role -- just parsing instructions into subgoals -- while other models handle spatial
reasoning and motor control. For our system, we should consider whether the LLM needs to
reason about coordinates at all, or if it should output high-level semantic goals
("explore the doorway on the left") that a Nav2-based planner translates to waypoints.

---

#### 3.5 VoxPoser (CoRL 2023) -- Composable 3D Value Maps via LLM Code Generation

**Paper**: "VoxPoser: Composable 3D Value Maps for Robotic Manipulation with Language Models"
([arXiv 2307.05973](https://arxiv.org/abs/2307.05973),
[project page](https://voxposer.github.io/),
[code](https://github.com/huangwl18/VoxPoser))

**Architecture**: LLM generates Python code that composes 3D voxel value maps (affordance
and avoidance maps). A motion planner then uses these maps to plan trajectories. Two-stage
LLM pipeline: Planner decomposes task into sub-tasks, Composer generates code for each.

**Prompt Structure**:
- **Two-stage prompting** (recursive code generation a la Liang et al.):
  1. **Planner prompt** ([real_planner_prompt.txt](https://voxposer.github.io/prompts/real_planner_prompt.txt)):
     Decomposes high-level instruction into sub-tasks
  2. **Composer prompt** ([real_composer_prompt.txt](https://voxposer.github.io/prompts/real_composer_prompt.txt)):
     Generates Python code for each sub-task
- Each prompt includes **5-20 few-shot examples** of instruction-to-code mappings
- The LLM is instructed to write Python code that calls a set of provided API functions

**Perception API Available to Generated Code**:
```python
detect(obj_name)         # Returns list of dicts: {center_pos, occupancy_grid, normal}
parse_query_obj(query)   # Specialized object/part detection
```

**Value Map API Available to Generated Code**:
```python
get_empty_affordance_map()   # 3D voxel grid, init 0, high values attract
get_empty_avoidance_map()    # 3D voxel grid, init 0, high values repulse
get_empty_rotation_map()     # For end-effector orientation
get_empty_velocity_map()     # For speed control
get_empty_gripper_map()      # For gripper open/close
set_voxel_by_radius(map, xyz, radius_cm, value)  # Paint voxels
cm2index(cm, direction)      # World-to-voxel coordinate conversion
index2cm(index, direction)   # Voxel-to-world coordinate conversion
```

**Execution API**:
```python
execute(movable, affordance_map=None, avoidance_map=None,
        rotation_map=None, velocity_map=None, gripper_map=None)
reset_to_default_pose()
```

**Action Output Format**: Python code (not JSON, not natural language, not tokens). The LLM
writes executable Python that calls the above APIs to compose value maps. A model-based
motion planner then synthesizes closed-loop trajectories from the value maps.

**Chain-of-Thought**: Implicit through the two-stage decomposition (Planner reasons about
sub-tasks, Composer reasons about spatial constraints in code comments).

**Spatial Information Encoding**: Programmatic. The LLM reasons about space by writing
code that queries object positions via `detect()` and manipulates 3D voxel arrays using
numpy operations. Spatial constraints are expressed as code logic, not text descriptions.

**Key Takeaway for Our Design**: VoxPoser's "LLM writes code that calls perception APIs"
pattern is directly applicable via function/tool calling. Instead of the LLM generating
raw code, we can expose our sensor queries as tools:
- `get_lidar_sectors()` returns distance data
- `get_depth_at_pixel(x, y)` returns depth
- `get_obstacle_map()` returns nearby obstacles
The LLM can call these tools to gather information, then call action tools to move.
This is essentially VoxPoser's pattern adapted for modern tool-calling LLMs.

---

#### 3.6 PIVOT (Google DeepMind, ICML 2024) -- Iterative Visual Prompting for VLMs

**Paper**: "PIVOT: Iterative Visual Prompting Elicits Actionable Knowledge for VLMs"
([arXiv 2402.07872](https://arxiv.org/abs/2402.07872),
[project page](https://pivot-prompt.github.io/))

**Architecture**: Uses GPT-4V as-is (zero-shot, no fine-tuning). Candidate actions are
drawn as **numbered arrows on the camera image**. The VLM selects the best arrow by number.
Iteratively refines action selection across multiple rounds.

**Prompt Structure**:
- **Visual annotations on images**: Candidate robot actions are rendered as numbered arrows
  emanating from the robot (or image center)
  - Arrow direction = movement direction
  - Arrow length = movement magnitude
  - Color coding for 3D: red = farther from camera, blue = closer
  - Circle size at arrow endpoint = distance from camera
  - Numbers in circles label each candidate
- **Text prompt structure** (order matters -- paper tested permutations):
  1. **Preamble**: High-level context and arrow semantics explanation
  2. **Image**: Camera view with annotated arrows
  3. **Task instruction**: Specific goal (e.g., "navigate to the kitchen")
- Chain-of-thought: VLM is instructed to "reason through the task first" before selecting
- VLM outputs selected arrow numbers in JSON: `{"points": [3, 7]}`
- VLM also provides "a one sentence analysis" and ranks candidates "from worst to best"

**Iterative Refinement Process** (3 iterations typical):
1. Sample ~10 candidate actions from a distribution (e.g., uniform over action space)
2. Render as numbered arrows on the current camera image
3. Query GPT-4V to select the best 2-3 candidates
4. Fit a new distribution centered on selected candidates (tighter variance)
5. Re-sample from the refined distribution and repeat
6. Final action is the mean of the last round's selected candidates

**Action Output Format**: JSON with selected arrow indices:
```json
{"points": [3, 7]}
```
Plus a natural language explanation. The arrow indices map back to concrete action vectors
(dx, dy, dz for manipulation; linear_vel, angular_vel for navigation).

**Multi-step**: Single-step per PIVOT call, but **3 parallel PIVOT instances** run
simultaneously for robustness, with results aggregated.

**Spatial Information Encoding**: Entirely through visual annotations on the image.
No text-based coordinates, no depth values, no occupancy grids. The VLM reasons about
space by looking at the annotated image.

**Key Takeaway for Our Design**: PIVOT is extremely relevant because it works with
unmodified GPT-4V (no fine-tuning) and addresses the same problem we face -- how to get
a VLM to output precise actions. The idea of annotating the camera image with candidate
action arrows is clever and could be adapted for our robot:
- Overlay 4-8 numbered arrows on the camera image (forward, slight-left, left, etc.)
- Include distance labels ("1m forward", "turn 45 left")
- Ask the VLM to select the best arrow
- This avoids the LLM needing to output precise continuous values

The iterative refinement is probably too slow for our 1-5 second decision budget
(3 iterations x 2-5s each = 6-15s), but a single-pass version with well-chosen candidates
could work.

---

#### 3.7 NavGPT / NavGPT-2 (AAAI 2024 / ECCV 2024) -- Explicit Reasoning for VLN

**Paper**: "NavGPT: Explicit Reasoning in Vision-and-Language Navigation with Large
Language Models"
([arXiv 2305.16986](https://arxiv.org/abs/2305.16986),
[NavGPT-2 arXiv 2407.12366](https://arxiv.org/abs/2407.12366),
[code](https://github.com/GengzeZhou/NavGPT))

**NavGPT-2 Prompt Structure** (most relevant version):
```
"You are navigating in an indoor environment given the instruction:
<INST>{instruction}</INST>; The navigable locations are listed below:
{ "Candidate 1, facing a1 degree, front" : <IMG>{image_tokens}</IMG>;
  "Candidate 2, facing a2 degree, right" : <IMG>{image_tokens}</IMG>;
  ...};
Please choose the next direction."
```

**Key Design Elements**:
- Special tokens: `<IMG>`/`</IMG>` for image token sequences, `<INST>`/`</INST>` for
  navigation instructions
- **Candidate-based selection**: Each navigable direction is presented as a labeled
  candidate with angle + relative direction + visual observation
- Image tokens are generated via Q-former mechanism with instruction-aware processing
- Each navigable view is projected into LLM latent space as fixed-length token sequences

**Action Output Format**: Selection of a candidate direction (by number/name). The model
chooses from the presented options rather than generating free-form actions.

**Chain-of-Thought**: Yes -- NavGPT performs explicit reasoning including:
- Decomposing instructions into sub-goals
- Integrating commonsense knowledge
- Identifying landmarks from observed scenes
- Tracking navigation progress
- Adapting to exceptions with plan adjustments

For GPT-4V training data generation, the prompt instructs: "determine the next step toward
completing this task...describe your immediate environment and specify the direction or
action you will take to proceed. Summarize this in a concise paragraph."

**Spatial Information Encoding**: Angle measurements + cardinal/relative directions
(front, left, right) + visual observations per candidate. No explicit maps or coordinates.

**Key Takeaway for Our Design**: The candidate-based selection pattern is highly applicable.
Instead of asking the LLM "where should I go?" (open-ended), we present candidates:
"Here are 6 options: (1) forward 1m, (2) slight left, (3) sharp left, (4) slight right,
(5) sharp right, (6) turn around. Each with a description of what's visible in that
direction. Choose one." This constrained output space dramatically reduces parsing errors
and hallucination risk.

---

#### 3.8 SG-Nav (NeurIPS 2024) -- 3D Scene Graph Prompting for Navigation

**Paper**: "SG-Nav: Online 3D Scene Graph Prompting for LLM-based Zero-shot Object
Navigation"
([arXiv 2410.08189](https://arxiv.org/abs/2410.08189),
[NeurIPS 2024](https://proceedings.neurips.cc/paper_files/paper/2024/file/098491b37deebbe6c007e69815729e09-Paper-Conference.pdf),
[code](https://github.com/bagh2178/SG-Nav))

**Architecture**: Constructs a hierarchical 3D scene graph online (from visual observations)
and prompts an LLM with text-serialized subgraphs to reason about where a target object
might be.

**Prompt Structure -- Hierarchical Chain-of-Thought**:
- The 3D scene graph has three layers of nodes:
  - **Room nodes**: bedroom, kitchen, hallway, etc.
  - **Group nodes**: clusters of related objects (e.g., "dining set")
  - **Object nodes**: individual objects (chair, table, etc.)
- Edges encode: spatial containment (object-in-group-in-room) and adjacency
- The full graph is serialized as **LLM-friendly text** with hierarchical structure
- At each step, the graph is divided into subgraphs, each prompted to the LLM with a
  hierarchical chain-of-thought:
  1. Predict which **room** is most likely to contain the target object
  2. Within that room, predict which **group** is most relevant
  3. Within that group, identify the most likely **frontier/direction** to explore

**Action Output Format**: The LLM outputs a target frontier/region to explore. A classical
path planner handles motor control to reach the selected frontier.

**Spatial Information Encoding**: Text-serialized scene graph with node names, hierarchical
containment, and spatial adjacency relationships. Distances may be predicted by the LLM
as part of the reasoning chain. No raw images or coordinates in the prompt.

**Re-perception Mechanism**: When the LLM's prediction fails (object not found at expected
location), the system re-perceives the scene and updates the graph, then re-prompts the LLM.

**Key Takeaway for Our Design**: The hierarchical scene graph concept is powerful for indoor
exploration. Our robot already builds a SLAM occupancy grid -- we could augment it with
semantic labels from GPT-4o vision observations to build a simple scene graph:
- Room-level: "kitchen", "hallway", "bedroom" (from visual classification)
- Object-level: discovered objects with approximate positions
- This graph could be serialized into the LLM prompt as exploration context, giving the
  LLM a "mental map" of where it has been and what it has found.

---

#### 3.9 COME-Robot (2024) -- Closed-Loop Mobile Manipulation with GPT-4V

**Paper**: "Closed-Loop Open-Vocabulary Mobile Manipulation with GPT-4V"
([arXiv 2404.10220](https://arxiv.org/abs/2404.10220),
[project page](https://come-robot.github.io/))

**Architecture**: GPT-4V acts as the "brain" -- receives system prompt + user query,
generates Python code that invokes robot primitive action APIs. Execution results
(including captured images) are fed back for closed-loop replanning.

**System Prompt Structure** (6 sections):
1. **Role**: High-level description of the robot's role and capabilities
2. **Feedback Handling**: Guidelines for processing user queries and API execution results
3. **Robot Setup**: Hardware capabilities (arm, mobile base, sensors)
4. **APIs**: Detailed documentation of all primitive action functions
5. **Response Guidelines**: Chain-of-thought reasoning instructions + JSON format specs
6. **Tips**: Practical advice for robust behaviors

**Perception APIs**:
```python
explore_local()          # Returns ObjectMap of nearby objects
explore_global()         # Builds room-level map, returns ObjectMap
report_observation()     # Captures 2D image, accepts SceneObject or string input
```

**Execution APIs**:
```python
navigate(SceneObject)    # Move robot to target object (returns Boolean)
grasp(SceneObject)       # Execute grasp attempt (returns Boolean)
place(SceneObject | location)  # Place held object (returns Boolean)
```

**Action Output Format**: Python code that invokes the above APIs. GPT-4V generates
executable code with chain-of-thought reasoning in comments. The code includes conditional
logic for handling failures (e.g., retry grasp if first attempt fails).

**Chain-of-Thought**: Explicitly required in the system prompt's "Response Guidelines"
section. The model must reason before generating action code.

**Closed-Loop Feedback**: After each API call executes, the result (success/failure +
captured images) is sent back to GPT-4V as a new user message. The model can then reason
about the feedback and generate the next action. This is the key innovation -- it is not
single-shot planning.

**Spatial Information Encoding**: Through the `explore_local()` and `explore_global()`
APIs, which return `ObjectMap` data structures with object names and positions. The LLM
reasons about space through these API returns, not through raw sensor data.

**Key Takeaway for Our Design**: COME-robot is the closest architecture to what we need.
Its pattern of "system prompt with API docs + chain-of-thought + code generation + feedback
loop" maps almost exactly to our proposed tool-calling approach. Key differences:
- COME-robot uses code generation; we should use native tool/function calling (more
  structured, less error-prone)
- COME-robot targets manipulation; we target navigation (different API set)
- COME-robot's `explore_global()` builds a map; we already have SLAM for this
- The closed-loop feedback pattern (action result returned to LLM) is essential

---

#### 3.10 GPT-4V(ision) for Robotics (Microsoft, 2024)

**Paper**: "GPT-4V(ision) for Robotics: Multimodal Task Planning from Human Demonstration"
([arXiv 2311.12015](https://arxiv.org/abs/2311.12015),
[prompts & code](https://microsoft.github.io/GPT4Vision-Robot-Manipulation-Prompts/),
[code](https://github.com/microsoft/GPT4Vision-Robot-Manipulation-Prompts))

**Architecture**: Three-stage pipeline using GPT-4V and GPT-4:
1. **Video Analyzer** (GPT-4V): Watches 5 frames from a human demo video, generates
   text description of actions
2. **Scene Analyzer** (GPT-4V): Encodes environment as structured Python dictionary
3. **Task Planner** (GPT-4): Generates symbolic task plan from descriptions

**Scene Description Output Format** (from Scene Analyzer):
```python
{
  "objects": ["coffee_mug", "office_table", "keyboard"],
  "object_properties": {
    "coffee_mug": ["GRABBABLE"],
    "office_table": [],
    "keyboard": ["GRABBABLE"]
  },
  "spatial_relations": {
    "coffee_mug": "on(<office_table>)",
    "keyboard": "on(<office_table>)"
  },
  "your_explanation": "A desk scene with a mug and keyboard..."
}
```

**Task Planner Action Types** (from Table I in paper):
- `Grab`, `MoveHand`, `Release`, `PickUp`, `Put`, `Rotate`, `Slide`, `MoveOnSurface`

**Action Output Format**: Hardware-independent executable JSON file containing:
- Action sequence with parameters
- Affordance information (approach directions, grasp types, waypoints, postures)

**Spatial Information Encoding**: Object properties (GRABBABLE) and spatial relations
using `inside()` and `on()` predicates. No raw coordinates -- symbolic spatial reasoning.

**Key Takeaway for Our Design**: The structured scene description format (objects +
properties + spatial relations) is a useful template for encoding what the robot has
discovered. We could maintain a similar dictionary that grows as the robot explores,
serving as the LLM's "memory" of the environment.

---

#### 3.11 Probing Prompt Design for Social Robot Navigation (January 2026)

**Paper**: "Probing Prompt Design for Socially Compliant Robot Navigation with Vision
Language Models"
([arXiv 2601.14622](https://arxiv.org/abs/2601.14622))

This is the most recent and directly relevant paper for prompt engineering in robot
navigation with off-the-shelf VLMs (GPT-4o, finetuned small VLMs).

**Prompt Structure -- Systematic Evaluation of 9 Variants**:

Base prompt shared by all variants:
```
"You are an intelligent assistant specializing in socially compliant
robot navigation. You must understand human behaviors, infer intentions,
and plan safe, smooth, and socially appropriate paths."
```

Three **guidance types** (what the model is asked to do):
- **A-series (Action-focused)**: Direct output of navigation commands, no reasoning
- **R-series (Reasoning-oriented)**: "Explain your reasoning clearly" before selecting
- **PR-series (Perception-Reasoning)**: "Explain both perception and reasoning clearly"

Three **motivational framings** (competitive pressure):
- **Against humans (A1, R1, PR1)**: "Perform competitively against humans"
- **Against AI systems (A2, R2, PR2)**: "Perform competitively against other AI systems"
- **Against past self (A3, R3, PR3)**: "Perform competitively against your past self"

This gives 9 prompt variants (3 guidance x 3 framing).

**Action Output Format**: Natural language discrete commands:
- `Move Forward`, `Forward Left`, `Forward Right`, `Stop`, `Turn Left`, `Turn Right`

**Input**: Egocentric RGB images from robot's perspective. No explicit spatial data.

**Key Findings**:
- Non-finetuned **GPT-4o performs best with reasoning-focused + human-competition** (R1)
- Finetuned small VLMs benefit most from **perception-reasoning + self-competition** (PR3)
- **Reasoning prompts consistently outperform action-only prompts** across all models
- The motivational framing matters -- "compete against humans" helps GPT-4o but hurts
  smaller models

**Key Takeaway for Our Design**: This directly informs our system prompt design:
1. Include reasoning instruction ("explain your reasoning before choosing an action")
2. GPT-4o responds well to human-competitive framing
3. Use perception-reasoning prompts if we later switch to a smaller finetuned model
4. Discrete action commands (not continuous values) are the standard output format

---

#### 3.12 SayPlan (CoRL 2023) -- 3D Scene Graphs for Scalable Task Planning

**Paper**: "SayPlan: Grounding Large Language Models using 3D Scene Graphs for Scalable
Robot Task Planning"
([arXiv 2307.06135](https://arxiv.org/abs/2307.06135),
[project page](https://sayplan.github.io/))

**Prompt Structure**:
- 3D scene graph serialized as **JSON** for LLM input
- Hierarchical graph with rooms, assets, and objects as nodes
- LLM performs "semantic search" on a collapsed graph to find task-relevant subgraphs
- Iterative replanning: initial plan is validated against a scene graph simulator,
  infeasible actions trigger re-prompting

**Action Format**: Fixed action predicates:
```
pick(robot, left_hand, apple)
place(robot, left_hand, apple, table)
navigate(robot, kitchen)
```

**Spatial Information Encoding**: JSON-serialized 3D scene graph with hierarchical
room/asset/object structure and spatial relationships.

**Key Takeaway for Our Design**: The JSON scene graph as LLM input is a proven pattern.
For our robot, we could serialize the SLAM map's semantic annotations as:
```json
{
  "rooms_discovered": [
    {"name": "hallway", "explored_pct": 80, "objects": ["door", "light_switch"]},
    {"name": "kitchen", "explored_pct": 30, "objects": ["counter", "refrigerator"]}
  ],
  "current_room": "hallway",
  "frontiers": [
    {"direction": "left", "distance_m": 2.3, "description": "unexplored doorway"},
    {"direction": "ahead", "distance_m": 5.1, "description": "long corridor"}
  ]
}
```

---

### 3.13 Synthesis: Design Patterns for Our Robot's LLM Navigation Prompt

Based on the survey of all 12 projects above, here are the key design patterns and decisions
for our ROSMASTER A1 robot:

#### Pattern 1: Action Space Design

| Project | Action Representation | Output Format |
|---------|----------------------|---------------|
| NaVid | Discrete set + continuous arg | Natural language parsed by regex |
| RT-2 | 256-bin discretized tokens | Token string "1 128 91 241 5 101 127" |
| SayCan | Fixed skill primitives | Skill selection by scoring |
| LM-Nav | Landmark names only | Numbered text list |
| VoxPoser | Python code calling APIs | Executable Python |
| PIVOT | Arrow selection on annotated image | JSON: `{"points": [3, 7]}` |
| NavGPT-2 | Candidate direction selection | Selected candidate ID |
| SG-Nav | Frontier/region selection | Target frontier |
| COME-Robot | Python code calling APIs | Executable Python |
| Microsoft GPT-4V | Symbolic action predicates | JSON action sequence |
| Social Nav | Discrete commands | Natural language |
| SayPlan | Fixed action predicates | `pick(robot, hand, obj)` format |

**Decision for our robot**: Use **tool/function calling** with a discrete action set.
This combines the structured output reliability of SayCan/SayPlan with the API pattern
of COME-robot/VoxPoser, adapted for modern LLM tool-calling capabilities. The LLM selects
from pre-defined tools rather than generating free-form text or code.

#### Pattern 2: Spatial Information Encoding

| Project | Spatial Data in LLM Prompt | Encoding Method |
|---------|---------------------------|-----------------|
| NaVid | None (visual only) | Learned from video frames |
| RT-2 | None (visual only) | Learned from images |
| SayCan | None | Affordance model handles it |
| LM-Nav | None | CLIP + ViNG handle it |
| VoxPoser | Object positions via API | Python code + detect() calls |
| PIVOT | Visual arrows on image | Annotated camera image |
| NavGPT-2 | Angle + relative direction per candidate | Text: "facing 30 degree, front" |
| SG-Nav | Hierarchical scene graph | Text-serialized graph |
| COME-Robot | Object maps via API | explore_local() returns |
| Social Nav | None (egocentric image only) | Implicit from image |
| SayPlan | JSON scene graph | Hierarchical JSON |

**Decision for our robot**: Use a **hybrid approach**:
1. Camera image (visual spatial reasoning) -- sent with every decision call
2. Text-structured LiDAR summary (6 sectors with distances) -- in system context
3. Semantic exploration memory (rooms discovered, objects seen) -- JSON in context
4. Current pose and heading -- text in context
This combines the visual approach (PIVOT, NaVid) with the structured data approach
(SG-Nav, SayPlan, NavGPT-2).

#### Pattern 3: Reasoning Format

| Project | Chain-of-Thought? | Reasoning Style |
|---------|-------------------|-----------------|
| NaVid | Partial (10k reasoning samples) | Implicit via training |
| RT-2 | No | Direct action prediction |
| SayCan | Yes (17 few-shot examples) | Explicit task decomposition |
| LM-Nav | No | Direct landmark extraction |
| VoxPoser | Implicit (two-stage decomposition) | Code comments |
| PIVOT | Yes ("reason through the task first") | One-sentence analysis |
| NavGPT | Yes (explicit multi-step) | Full paragraph reasoning |
| SG-Nav | Yes (hierarchical CoT) | Room -> group -> object |
| COME-Robot | Yes (required in prompt) | Comments in generated code |
| Social Nav | R-series and PR-series outperform A-series | Explicit reasoning text |

**Decision for our robot**: **Require chain-of-thought reasoning**. The evidence is
overwhelming -- every project that uses off-the-shelf LLMs (as opposed to fine-tuned
models) benefits from explicit reasoning. Our format:
```json
{
  "perception": "I see a doorway to my left about 2m away, and a wall ahead at 1.5m...",
  "reasoning": "The doorway leads to an unexplored area. The hallway ahead is a dead end...",
  "action": "navigate_toward",
  "parameters": {"direction": "left", "reason": "unexplored doorway"}
}
```

#### Pattern 4: Single-Step vs. Multi-Step Output

| Project | Steps per LLM Call | Notes |
|---------|-------------------|-------|
| NaVid | 1 | One action per video input |
| RT-2 | 1 | One 7-DoF action per image |
| SayCan | 1 (iterative) | One skill per call, loop until done |
| LM-Nav | All landmarks at once | Full plan, then execute sequentially |
| VoxPoser | Multi (via code) | Code can contain loops/conditionals |
| PIVOT | 1 (with 3 iterations) | Single action, refined iteratively |
| NavGPT | 1 | One direction per step |
| SG-Nav | 1 | One frontier per step |
| COME-Robot | Multi (via code) | Code can contain sequences |
| SayPlan | Multi (full plan) | Full task plan, then refine |

**Decision for our robot**: **Single primary action + optional short plan**. The LLM
outputs one immediate action to execute, plus an optional 2-3 step lookahead plan that
informs but does not commit to future actions. This matches the SayCan iterative pattern
while allowing the LLM to express intent:
```json
{
  "immediate_action": {"tool": "move_toward", "direction": "left", "speed": "slow"},
  "plan_context": "Heading toward doorway, will turn right once through it",
  "next_decision_hint": "After passing through doorway, look for kitchen landmarks"
}
```

#### Pattern 5: Feedback Loop Design

| Project | Closed-Loop? | Feedback Mechanism |
|---------|-------------|-------------------|
| NaVid | Yes (video stream) | Continuous visual observation |
| RT-2 | Yes (1-3 Hz) | New image each cycle |
| SayCan | Yes (after each skill) | Skill success/failure + new state |
| COME-Robot | Yes (explicit) | API results + captured images returned |
| PIVOT | Yes (iterative) | Previous round's selections inform next |
| SG-Nav | Yes (re-perception) | Scene graph updated on failure |
| NavGPT | Yes (per step) | New observations at each navigable point |

**Decision for our robot**: **Closed-loop with explicit feedback**. After each action
executes, the next LLM call includes:
1. Result of the previous action (success, distance moved, any issues)
2. Fresh camera image from the new position
3. Updated sensor readings (LiDAR, pose)
4. Updated exploration memory (what changed)

---

### 3.14 Proposed Tool/Function Definitions

Based on the patterns above, here is the proposed tool set for the LLM navigation system:

**Perception Tools** (LLM calls these to gather information):
```
get_sensor_summary()     -- Returns LiDAR sectors, pose, heading, battery
get_exploration_status() -- Returns rooms discovered, frontier count, coverage %
describe_scene()         -- Returns latest camera observation description
get_depth_at(direction)  -- Returns distance to nearest obstacle in a direction
```

**Action Tools** (LLM calls these to move):
```
move_forward(distance_m: 0.5|1.0|2.0)     -- Move forward specified distance
turn(degrees: -180 to 180)                 -- Turn in place (negative = left)
navigate_to_frontier(frontier_id: int)     -- Use Nav2 to reach a specific frontier
explore_direction(direction: str)          -- Move toward "left"/"right"/"ahead"
stop()                                     -- Emergency or intentional stop
```

**Communication Tools** (LLM calls these for output):
```
speak(message: str)                  -- Announce observation/finding via TTS
mark_discovery(label: str, notes: str)  -- Record a point of interest on the map
```

**Proposed System Prompt Structure** (informed by all surveyed projects):
```
[Role]: You are the navigation intelligence for an indoor exploration robot.
[Capabilities]: Camera (RGB), LiDAR (360 degrees), depth sensing, SLAM mapping.
[Philosophy]: Explore systematically, prioritize unexplored areas, investigate
  interesting features (doorways, rooms, objects).
[Safety]: Never exceed 0.2 m/s. Stop if obstacle < 0.3m. Do not attempt stairs.
[Reasoning]: Before each action, describe what you perceive and explain your
  reasoning for the chosen action.
[Context]: {dynamic sensor data, exploration memory, previous action result}
[Image]: {current camera frame}
```

This structure draws from:
- COME-Robot's system prompt sections (role, APIs, guidelines, tips)
- Social Nav's reasoning requirement (PR-series outperforms A-series)
- SayCan's iterative single-action pattern
- PIVOT's visual reasoning from camera images
- SG-Nav's hierarchical spatial memory
- VoxPoser/COME-Robot's API-based interaction pattern

---

### 3.15 Current Codebase Prompt & Action Analysis

Before designing the new navigation prompt, we must understand what exists today and what can be reused.

#### Current System Prompt (`_build_system_prompt()`, line 843)

The production `voice_mapper.py` system prompt is ~950 tokens and defines:
- **Identity**: "voice-controlled mapping robot with camera, depth camera, LiDAR, and SLAM"
- **14 actions**: Each defined inline as JSON example with field descriptions
- **Context section**: Describes what dynamic data the LLM receives (obstacle distances, map coverage, etc.)
- **Personality**: "Enthusiastic cartographer/explorer, curious, celebrates discoveries"
- **Constraints**: "Always respond with valid JSON. Keep speech short (1-2 sentences)."

**Weaknesses for navigation use**:
1. **No reasoning instruction** — the prompt says "respond with JSON" but never asks the LLM to explain its thinking. Social Nav research (section 3.11) shows reasoning prompts significantly outperform action-only prompts.
2. **No camera awareness** — the brain (`think()`) never receives images, yet the prompt doesn't acknowledge this blindness. The LLM doesn't know what it can and cannot see.
3. **Continuous velocity parameters** — the LLM must output precise `linear: 0.15, angular: 0.3` values. RT-2 research (section 3.2) shows discrete action spaces are more reliable for non-fine-tuned models.
4. **No exploration strategy** — the prompt doesn't instruct the LLM on how to explore (systematic coverage, doorway prioritization, backtracking).
5. **No safety rules** — no mention of minimum obstacle distance, maximum speed near walls, or what to do when the safety layer overrides.

#### Current Action Vocabulary Comparison

| voice_mapper.py Action | llm_robot_brain.py Equivalent | Navigation Relevance |
|------------------------|------------------------------|---------------------|
| `move` (linear, angular, duration) | `move_forward/backward` (speed, duration) | **High** — primary movement |
| `turn_around` | `turn_left/right` (degrees) | **High** — reorientation |
| `look` | `look_around` (seewhat) | **High** — gather visual info |
| `get_dist` | `get_distance` (x, y) | **Medium** — depth queries |
| `explore` | — | **Replace** — this is the algorithmic loop we're replacing |
| `navigate` (x, y) | `navigate_to` (named point) | **High** — Nav2 goal delegation |
| `start/stop_mapping` | `start/stop_mapping` | **Low** — session-level, not per-decision |
| `start/stop_vslam` | — | **Low** — session-level |
| `stop` | `stop` | **Critical** — emergency and intentional stop |
| `status` | — | **Low** — informational only |
| `speak` | (via `response` field) | **Medium** — announce discoveries |
| — | `label_room`, `save_map` | **Medium** — semantic mapping |

**Key insight**: The current action vocabulary is designed for **voice command interpretation** (user says "explore", LLM starts exploration). The navigation vocabulary needs to be designed for **continuous autonomous decisions** (LLM sees environment, decides where to go next).

#### Current Structured Output: Prompt-Instructed JSON (No Schema Enforcement)

Both `voice_mapper.py` and `llm_robot_brain.py` use **prompt-instructed JSON** — the system prompt contains example JSON and says "respond with valid JSON." Neither uses:
- OpenAI's `tools`/`functions` parameter (function calling)
- OpenAI's `response_format: json_schema` (structured outputs)
- Anthropic's `tools` parameter
- Any formal schema validation

**Failure modes observed**:
- JSON parsing relies on `find('{')` / `rfind('}')` — fragile if LLM wraps JSON in markdown or includes nested braces
- No type validation — if the LLM outputs `"linear": "fast"` instead of `"linear": 0.15`, the `float()` cast in `execute()` will throw
- No required field enforcement — if the LLM omits `action`, the default is `"speak"` (silent failure)

**Recommendation**: Use native **tool/function calling** for all three providers. This provides schema-enforced output, eliminates JSON parsing issues, and enables `strict: true` mode for guaranteed schema adherence.

#### Current Conversation History: Minimal and Text-Only

- `voice_mapper.py`: Rolling window of last 4 messages (2 turns) in GPT-4o context, trimmed from storage of 10 entries. Text only — no images in history.
- `llm_robot_brain.py`: Unbounded history — every message appended forever, never trimmed. Will exceed context window on long sessions.

For navigation, the "conversation history" concept must be replaced with an **exploration state** that tracks:
- What rooms/areas have been visited
- What was seen in each area
- Which directions have been tried
- What the previous decision was and its outcome

---

### 3.16 Tool/Function Calling Schema Design

Based on the prior art survey (sections 3.1–3.14) and provider API research, the navigation system should use **native tool/function calling** rather than prompt-instructed JSON. This provides:
- Schema-enforced arguments (no malformed motor commands)
- Clear separation of perception tools (gather info) vs. action tools (move)
- Multi-tool calls in a single response (e.g., speak + move simultaneously)
- Provider-agnostic abstraction (same tool set, different wire formats)

#### Proposed Navigation Tool Set

**7 tools** organized into 3 categories:

##### Action Tools (4 tools — the LLM calls these to move)

**Tool 1: `move_toward`** — Move in a direction at a speed
```
Parameters:
  direction: enum ["forward", "forward_left", "left", "forward_right", "right", "backward"]
  speed: enum ["slow", "medium", "fast"]
    slow = 0.08 m/s (near obstacles, tight spaces)
    medium = 0.12 m/s (normal exploration)
    fast = 0.18 m/s (open corridors, clear path)
  duration_s: number (1.0 to 8.0, default 3.0)
```
**Why discrete direction + speed**: RT-2 (section 3.2) and Social Nav (section 3.11) show discrete action spaces outperform continuous for non-fine-tuned VLMs. PIVOT (section 3.6) confirms arrow-selection outperforms free-form value prediction.

**Mapping to cmd_vel**: Each direction maps to fixed (linear, angular) pairs:
- `forward` → (speed, 0.0)
- `forward_left` → (speed, +0.25)
- `left` → (0.06, +0.45)
- `forward_right` → (speed, -0.25)
- `right` → (0.06, -0.45)
- `backward` → (-speed, 0.0)

**Tool 2: `navigate_to_goal`** — Delegate pathfinding to Nav2
```
Parameters:
  goal_type: enum ["frontier", "coordinates", "retrace"]
  frontier_id: integer (optional — which frontier from the sensor summary)
  x: number (optional — map coordinates)
  y: number (optional — map coordinates)
  reason: string (why this goal — logged for exploration memory)
```
**When to use**: When the destination is >2m away and Nav2 can handle pathfinding. The LLM picks the strategic goal; Nav2 handles local obstacle avoidance en route.

**Tool 3: `rotate`** — Turn in place (Ackerman-adapted)
```
Parameters:
  degrees: enum [-180, -135, -90, -45, 45, 90, 135, 180]
    Negative = left, Positive = right
```
**Why enum not continuous**: Discrete turn options align with PIVOT's candidate arrows and NavGPT-2's candidate selection. The Ackerman steering constraint means exact rotation angles aren't achievable anyway — the `_execute_uturn()` (for 180°) or arc-based turns (for smaller angles) will approximate.

**Tool 4: `stop_robot`** — Immediate halt
```
Parameters:
  reason: enum ["obstacle_detected", "exploration_complete", "need_to_observe", "user_requested", "uncertain"]
```
**Why reason is required**: Forces the LLM to justify stopping, preventing overly cautious behavior. Also logged for debugging.

##### Perception Tools (2 tools — the LLM calls these to gather information)

**Tool 5: `observe_scene`** — Request detailed visual description
```
Parameters:
  focus: enum ["general", "doorways", "obstacles", "objects", "floor_surface"]
```
**Purpose**: Replaces the current `observe()` call. The LLM can request a focused observation when it needs more detail about a specific aspect. Returns a text description (from a secondary VLM call or local model).

**Note**: In the standard decision loop, the camera image is ALREADY included with every call (as per Phase 2 design). This tool is for when the LLM wants a higher-detail analysis — e.g., "Is that a doorway or a dark wall?" It triggers a `detail: "high"` image analysis.

**Tool 6: `check_path_clear`** — Query if a specific direction is traversable
```
Parameters:
  direction: enum ["forward", "forward_left", "left", "forward_right", "right", "backward"]
  distance_m: number (0.5 to 5.0)
```
**Returns**: JSON with `{clear: bool, obstacle_distance_m: float, obstacle_type: string}`
**Implementation**: Samples LiDAR arc + depth points in the specified direction. No LLM call — pure sensor query, <5ms.
**Purpose**: Gives the LLM on-demand obstacle checking beyond the standard 12-sector LiDAR summary. Inspired by VoxPoser's `detect()` API pattern (section 3.5).

##### Communication Tools (1 tool — for output)

**Tool 7: `report_discovery`** — Announce and log a finding
```
Parameters:
  label: string (what was discovered — "kitchen", "doorway", "dead_end", etc.)
  notes: string (brief description)
  significance: enum ["landmark", "room", "obstacle", "point_of_interest"]
```
**Implementation**: Logs to `DiscoveryLog`, speaks via TTS, and annotates the SLAM map at current position. Inspired by SG-Nav's semantic scene graph building (section 3.8) and SayPlan's `label_room()` pattern (section 3.12).

#### Provider-Specific Wire Format

All 7 tools use the same logical schema but different JSON wire formats per provider:

**OpenAI format** (one tool example):
```json
{
  "type": "function",
  "function": {
    "name": "move_toward",
    "description": "Move the robot in a direction at a specified speed. Use for short-range navigation (under 2m). For longer distances, use navigate_to_goal instead.",
    "parameters": {
      "type": "object",
      "properties": {
        "direction": {
          "type": "string",
          "enum": ["forward", "forward_left", "left", "forward_right", "right", "backward"],
          "description": "Direction to move relative to current heading"
        },
        "speed": {
          "type": "string",
          "enum": ["slow", "medium", "fast"],
          "description": "Movement speed: slow (0.08 m/s, near obstacles), medium (0.12 m/s, normal), fast (0.18 m/s, open space)"
        },
        "duration_s": {
          "type": "number",
          "description": "Movement duration in seconds (1.0-8.0)"
        }
      },
      "required": ["direction", "speed"],
      "additionalProperties": false
    },
    "strict": true
  }
}
```

**Anthropic format** (same tool):
```json
{
  "name": "move_toward",
  "description": "Move the robot in a direction at a specified speed. Use for short-range navigation (under 2m). For longer distances, use navigate_to_goal instead.",
  "input_schema": {
    "type": "object",
    "properties": {
      "direction": {
        "type": "string",
        "enum": ["forward", "forward_left", "left", "forward_right", "right", "backward"],
        "description": "Direction to move relative to current heading"
      },
      "speed": {
        "type": "string",
        "enum": ["slow", "medium", "fast"],
        "description": "Movement speed: slow (0.08 m/s, near obstacles), medium (0.12 m/s, normal), fast (0.18 m/s, open space)"
      },
      "duration_s": {
        "type": "number",
        "description": "Movement duration in seconds (1.0-8.0)"
      }
    },
    "required": ["direction", "speed"]
  },
  "strict": true
}
```

**Gemini format** (same tool):
```json
{
  "functionDeclarations": [{
    "name": "move_toward",
    "description": "Move the robot in a direction at a specified speed. Use for short-range navigation (under 2m). For longer distances, use navigate_to_goal instead.",
    "parameters": {
      "type": "object",
      "properties": {
        "direction": {
          "type": "string",
          "enum": ["forward", "forward_left", "left", "forward_right", "right", "backward"],
          "description": "Direction to move relative to current heading"
        },
        "speed": {
          "type": "string",
          "enum": ["slow", "medium", "fast"],
          "description": "Movement speed: slow (0.08 m/s, near obstacles), medium (0.12 m/s, normal), fast (0.18 m/s, open space)"
        },
        "duration_s": {
          "type": "number",
          "description": "Movement duration in seconds (1.0-8.0)"
        }
      },
      "required": ["direction", "speed"]
    }
  }]
}
```

#### Provider Feature Comparison for Navigation

| Feature | OpenAI GPT-4o/mini | Anthropic Claude | Google Gemini 2.0 Flash |
|---------|-------------------|-----------------|----------------------|
| Tool schema field | `parameters` (JSON Schema) | `input_schema` (JSON Schema) | `parameters` (OpenAPI 3.0 subset) |
| Strict mode | `strict: true` on function | `strict: true` on tool | `mode: "ANY"` guarantees schema |
| Force tool call | `tool_choice: "required"` | `tool_choice: {type: "any"}` | `mode: "ANY"` |
| Images + tools | Yes (native multimodal) | Yes (up to 100 images/request) | Yes (native multimodal) |
| Parallel tool calls | Yes (`parallel_tool_calls: true`) | Yes (multiple `tool_use` blocks) | Yes (multiple `functionCall` parts) |
| Streaming tool args | Yes (SSE) | Yes (fine-grained streaming beta) | Yes (`streamFunctionCallArguments`) |
| Real-time API | Yes (WebSocket) | No (HTTP only) | Yes (Live API WebSocket) |
| Max tools/request | 128 | No hard limit (context-bounded) | 128 |

**Recommendation for navigation**: Use `tool_choice: "required"` (or equivalent) to **force** the LLM to always call a tool. The navigation loop should never receive a text-only response — every decision cycle must produce an action or perception query. The one exception is when using chain-of-thought: let the LLM emit text reasoning BEFORE the tool call (Anthropic naturally supports this with mixed `text` + `tool_use` content blocks; OpenAI requires `parallel_tool_calls: false` for sequential reasoning+action).

---

### 3.17 Structured Output & Decision Format

#### Decision Cycle Output Structure

Each navigation decision cycle produces one of:
1. **Action tool call** — the LLM moves the robot (`move_toward`, `navigate_to_goal`, `rotate`, `stop_robot`)
2. **Perception tool call** — the LLM gathers more info before acting (`observe_scene`, `check_path_clear`)
3. **Communication tool call** — the LLM reports a discovery (`report_discovery`), possibly alongside an action

The LLM may call **multiple tools** in one response (e.g., `report_discovery` + `move_toward`), but should call at most **one action tool** per decision cycle to prevent conflicting motor commands.

#### Chain-of-Thought in Tool Calling

Based on Social Nav findings (section 3.11: reasoning prompts outperform action-only), the LLM should reason before acting. Provider-specific approaches:

**Anthropic Claude** (natural mixed content):
```json
{
  "content": [
    {
      "type": "text",
      "text": "I see a doorway to my left about 2m away based on the LiDAR gap at 270°. The hallway ahead narrows to 0.5m (wall). I should explore through the left doorway — it leads to an unmapped area."
    },
    {
      "type": "tool_use",
      "name": "move_toward",
      "input": {"direction": "left", "speed": "slow", "duration_s": 3.0}
    }
  ]
}
```

**OpenAI GPT-4o** (reasoning via system prompt instruction):
The system prompt instructs: "Before calling any tool, briefly state what you observe and why you chose this action." With `parallel_tool_calls: false`, the model can output reasoning text followed by a single tool call. Alternatively, add a `reasoning` field to each tool's parameters.

**Gemini** (reasoning in text parts):
Similar to Claude — Gemini can include text parts before `functionCall` parts in the same response.

#### Tool Call Result Format (Fed Back to LLM)

After executing a tool, the result is sent back as the next message. Standardized result format:

**Action result** (returned after `move_toward`, `rotate`, `navigate_to_goal`):
```json
{
  "success": true,
  "actual_distance_m": 0.35,
  "actual_duration_s": 3.0,
  "stopped_early": false,
  "stopped_reason": null,
  "new_position": {"x": 2.1, "y": -1.8, "heading_deg": 315, "heading_cardinal": "NW"},
  "safety_override": false
}
```

**Safety-overridden result** (when the safety layer modifies or blocks a command):
```json
{
  "success": false,
  "actual_distance_m": 0.12,
  "actual_duration_s": 0.8,
  "stopped_early": true,
  "stopped_reason": "obstacle_at_0.3m_ahead",
  "safety_override": true,
  "safety_message": "Movement stopped: obstacle detected 0.3m ahead in the forward direction. Consider turning or choosing a different direction."
}
```

**Perception result** (returned after `check_path_clear`):
```json
{
  "clear": false,
  "obstacle_distance_m": 1.2,
  "obstacle_type": "wall",
  "traversable_width_m": 0.0,
  "recommendation": "Direction blocked. Try forward_left or forward_right."
}
```

---

### 3.18 System Prompt Design

Drawing from all prior art findings, the system prompt has 7 sections:

#### Section 1: Role & Identity (~60 tokens)

```
You are the autonomous navigation intelligence for an indoor exploration robot
(ROSMASTER A1, Ackerman steering). You continuously perceive the environment through
a camera, LiDAR, and depth sensors, and decide where to explore next. Your goal is
to systematically discover and map all accessible rooms and areas.
```

**Informed by**: COME-Robot's role section (3.9), Social Nav's reasoning framing (3.11)

#### Section 2: Perception Summary Format (~40 tokens)

```
Each decision cycle, you receive:
- An annotated camera image (RGB with depth values overlaid at 7 points and heading indicator)
- A 12-sector LiDAR summary with distances and qualitative labels (CLEAR/NEAR/OBSTACLE/WALL)
- Doorway/gap detections from LiDAR analysis
- Your current position, heading, speed, and exploration statistics
- The result of your previous action (success/failure, actual distance moved, safety overrides)
- Your exploration memory (rooms discovered, areas visited, notable observations)
```

#### Section 3: Exploration Philosophy (~80 tokens)

```
EXPLORATION STRATEGY:
1. Prioritize unexplored areas — seek doorways, openings, and paths to new rooms
2. Be systematic — don't revisit areas you've already explored unless seeking a missed path
3. When you see a doorway or opening, investigate it — doorways lead to new rooms
4. In open spaces, sweep the perimeter before crossing the center
5. If stuck or in a dead end, backtrack to the last junction and try a different direction
6. Announce significant discoveries (new rooms, interesting objects, layout features)
7. Maintain awareness of your return path — don't explore so far that return is uncertain
```

**Informed by**: SG-Nav's hierarchical reasoning (3.8), current `exploration_loop()` frontier strategy

#### Section 4: Safety Rules (~50 tokens)

```
SAFETY CONSTRAINTS (non-negotiable):
- Maximum speed: 0.18 m/s. Use slow (0.08) near obstacles, medium (0.12) normally.
- A safety layer monitors LiDAR at 10Hz and WILL override your commands if obstacles are too close.
- If your action was safety-overridden, do NOT retry the same direction. Choose a different path.
- Never attempt to push through narrow gaps (<0.5m width) — the robot is 0.25m wide.
- Ackerman steering: the robot cannot spin in place. All turns require forward motion.
- If you are unsure about safety, call stop_robot or check_path_clear before moving.
```

#### Section 5: Reasoning Instruction (~30 tokens)

```
REASONING (required):
Before each action, briefly state:
1. What you PERCEIVE (key observations from camera + LiDAR)
2. What you REASON (why this action advances exploration)
Keep reasoning to 1-3 sentences. Be concise.
```

**Informed by**: Social Nav's PR-series prompt design (3.11) — perception+reasoning outperforms action-only. NavGPT's explicit reasoning steps (3.7).

#### Section 6: Dynamic Context Block (~120 tokens, injected per call)

```
CURRENT STATE:
  Position: (2.3, -1.5) meters from start | Heading: 47° (NE)
  Speed: 0.12 m/s forward | Distance traveled: 23.4m
  Exploration time: 4m 32s | Coverage: 34%
  VSLAM: tracking (quality: good) | Battery: 72%

LIDAR (12 sectors, 30° each):
  000° front: 2.9m CLEAR | 030° front-right: 3.2m CLEAR
  060° right-front: 1.8m CLEAR | 090° right: 0.5m WALL
  ... (remaining sectors)
  Nearest: 0.5m at 090° (right)
  Gaps: doorway at 315°, width 0.9m, dist 2.1m

PREVIOUS ACTION: move_toward(forward, medium, 3.0)
  Result: success, moved 0.36m, no safety override

EXPLORATION MEMORY:
  Rooms: hallway (80% explored), kitchen (30% explored)
  Current room: hallway
  Frontiers: 3 open — largest at 315° (doorway, 2.1m), second at 180° (corridor, 4.5m)
  Last 3 decisions: forward→success, forward→success, left→safety_override
```

**Informed by**: Phase 2's enhanced sensor representation (sections 2.3–2.6), SG-Nav's scene graph context (3.8), COME-Robot's feedback handling (3.9)

#### Section 7: Camera Image (~85 tokens OpenAI / ~410 tokens Claude)

Sent as the user message alongside the context block. Annotated RGB 640×480 with:
- Depth values at 7 points (white text, black outline)
- Heading indicator (arrow + "NE 47°" in top-right corner)

#### Total System Prompt Token Budget

| Section | Tokens (est.) | Cacheable? |
|---------|--------------|-----------|
| Sections 1-5 (static) | ~260 | **Yes** — identical every call |
| Tool definitions (7 tools) | ~400 | **Yes** — identical every call |
| Section 6 (dynamic context) | ~120 | No — changes every call |
| Camera image | 85–410 | No — changes every call |
| Exploration memory | ~150 | No — grows over session |
| **Total** | **~1,015–1,340** | ~660 cacheable (50–65%) |

With prompt caching, the effective input cost is reduced by ~15-25% since the static portions (660 tokens) are cached across calls.

---

### 3.19 Memory & Exploration Context Design

The current system has no navigation memory — `observe()` results are spoken and discarded, conversation history is a 4-message text window. The new system needs structured exploration memory that persists across decision cycles.

#### Memory Architecture: Three Tiers

**Tier 1: Immediate Context (included every call, ~30 tokens)**
- Previous action + result (success/failure/override)
- Current motion state (moving/stopped, direction, speed)
- Time since last meaningful movement (stuck detection)

**Tier 2: Recent Decisions (rolling window, ~80 tokens)**
- Last 5 decisions with outcomes: `[{action, result, position}, ...]`
- Used for: detecting loops (same action repeated), backtracking triggers, stuck detection
- Inspired by NaVid's temporal frame context (section 3.12 from Phase 2)

**Tier 3: Exploration Map (cumulative, ~100-200 tokens, summarized)**
- Rooms discovered with exploration percentage and notable objects
- Current room identification
- Open frontiers with direction, distance, and descriptions
- Dead ends encountered (directions to avoid)
- Significant observations from previous `observe_scene` calls

```json
{
  "rooms": [
    {"name": "hallway", "explored_pct": 80, "objects": ["light_switch", "doormat"], "entered_from": "start"},
    {"name": "kitchen", "explored_pct": 30, "objects": ["counter", "refrigerator"], "entered_from": "hallway_north_door"}
  ],
  "current_room": "hallway",
  "dead_ends": [
    {"direction_deg": 90, "reason": "wall", "position": {"x": 3.1, "y": -1.2}}
  ],
  "frontiers": [
    {"id": 1, "direction_deg": 315, "distance_m": 2.1, "type": "doorway", "width_m": 0.9},
    {"id": 2, "direction_deg": 180, "distance_m": 4.5, "type": "corridor"}
  ],
  "total_discoveries": 7,
  "exploration_duration_min": 4.5
}
```

**Implementation**: The exploration memory is maintained by the navigation controller (Python), not by the LLM itself. Each LLM decision cycle receives the current memory snapshot as text in the dynamic context block. After each action completes:
1. Update position from odometry
2. Update room identification (from frontier crossing detection)
3. Update frontier list (from SLAM map analysis)
4. Append any `report_discovery` calls to the room's object list
5. Serialize to text for the next LLM call

**Memory budget**: The exploration memory starts at ~80 tokens (empty map) and grows to ~200 tokens over a full house exploration. This is well within the total budget of ~1,700 tokens/cycle from Phase 2.

#### Conversation History: Replace with Action History

The current 4-message conversation history is inappropriate for navigation. Replace with:

**Action history format** (last 5 cycles, ~100 tokens):
```
RECENT DECISIONS:
  [5] move_toward(forward, medium) → success, +0.36m
  [4] move_toward(forward, medium) → success, +0.35m
  [3] move_toward(forward_left, slow) → safety_override at 0.3m
  [2] rotate(-90) → success, now facing NW
  [1] check_path_clear(left, 2.0) → clear, no obstacles
```

This gives the LLM context on what it just did without the token overhead of full message history. The reasoning text from previous cycles is NOT included (too expensive) — only the action + result.

---

### 3.20 Complete Decision Loop Interaction Sequence

Putting sections 3.16–3.19 together, here is the full interaction sequence for one navigation decision cycle:

```
Step 1: PREPARE SENSOR SUMMARY (Python, <20ms)
  ├── Capture latest camera frame → annotate with depth + heading → JPEG → base64
  ├── Generate 12-sector LiDAR text summary + doorway gaps
  ├── Read current position, heading, speed from odometry
  ├── Get previous action result from execution layer
  ├── Serialize exploration memory to text
  └── Assemble full context block

Step 2: CALL LLM (API, 300ms–3000ms)
  ├── System message: static prompt (sections 1-5) — CACHED
  ├── User message: [annotated camera image] + [dynamic context block]
  ├── Tools: 7 tool definitions — CACHED
  ├── Tool choice: "required" (must call at least one tool)
  └── Model generates: reasoning text + tool call(s)

Step 3: PARSE & VALIDATE (Python, <5ms)
  ├── Extract tool call name + arguments from response
  ├── Validate arguments against schema (already guaranteed by strict mode)
  ├── If action tool: pass to safety layer for pre-execution check
  └── If perception tool: execute immediately, return result to LLM

Step 4: EXECUTE ACTION (Robot, 1-8 seconds)
  ├── Safety layer validates/modifies command (see Phase 4)
  ├── Publish cmd_vel or Nav2 goal
  ├── Monitor execution (obstacle detection, movement tracking)
  └── Record actual outcome (distance moved, safety overrides, position change)

Step 5: UPDATE MEMORY (Python, <5ms)
  ├── Update position and heading
  ├── Update room tracking
  ├── Record action outcome in recent decisions list
  ├── Update frontier list from latest SLAM map
  └── Return to Step 1
```

**Total cycle time**: ~1.5–11 seconds (dominated by API latency + action execution)
**Effective decision frequency**: ~0.1–0.7 Hz (depends on action duration and API speed)

#### Perception Tool Handling (Sub-Loop)

When the LLM calls a perception tool (`observe_scene` or `check_path_clear`), a sub-loop occurs:
1. Execute the perception query locally (<50ms for `check_path_clear`, ~1-3s for `observe_scene` which invokes a secondary VLM call)
2. Return the result as a tool_result message
3. The LLM then makes a SECOND call with the perception result, producing an action tool call
4. This doubles the API latency for that cycle (~2-6 seconds total) but gives the LLM on-demand information

**Recommendation**: Limit perception sub-loops to at most 1 per decision cycle. If the LLM calls a perception tool, the next response MUST be an action tool (enforced by switching `tool_choice` to action-tools-only for the follow-up call).

---

### 3.21 Key Findings Summary

1. **Native tool/function calling should replace prompt-instructed JSON** — all three providers (OpenAI, Anthropic, Gemini) support strict schema enforcement via `strict: true` or `mode: "ANY"`, eliminating the JSON parsing fragility in the current codebase
2. **Discrete action space dramatically outperforms continuous** — RT-2's binned actions, PIVOT's arrow selection, NavGPT-2's candidate selection, and Social Nav's discrete commands all confirm that non-fine-tuned VLMs produce more reliable outputs when choosing from enumerated options rather than generating precise continuous values
3. **Chain-of-thought reasoning is mandatory for off-the-shelf VLMs** — Social Nav (2026) shows reasoning prompts consistently outperform action-only prompts; NavGPT, COME-Robot, and PIVOT all require reasoning before action
4. **7 tools across 3 categories is the right scope** — 4 action tools (move, navigate, rotate, stop), 2 perception tools (observe, check_path), 1 communication tool (report_discovery). This is aligned with COME-Robot's API pattern and VoxPoser's tool separation
5. **The current `voice_mapper.py` prompt has 5 critical gaps** — no reasoning instruction, no camera awareness in brain, continuous velocity output, no exploration strategy, no safety rules
6. **Exploration memory replaces conversation history** — instead of 4-message text windows, the navigation system needs structured JSON tracking rooms, frontiers, dead ends, and action outcomes
7. **Closed-loop feedback is essential** — COME-Robot and SayCan both demonstrate that feeding action results back to the LLM improves decision quality. The proposed format includes success/failure, actual distance moved, and safety override information
8. **The SayCan scoring pattern is applicable** — LLM proposes action, LiDAR-based safety layer scores feasibility, combined score selects final action. This bridges Phase 3 (prompt design) with Phase 4 (safety architecture)
9. **PIVOT's visual annotation approach is high-potential** — annotating candidate directions as numbered arrows on the camera image could further improve spatial reasoning, especially for directional decisions at intersections
10. **Provider abstraction is feasible** — the same 7 tools can be expressed in OpenAI, Anthropic, and Gemini tool formats with only wire-format differences, enabling runtime provider switching

---

## Phase 4: Safety Architecture & Reactive Layer

**Scope**: The LLM is too slow for real-time obstacle avoidance. Design the safety layer:

- **Two-Layer Architecture**: Reactive safety (fast, local) + Cognitive navigation (slow, LLM)
- **LiDAR Safety Guardian**: Always-on obstacle detection that can override LLM commands
- **Speed Limiting**: LLM-commanded velocity capped based on nearest obstacle distance
- **Emergency Stop**: Hardware-level or ROS-level e-stop independent of LLM
- **Collision Prediction**: Using current velocity + LiDAR to predict collisions before they happen
- **Geofencing**: LLM-aware boundaries (don't go down stairs, stay in room, etc.)
- **Timeout/Watchdog**: If LLM doesn't respond within N seconds, stop the robot

**Key Questions**:
- What's the maximum safe speed given LLM decision latency (~1-3 seconds)?
- Should the safety layer modify LLM commands (reduce speed) or reject them (full stop)?
- How to inform the LLM when its command was overridden by the safety layer?
- Can the safety layer pre-filter the action space before sending to the LLM?

**Status**: ✅ Complete

### Findings

#### 4.1 The Fundamental Safety Principle

**The safety layer is not a feature of the LLM — it is an independent, always-on system that the LLM cannot bypass.** The LLM proposes, the safety layer disposes. This is the subsumption principle (Brooks, 1986) applied to modern LLM-controlled robotics.

Every major LLM-robotics project — SayCan, RT-2, PaLM-E, NaVid, VoxPoser — uses some form of a two-layer architecture where the LLM never has direct, unmediated access to actuators. No project trusts the LLM for real-time safety.

#### 4.2 Three-Layer Architecture (3T) — The Design Framework

The Three-Layer Architecture (Gat, 1998; Bonasso et al., 1997) is the standard framework for autonomous robotics. Applied to LLM-controlled navigation:

| Layer | Name | Frequency | Latency | Robot Mapping |
|-------|------|-----------|---------|---------------|
| **Deliberative** | LLM Reasoner | 0.2–0.5 Hz | 1–5s | Cloud VLM — chooses goals, exploration strategy, interprets scenes |
| **Sequencer** | Executive / Coordinator | 1–10 Hz | 10–100ms | `SafetyExecutor` — validates LLM commands, manages timeouts, tracks blocked actions, handles degradation |
| **Controller** | Reactive / Behavioral | 10–100 Hz | 1–10ms | Nav2 Controller (20 Hz) + Collision Monitor + LiDAR emergency stop |

**Key insight**: The current `voice_mapper.py` combines all three layers in a single monolithic class. The LLM-controlled architecture requires separating them so the Sequencer and Controller can operate independently of LLM availability.

##### Mapping to Existing Infrastructure

```
┌─────────────────────────────────────────────────────────────┐
│              DELIBERATIVE LAYER (0.2–0.5 Hz)                │
│                                                              │
│   Cloud VLM (GPT-4o / Claude / Gemini)                      │
│   ├── Receives: annotated camera + LiDAR text + state       │
│   ├── Outputs: tool calls (move_toward, navigate_to, etc.)  │
│   └── Context: exploration memory, room tracking, frontiers  │
└────────────────────────┬────────────────────────────────────┘
                         │ tool_call JSON
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              SEQUENCER LAYER (1–10 Hz)                       │
│                                                              │
│   SafetyExecutor (new component)                             │
│   ├── Pre-validates LLM commands against LiDAR state         │
│   ├── Computes affordance scores (SayCan pattern)            │
│   ├── Enforces blocked-action memory (retry suppression)     │
│   ├── Manages LLM watchdog / timeout / degradation tiers     │
│   ├── Translates tool calls → Nav2 goals or cmd_vel          │
│   ├── Records execution results for LLM feedback             │
│   └── Checks geofence / keepout zone compliance              │
└────────────────────────┬────────────────────────────────────┘
                         │ cmd_vel or Nav2 goal
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              CONTROLLER LAYER (10–100 Hz)                    │
│                                                              │
│   Already implemented in the existing system:                │
│   ├── Nav2 Regulated Pure Pursuit (20 Hz)                    │
│   │   └── use_collision_detection: true                      │
│   │   └── max_allowed_time_to_collision_up_to_carrot: 1.0    │
│   ├── Nav2 Velocity Smoother (20 Hz)                         │
│   │   └── max_velocity: [0.15, 0.0, 0.5]                    │
│   │   └── max_accel: [0.5, 0.0, 1.0]                        │
│   ├── Nav2 Collision Monitor (continuous)                    │
│   │   └── FootprintApproach, time_before_collision: 1.2s     │
│   ├── LiDAR scan_callback (10 Hz)                            │
│   │   └── 6-sector obstacle distances                        │
│   │   └── ±30° emergency stop zone                           │
│   └── voice_mapper.move() obstacle avoidance (20 Hz loop)    │
│       └── Emergency stop at 0.3m                             │
│       └── Speed reduction at 0.5–1.0m                        │
│       └── Full stop at 0.5m                                  │
└─────────────────────────────────────────────────────────────┘
```

##### Simplex Architecture for Layer Switching

The Simplex Architecture (Sha, 2001) provides a formal framework for switching between the LLM (advanced controller) and a proven-safe local controller:

```
LLM Decision ──→ Decision Module ──→ cmd_vel_out
                      ↑
Safety Controller ────┘
(Nav2 Collision Monitor +
 local obstacle avoidance)

Decision Module logic:
  IF safety_envelope.is_safe(llm_cmd_vel):
    forward(llm_cmd_vel)
  ELSE:
    forward(safety_controller.compute())
    notify LLM of override
```

This maps directly to what Nav2's Collision Monitor already does — it sits between the controller's `cmd_vel_smoothed` output and the final `cmd_vel` sent to the base (configured in `nav2_params.yaml` lines 367–368).

#### 4.3 Existing Safety Inventory

The current codebase already contains four safety layers, documented in research doc 006:

| Layer | Component | Trigger | Action | Frequency | Files |
|-------|-----------|---------|--------|-----------|-------|
| **L0** | Emergency stop | `front_wide < 0.3m` | Publish zero Twist 5x, backup | 10 Hz (LiDAR callback) | `voice_mapper.py:1078-1083` |
| **L1** | `move()` obstacle avoidance | `min_front < 0.5m` | Stop movement, proportional slowdown in 0.5–1.0m zone | 20 Hz (move loop) | `voice_mapper.py:2050-2074` |
| **L2** | Nav2 costmap + RPP controller | Obstacle in costmap (inflation 0.35m) | Path replanning, velocity regulation | 20 Hz (controller) | `nav2_params.yaml:132-155` |
| **L3** | Nav2 Collision Monitor | `time_before_collision < 1.2s` | Proportional velocity reduction (approach) | Continuous (per cmd_vel) | `nav2_params.yaml:362-397` |

Additionally:
- **Sensor monitor** (`voice_mapper.py:2722-2768`): Checks LiDAR, odom, camera every 2s; stops exploration if LiDAR is lost
- **Ackerman constraint**: `min_turn_radius = 0.3`, cannot spin in place — all turns require forward motion
- **Velocity smoother**: `max_velocity: [0.15, 0.0, 0.5]`, `max_accel: [0.5, 0.0, 1.0]` — hardware limits

**What's MISSING for LLM-controlled navigation** (the Sequencer layer):

| Component | Status | Why Needed |
|-----------|--------|------------|
| LLM command pre-validation | **Missing** | Validate against LiDAR before executing LLM-chosen direction |
| Affordance scoring | **Missing** | Pre-filter infeasible actions before LLM sees them |
| Blocked-action memory | **Missing** | Prevent LLM from repeatedly trying same blocked direction |
| Watchdog / timeout | **Missing** | Handle LLM non-response (network outage, API error) |
| Graceful degradation | **Missing** | Fall back to local navigation when LLM unavailable |
| Override feedback | **Missing** | Inform LLM when its command was modified by safety layer |
| Geofencing | **Missing** | Virtual no-go zones the LLM can define or obey |
| Cliff detection | **Missing** | No stair/drop protection (2D LiDAR can't see down) |

#### 4.4 LiDAR Safety Guardian — Collision Prediction

##### Time-to-Collision (TTC) from LiDAR

TTC is the primary collision prediction metric. For each LiDAR beam at angle `θ_i` with range `r_i`:

```
v_closing = v_x · cos(θ_i) + v_y · sin(θ_i)     [robot velocity projected onto beam direction]

if v_closing > 0:   TTC_i = r_i / v_closing       [moving toward obstacle]
else:                TTC_i = ∞                     [moving away, no collision risk]

min_TTC = min(TTC_i for all beams)
```

This is exactly what Nav2's Collision Monitor implements internally — it forward-projects the robot's footprint polygon along the current velocity trajectory in `simulation_time_step` (0.1s) increments and checks if any sensor points fall inside the projected footprint.

##### Custom TTC Implementation (for Sequencer layer)

```python
def compute_min_ttc(scan_ranges, scan_angle_min, scan_angle_inc,
                     vx: float, omega: float) -> float:
    """Compute minimum time-to-collision from LiDAR + current velocity.

    Used by Sequencer to pre-validate LLM commands before execution.
    Not a replacement for Collision Monitor — a planning-time check.
    """
    min_ttc = float('inf')
    for i, r in enumerate(scan_ranges):
        if r < 0.05 or r > 12.0 or math.isinf(r):
            continue
        angle = scan_angle_min + i * scan_angle_inc
        v_closing = vx * math.cos(angle)  # Ackerman: vy ≈ 0
        if v_closing > 0.01:
            ttc = r / v_closing
            min_ttc = min(min_ttc, ttc)
    return min_ttc
```

**Decision thresholds** (informed by Nav2 defaults and ISO practice):

| min_TTC | Action | Rationale |
|---------|--------|-----------|
| > 5.0s | Full speed allowed | Ample time to react |
| 2.0–5.0s | Medium speed | Within planning horizon |
| 1.2–2.0s | Slow speed | Collision Monitor deceleration zone |
| 0.5–1.2s | Creep / safety override | Imminent collision zone |
| < 0.5s | Emergency stop | Must stop immediately |

##### Velocity Obstacles (for Dynamic Environments)

The Velocity Obstacles algorithm (Fiorini & Shiller, 1998) computes the set of velocities that would lead to collision within time horizon τ. For each obstacle, it defines a cone in velocity space — any chosen velocity must be outside all cones.

```
Safe_velocity_set = All_velocities \ Union(VO_1, VO_2, ..., VO_n)

If LLM requests velocity v:
  if v ∈ Safe_velocity_set:    execute v
  else:                        find nearest safe v' → execute v' → report modification
```

**Applicability**: VO is primarily relevant for dynamic obstacles (people walking). At 0.15 m/s max speed with static obstacles, simple TTC is sufficient. VO becomes important if the robot operates in spaces with foot traffic.

##### DWA Admissibility Check

The admissible velocity concept from DWA (Fox, 1997) provides a physics-based safety guarantee:

```
v_admissible ≤ √(2 · a_max · d_obstacle)
```

With current robot parameters (`a_max = 0.5 m/s²`, `v_max = 0.15 m/s`):
- Stopping distance at max speed: `0.15² / (2 × 0.5) = 0.0225m` (2.25 cm)
- The robot stops almost instantly — braking distance is negligible at these speeds

#### 4.5 Speed Limiting — Physics-Based Analysis

##### Braking Distance at Robot Speeds

For the ROSMASTER A1 (~3 kg, rubber wheels, indoor floors):

| Speed | Reaction Distance (200ms) | Braking Distance | Total Stopping Distance |
|-------|--------------------------|------------------|------------------------|
| 0.08 m/s (slow) | 16 mm | ~1 mm | **17 mm** |
| 0.12 m/s (medium) | 24 mm | ~2 mm | **26 mm** |
| 0.18 m/s (fast) | 36 mm | ~3 mm | **39 mm** |

Physics: `KE = ½mv² = ½(3)(0.18²) = 0.049 J`. Friction force = `μmg = 0.5(3)(9.81) = 14.7 N`. Braking distance = `KE/F = 0.003m` (3 mm). At these speeds, braking is essentially instantaneous — the dominant factor is **reaction time** (sensor latency + processing + motor response ≈ 200ms at 10 Hz LiDAR).

##### Speed Safety with vs. without Reactive Layer

| Scenario | With 10 Hz Reactive Layer | Without Reactive Layer (LLM-only) |
|----------|--------------------------|-----------------------------------|
| Max safe speed | **0.18–0.20 m/s** | **~0.06 m/s** |
| Stopping distance from detection | ~4 cm | 18–54 cm (1–3s LLM latency) |
| Safety guarantee | Near-certain | Probabilistic (depends on LLM speed) |

**Critical conclusion**: The LLM decision latency (1–3s) is irrelevant for obstacle avoidance **as long as the 10 Hz reactive layer is always active**. Without it, the robot must creep at 0.06 m/s to be safe.

##### Proposed Speed Tiers for LLM Navigation

| Tier | Speed | Condition | LiDAR Clearance |
|------|-------|-----------|-----------------|
| `stop` | 0.0 m/s | Emergency or safety override | < 0.3m |
| `creep` | 0.05 m/s | Very tight spaces, doorways | 0.3–0.5m |
| `slow` | 0.08 m/s | Near obstacles, cautious approach | 0.5–1.0m |
| `medium` | 0.12 m/s | Normal indoor navigation | 1.0–2.0m |
| `fast` | 0.18 m/s | Open corridors, clear path | > 2.0m |

These match the existing `voice_mapper.py` speed parameters (`linear_speed = 0.12`, `slow_speed = 0.06`, `emergency_dist = 0.3`, `min_obstacle_dist = 0.5`).

##### Proportional Speed Scaling Formula

The existing `move()` function already implements proportional scaling (`voice_mapper.py:2067-2074`). The formalized version:

```
v_safe = v_max × clamp((d_obstacle - d_stop) / (d_full - d_stop), 0.0, 1.0)

where:
  d_stop = 0.3m   (emergency stop distance)
  d_full = 1.0m   (full speed allowed beyond this)
  v_max  = 0.18 m/s
```

This matches Nav2's Regulated Pure Pursuit behavior with `approach_velocity_scaling_dist: 0.8` and `use_regulated_linear_velocity_scaling: true`.

##### ISO Standards Reference

- **ISO 13482:2014** (Personal care robots): Requires a "protective stop" capability. For robots this size (~3 kg at 0.15 m/s), kinetic energy is 0.03–0.07 J — far below harm thresholds. No prescriptive speed limits; derived from risk assessment.
- **ISO 13849-1** (Safety control systems): Performance Level a (lowest) is appropriate for a research/hobby robot. Category B control — safety relies on correct design.
- **ISO/TS 15066** (Collaborative robots): Max contact force thresholds (140N quasi-static, 280N transient) are orders of magnitude above what this robot can produce.

#### 4.6 Emergency Stop Architecture

##### Current Implementation

The existing e-stop (`voice_mapper.py:1943-1948`) publishes zero Twist 5 times at 50 Hz:

```python
def emergency_stop(self):
    twist = Twist()
    for _ in range(5):
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.02)
```

This is a software e-stop. There is no hardware e-stop (physical button or relay).

##### Proposed E-Stop Hierarchy for LLM Mode

| Priority | Trigger | Action | Latency |
|----------|---------|--------|---------|
| **P0** | Hardware e-stop button (future) | Cut motor power directly | < 1 ms |
| **P1** | LiDAR emergency (`front_wide < 0.3m`) | Software e-stop + backup | 100 ms |
| **P2** | Collision Monitor (`TTC < 0.5s`) | Proportional decel to zero | ~100 ms |
| **P3** | Sensor loss (LiDAR timeout > 5s) | Stop exploration, halt | 5 s |
| **P4** | LLM timeout (no response > 5s) | Tiered degradation | 5 s |
| **P5** | Geofence violation | Block command, notify LLM | ~10 ms |

Key principle: **Lower-priority layers cannot override higher-priority layers.** The LLM (P4/P5) can never suppress an emergency stop (P1).

#### 4.7 LLM Command Override — Feedback Patterns

##### Two Approaches: Pre-Filter vs. Post-Filter

**Approach A: Pre-filter (remove unsafe options before LLM call)**
- Check each direction with LiDAR before sending context to LLM
- If "forward" is blocked (< 0.5m), include in context as blocked
- LLM sees feasibility information but all tools remain available
- **Pro**: LLM makes better strategic decisions with full awareness
- **Con**: LLM may still propose blocked actions (tool calling doesn't enforce soft constraints)

**Approach B: Post-filter (validate after LLM decides)**
- Let LLM propose any action with full context
- Safety layer validates proposal: execute, modify (reduce speed), or reject (stop)
- Feed override result back to LLM on next cycle
- **Pro**: LLM has full situational awareness
- **Con**: Wastes one decision cycle when LLM proposes something unsafe

**Recommendation: Hybrid approach** — include affordance scores in context (pre-filter information) AND post-filter with override feedback. This is the SayCan scoring pattern adapted for navigation.

##### Affordance Scoring (SayCan Pattern for Navigation)

Adapting SayCan's `score(skill) = P_LLM(skill|task) × V(skill|state)`:

```python
def compute_affordance_scores(obstacle_distances: dict) -> dict:
    """Compute feasibility scores for each direction based on LiDAR state.

    Included in the LLM context so the LLM prefers feasible actions.
    Not a hard constraint — the post-filter safety layer enforces actual limits.
    """
    def clearance_score(dist, threshold_blocked=0.5, threshold_clear=2.0):
        if dist <= threshold_blocked: return 0.1
        if dist >= threshold_clear:   return 1.0
        return 0.1 + 0.9 * (dist - threshold_blocked) / (threshold_clear - threshold_blocked)

    return {
        "forward":       clearance_score(obstacle_distances.get("front_wide", 10)),
        "forward_left":  clearance_score(obstacle_distances.get("front_left", 10)),
        "forward_right": clearance_score(obstacle_distances.get("front_right", 10)),
        "left":          clearance_score(obstacle_distances.get("left", 10)),
        "right":         clearance_score(obstacle_distances.get("right", 10)),
        "backward":      0.5,   # Almost always possible
        "stop":          1.0,   # Always possible
    }
```

Injected into the LLM context as:
```
ACTION FEASIBILITY (1.0 = clear, 0.1 = blocked):
  forward: 0.1  |  forward_left: 0.7  |  forward_right: 0.9
  left: 1.0     |  right: 0.3         |  backward: 0.5
  → Prefer actions with high scores. Low scores will likely be overridden.
```

##### Override Feedback Format

When the safety layer modifies or blocks an LLM command (already proposed in Phase 3, section 3.17):

```json
{
  "success": false,
  "actual_distance_m": 0.12,
  "stopped_early": true,
  "stopped_reason": "obstacle_at_0.3m_ahead",
  "safety_override": true,
  "safety_message": "Movement stopped: obstacle detected 0.3m ahead in forward direction. Consider turning or choosing a different direction.",
  "original_command": {"tool": "move_toward", "direction": "forward", "speed": "medium"},
  "executed_command": {"tool": "move_toward", "direction": "forward", "speed": "slow", "actual_duration": 0.8}
}
```

##### Prompt Rule for Overrides

From Phase 3 section 4 (safety rules), extended:
```
If your action was safety-overridden:
1. DO NOT retry the same direction immediately.
2. Read the safety_message and obstacle distances in your context.
3. Choose a DIFFERENT direction or call check_path_clear to scout alternatives.
4. If the same direction is overridden 3+ times, treat it as a dead end.
```

##### Blocked-Action Memory (Retry Suppression)

The Sequencer tracks repeatedly-blocked actions to prevent the LLM from wasting decision cycles:

```python
class BlockedActionMemory:
    """Tracks actions that were blocked by the safety layer.

    Prevents the LLM from retrying the same blocked action repeatedly.
    Actions are "forgotten" after a cooldown period (environment may change).
    """
    def __init__(self, max_retries=2, cooldown_seconds=15):
        self.blocked = {}  # action_hash → (count, last_blocked_time)
        self.max_retries = max_retries
        self.cooldown = cooldown_seconds

    def should_allow(self, action: dict) -> tuple:
        key = self._hash(action)
        if key not in self.blocked:
            return True, ""
        count, last_time = self.blocked[key]
        if time.time() - last_time > self.cooldown:
            del self.blocked[key]
            return True, ""
        if count >= self.max_retries:
            return False, f"Blocked {count}x in {self.cooldown}s. Try a different approach."
        return True, f"Warning: blocked {count}x recently."

    def record_blocked(self, action: dict):
        key = self._hash(action)
        count = self.blocked.get(key, (0, 0))[0]
        self.blocked[key] = (count + 1, time.time())

    def _hash(self, action: dict) -> str:
        # Hash on tool name + direction (not speed — speed changes don't affect blockage)
        return f"{action.get('tool')}:{action.get('direction', action.get('heading_deg', ''))}"
```

When an action is suppressed by the blocked-action memory (before it even reaches the safety layer), the feedback to the LLM includes:
```json
{
  "success": false,
  "stopped_reason": "action_suppressed",
  "safety_message": "This direction was blocked 3 times in the last 15 seconds. The obstacle has not moved. Please choose a completely different direction or strategy."
}
```

#### 4.8 Watchdog / Timeout — Graceful Degradation

##### The Problem

If the LLM API call takes too long (network issues, rate limiting, API outage) or fails entirely, the robot must not:
- Continue moving blindly at its last commanded velocity
- Stop indefinitely in an unsafe location (doorway, hallway)
- Panic and make erratic movements

##### Tiered Degradation Strategy

```python
class LLMWatchdog:
    """Manages robot behavior when LLM is delayed or unavailable.

    Four tiers of graceful degradation, inspired by autonomous vehicle
    "minimal risk condition" (MRC) patterns.
    """
    TIER_1_TIMEOUT = 3.0    # LLM slightly delayed
    TIER_2_TIMEOUT = 10.0   # LLM significantly delayed
    TIER_3_TIMEOUT = 30.0   # LLM appears down
    TIER_4_TIMEOUT = 120.0  # Extended outage

    def evaluate(self, elapsed_since_last_response: float) -> str:
        if elapsed < self.TIER_1_TIMEOUT:
            return "CONTINUE"     # Keep executing last command at reduced speed
        elif elapsed < self.TIER_2_TIMEOUT:
            return "STOP_WAIT"    # Stop, wait for LLM, announce delay
        elif elapsed < self.TIER_3_TIMEOUT:
            return "LOCAL_NAV"    # Fall back to frontier exploration
        else:
            return "RETURN_HOME"  # Navigate to starting position
```

| Tier | Timeout | Robot Behavior | User Notification |
|------|---------|----------------|-------------------|
| **Tier 1** | 0–3s | Continue at 50% speed, maintain heading | None (normal variation) |
| **Tier 2** | 3–10s | Stop movement, wait for response | "I'm thinking... please wait." |
| **Tier 3** | 10–30s | Fall back to local frontier exploration | "Connection slow. Switching to autonomous exploration." |
| **Tier 4** | 30–120s | Navigate to start position and stop | "Extended connection loss. Returning to start." |

**Exponential backoff is wrong for robot control**: It's designed for retrying API calls, not commanding physical systems. A robot can't wait 2, 4, 8, 16 seconds between decisions while potentially in an unsafe position. Use fixed timeouts with tiered degradation.

##### Velocity Decay During Tier 1

When the LLM is slightly delayed (0–3s), the robot can continue its last command but with exponential velocity decay:

```python
def decay_velocity(self, elapsed: float, last_cmd: Twist) -> Twist:
    """Exponentially decay velocity as LLM response is delayed."""
    decay_factor = math.exp(-elapsed / 2.0)  # τ = 2 seconds
    result = Twist()
    result.linear.x = last_cmd.linear.x * decay_factor
    result.angular.z = last_cmd.angular.z * decay_factor
    if abs(result.linear.x) < 0.01:
        result.linear.x = 0.0
    return result
```

At 2s delay: velocity is 37% of original. At 3s: 22%. At 5s: 8%.

##### Tier 3 Fallback: Existing Frontier Exploration

The existing `exploration_loop()` in `voice_mapper.py` with `nav2-frontier` or `reactive` mode is the perfect Tier 3 fallback. It already handles:
- Frontier detection from the SLAM map
- Local obstacle avoidance via Nav2
- Doorway detection and traversal
- Stuck detection and recovery

The Watchdog simply calls `start_exploration()` with the reactive mode when the LLM is unavailable for >10 seconds.

##### Configuration Parameters

| Parameter | Recommended Value | Rationale |
|-----------|-----------------|-----------|
| LLM API call timeout | 8 seconds | Most calls complete in 1–3s; 8s catches slow but valid responses |
| Decision freshness | 5 seconds | A navigation decision older than 5s should be re-evaluated |
| Heartbeat interval | 1 second | Check LLM availability every second |
| Full fallback trigger | 30 seconds | Switch to fully autonomous mode after 30s |
| Return-home trigger | 120 seconds | Navigate home after 2 minutes without LLM |

#### 4.9 Geofencing — Virtual Boundaries

##### Nav2 Keepout Zones (Costmap Filters)

Nav2 provides costmap filter plugins for no-go zones:

```yaml
# Filter mask info server
costmap_filter_info_server:
  ros__parameters:
    type: 0              # 0 = keepout filter
    filter_info_topic: "costmap_filter_info"
    mask_topic: "filter_mask"

# Add to both local and global costmap:
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer", "keepout_filter"]
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: True
        filter_info_topic: "costmap_filter_info"
```

The filter mask is a grayscale image aligned to the map where white pixels (254) = lethal/keepout and black (0) = free space. Nav2 prevents path planning through keepout pixels.

##### LLM-Defined Virtual Boundaries

The LLM can define keepout zones by name ("don't go into the bathroom"), translated by the Sequencer into costmap filter updates:

```python
class GeofenceManager:
    """Virtual boundaries the LLM can define and the safety layer enforces."""

    def __init__(self):
        self.zones = {}  # name → {polygon: [(x,y),...], type: "keepout"|"slowdown"}

    def add_zone(self, name: str, polygon: list, zone_type: str = "keepout"):
        self.zones[name] = {"polygon": polygon, "type": zone_type}
        self._update_costmap_filter()

    def validate_nav_goal(self, goal_x: float, goal_y: float) -> tuple:
        for name, zone in self.zones.items():
            if zone["type"] == "keepout" and point_in_polygon(goal_x, goal_y, zone["polygon"]):
                return False, f"Goal is inside keepout zone '{name}'"
        return True, ""
```

This requires a new LLM tool:
```json
{
  "name": "define_zone",
  "description": "Define a virtual boundary zone on the map",
  "parameters": {
    "name": "string — human-readable name (e.g., 'stairs_area')",
    "zone_type": "enum ['keepout', 'slowdown']",
    "significance": "string — why this zone exists"
  }
}
```

The zone's polygon would be derived from the robot's current position + observed landmarks, not from LLM-generated coordinates (the LLM doesn't have precise map coordinates).

##### Stair/Drop Detection

The 2D LiDAR (SLLidar C1, horizontal mount) **cannot detect stairs or drops**. Options:

| Method | Reliability | Cost | Latency | Recommendation |
|--------|-------------|------|---------|----------------|
| **IR cliff sensors** (hardware) | Very high | $2–5 each | < 1 ms | **Best option** — industry standard (Roomba, etc.) |
| **OAK-D depth camera** (software) | Moderate | $0 (existing hardware) | ~50 ms | Supplement, not primary — requires downward camera tilt |
| **LiDAR floor assumption** (software) | Low | $0 | 100 ms | Unreliable with horizontal 2D LiDAR |

**Recommendation**: Add 2–3 IR cliff sensors to the front bumper, wired to Jetson GPIO. Publish as `sensor_msgs/Range` topic and feed to Collision Monitor as an observation source. This is the established solution across the entire consumer robotics industry.

#### 4.10 Expanded Collision Monitor Configuration

The current `nav2_params.yaml` only configures `FootprintApproach`. For LLM-controlled navigation, add Stop and Slowdown zones:

```yaml
collision_monitor:
  ros__parameters:
    polygons: ["EmergencyStop", "SlowdownZone", "FootprintApproach"]

    EmergencyStop:
      type: "circle"
      action_type: "stop"
      radius: 0.25           # Hard stop if anything within 25 cm
      min_points: 3           # 3 LiDAR points (avoids false positives from noise)
      visualize: True
      enabled: True

    SlowdownZone:
      type: "circle"
      action_type: "slowdown"
      radius: 0.50            # Slow to 40% if anything within 50 cm
      slowdown_ratio: 0.4
      min_points: 4
      visualize: True
      enabled: True

    FootprintApproach:
      type: "polygon"
      action_type: "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 2.0  # Increased from 1.2s for LLM latency margin
      simulation_time_step: 0.1
      min_points: 6
      visualize: True
      enabled: True

    observation_sources: ["scan", "pointcloud"]
    scan:
      type: "scan"
      topic: "/scan"
      min_height: 0.05
      max_height: 0.5
      enabled: True
    pointcloud:
      type: "pointcloud"
      topic: "/oak/points"
      min_height: 0.05
      max_height: 1.0
      enabled: True
```

This gives three concentric safety zones:
1. **0–25 cm**: Emergency stop (highest priority)
2. **25–50 cm**: Slowdown to 40% of commanded velocity
3. **50 cm – TTC 2.0s**: Proportional approach deceleration

#### 4.11 Complete SafetyExecutor — Sequencer Layer Design

Bringing together all Phase 4 findings, the SafetyExecutor is the new component that sits between the LLM and the robot's actuators:

```python
class SafetyExecutor:
    """Sequencer layer: validates, modifies, and executes LLM navigation commands.

    Responsibilities:
    1. Pre-validate LLM commands against current LiDAR state
    2. Compute and report affordance scores to LLM
    3. Enforce blocked-action memory (retry suppression)
    4. Manage LLM watchdog / timeout / degradation
    5. Execute validated commands via Nav2 or cmd_vel
    6. Record execution results for LLM feedback
    7. Check geofence compliance
    """

    def __init__(self):
        self.blocked_memory = BlockedActionMemory(max_retries=2, cooldown_seconds=15)
        self.watchdog = LLMWatchdog()
        self.geofence = GeofenceManager()
        self.last_llm_response_time = time.time()
        self.last_command = None
        self.execution_result = None

    def execute_llm_command(self, tool_call: dict) -> dict:
        """Main entry point: validate and execute an LLM tool call.

        Returns a result dict to be included in the next LLM prompt.
        """
        # 1. Check blocked-action memory
        allowed, warning = self.blocked_memory.should_allow(tool_call)
        if not allowed:
            return {"success": False, "stopped_reason": "action_suppressed",
                    "safety_message": warning}

        # 2. Pre-validate against LiDAR
        valid, reason = self._pre_validate(tool_call)
        if not valid:
            self.blocked_memory.record_blocked(tool_call)
            return {"success": False, "stopped_reason": reason,
                    "safety_override": True,
                    "safety_message": f"Command rejected before execution: {reason}"}

        # 3. Check geofence (for navigate_to_goal)
        if tool_call.get("tool") == "navigate_to_goal":
            geo_ok, geo_msg = self.geofence.validate_nav_goal(
                tool_call["x"], tool_call["y"])
            if not geo_ok:
                return {"success": False, "stopped_reason": "geofence_violation",
                        "safety_message": geo_msg}

        # 4. Execute (with Collision Monitor + L0-L3 running underneath)
        result = self._execute(tool_call)

        # 5. Record if blocked
        if result.get("safety_override"):
            self.blocked_memory.record_blocked(tool_call)

        # 6. Update watchdog timestamp
        self.last_llm_response_time = time.time()

        return result

    def get_affordance_context(self) -> str:
        """Generate affordance score text for LLM context injection."""
        scores = compute_affordance_scores(self.obstacle_distances)
        lines = ["ACTION FEASIBILITY (1.0=clear, 0.1=blocked):"]
        for direction, score in scores.items():
            lines.append(f"  {direction}: {score:.1f}")
        lines.append("  → Prefer high-score directions.")
        return "\n".join(lines)

    def watchdog_tick(self):
        """Called at 1 Hz by the main loop. Handles LLM timeout degradation."""
        elapsed = time.time() - self.last_llm_response_time
        tier = self.watchdog.evaluate(elapsed)

        if tier == "CONTINUE":
            pass  # Normal operation
        elif tier == "STOP_WAIT":
            self.publish_cmd_vel(Twist())
            self.speak("I'm thinking... please wait.")
        elif tier == "LOCAL_NAV":
            self.speak("Connection slow. Switching to autonomous mode.")
            self.start_local_exploration()
        elif tier == "RETURN_HOME":
            self.speak("Connection lost. Returning to start.")
            self.navigate_to_start()
```

#### 4.12 Safety Data Flow — Complete Picture

```
     ┌───────────────────────────────────────────────────┐
     │  Cloud VLM (Deliberative Layer, 0.2–0.5 Hz)      │
     │  Receives: camera + LiDAR summary + affordance   │
     │  scores + last action result + exploration memory  │
     │  Returns: tool_call (move/navigate/rotate/stop)   │
     └───────────────────┬───────────────────────────────┘
                         │
                         ▼
     ┌───────────────────────────────────────────────────┐
     │  SafetyExecutor (Sequencer Layer, 1–10 Hz)       │
     │                                                    │
     │  ┌─ 1. Blocked-Action Memory ─ suppress retries   │
     │  ├─ 2. Pre-Validate (LiDAR check) ─ reject unsafe │
     │  ├─ 3. Geofence Check ─ reject keepout violations  │
     │  ├─ 4. Speed Limit ─ cap based on proximity        │
     │  ├─ 5. Execute ─ publish cmd_vel or Nav2 goal      │
     │  ├─ 6. Monitor ─ watch for safety overrides        │
     │  ├─ 7. Record ─ save result for LLM feedback       │
     │  └─ 8. Watchdog ─ handle LLM timeout/degradation   │
     └───────────────────┬───────────────────────────────┘
                         │ cmd_vel_raw
                         ▼
     ┌───────────────────────────────────────────────────┐
     │  Nav2 Stack (Controller Layer, 10–100 Hz)         │
     │                                                    │
     │  ├─ RPP Controller (20 Hz) ─ follow path           │
     │  ├─ Velocity Smoother (20 Hz) ─ enforce accel      │
     │  ├─ Collision Monitor (per msg) ─ 3-zone safety    │
     │  │   ├─ EmergencyStop: r < 0.25m → zero velocity   │
     │  │   ├─ SlowdownZone: r < 0.50m → 40% velocity     │
     │  │   └─ FootprintApproach: TTC < 2.0s → decel       │
     │  └─ LiDAR Emergency (10 Hz) ─ front_wide < 0.3m   │
     └───────────────────┬───────────────────────────────┘
                         │ cmd_vel (safe)
                         ▼
                    ┌──────────┐
                    │  Motors   │
                    └──────────┘
```

#### 4.13 Key Findings Summary

1. **Three-layer architecture (3T) is the correct framework** — Deliberative (LLM), Sequencer (SafetyExecutor), Controller (Nav2 + Collision Monitor). The current monolithic `voice_mapper.py` needs a new Sequencer layer, not changes to the existing Controller layer.

2. **The existing safety infrastructure is almost sufficient** — four layers of obstacle avoidance already exist (L0–L3). The gap is a Sequencer that interprets LLM commands, validates them, handles timeouts, and reports overrides.

3. **SayCan-style affordance scoring is the best pre-filter pattern** — compute LiDAR-based direction feasibility scores and include them in the LLM context. The LLM naturally prefers high-scoring (safe) directions without being hard-constrained.

4. **Speed safety is a non-issue at current speeds** — at 0.18 m/s max with a 10 Hz reactive layer, total stopping distance is ~4 cm. The braking physics are negligible; reaction time dominates. The current speed tiers (0.06/0.12/0.18 m/s) are well-justified.

5. **LLM latency doesn't affect safety — the reactive layer does** — the 1–3s LLM decision time is irrelevant because the Collision Monitor + LiDAR emergency stop run continuously. Without the reactive layer, max safe speed would be ~0.06 m/s.

6. **Tiered degradation handles LLM unavailability** — stop at 3s, local nav at 10s, return home at 30s. Exponential backoff is wrong for robot control. The existing frontier exploration serves as a proven Tier 3 fallback.

7. **Blocked-action memory prevents retry loops** — hash(direction+tool) with 15-second cooldown and max 2 retries. Inform the LLM explicitly when its action is suppressed.

8. **Collision Monitor needs expansion** — add EmergencyStop (0.25m, hard stop) and SlowdownZone (0.50m, 40%) in addition to existing FootprintApproach. Config-only change, no code needed.

9. **Override feedback must be structured and actionable** — include original vs. executed command, reason, current sensor state, and suggested alternatives. The LLM prompt must instruct "do NOT retry overridden directions."

10. **Geofencing is possible via Nav2 keepout filters** — costmap filter plugin with LLM-definable zones. Cliff/stair detection requires hardware (IR cliff sensors, $2–5 each).

11. **Nav2 Collision Monitor's approach algorithm** is the ideal outer safety wrapper — it forward-projects the robot's footprint along the current velocity trajectory and proportionally decelerates, rather than hard-stopping. This provides smooth, safe deceleration that doesn't alarm nearby humans.

12. **No off-the-shelf "LLM safety wrapper for ROS2" exists** — this would need to be built as the SafetyExecutor component. The closest is the Collision Monitor (cmd_vel filtering) + Nav2 behavior trees (recovery), but neither understands LLM-specific patterns like retry suppression or affordance scoring.

---

## Phase 5: Latency, Loop Timing & Streaming Strategies

**Scope**: Analyze timing constraints and optimization strategies:

- **Decision Loop Budget**: perception → API call → parse → execute → repeat
- **API Latency**: GPT-4o vision (~2-5s), Claude Sonnet (~1-3s), Claude Haiku (~0.5-1s)
- **Streaming**: Can the LLM stream partial decisions while still "thinking"?
- **Local Models**: LLaVA, CogVLM, Phi-3-Vision on Jetson Orin Nano feasibility
- **Batching**: Process multiple sensor frames while waiting for LLM response
- **Predictive Motion**: Continue moving in predicted direction while waiting for next decision
- **Token Economy**: Cost per decision at various calling frequencies

**Key Questions**:
- What decision frequency is needed? (1 Hz? 0.5 Hz? 0.2 Hz?)
- Can we use a fast local model for routine navigation and cloud LLM for complex decisions?
- What's the token cost per hour of exploration at different frequencies?
- How to handle network outages gracefully?

**Status**: ✅ Complete

### Findings

#### 5.1 Decision Loop Architecture: Anatomy of One Cycle

The proposed LLM navigation replaces the current `exploration_loop()` (line 2338) with an LLM decision cycle. Here is the complete timing breakdown for one cycle, measured against the existing codebase:

##### Current Exploration Loop Timing (Baseline)

```
exploration_loop() — runs on dedicated thread
  ├── choose_frontier()         ~2-10ms   (NumPy vectorized map analysis)
  ├── navigate_to(fx, fy)      ~50-100ms (send Nav2 goal, return immediately)
  ├── _observe_in_background()  ~2-5s     (GPT-4o vision call, fires every 15s)
  ├── sleep(0.5)               500ms     (poll interval)
  └── stuck detection           ~1ms      (time comparison)

  Total cycle: ~500ms (dominated by sleep)
  Frontier decisions: ~2 Hz (every 0.5s)
  LLM involvement: 0.067 Hz (every 15s, observation only, discarded)
```

##### Proposed LLM Decision Loop Timing

```
llm_decision_loop() — replaces exploration_loop()
  ├── PHASE 1: Sensor Fusion (local, parallel)           5-25ms
  │   ├── image_to_base64(latest_image)                   5-15ms
  │   ├── depth_sample(7 points)                          1-3ms
  │   ├── annotate_image(depth + heading + sectors)       2-5ms
  │   ├── lidar_12sector_summary()                        1-2ms
  │   └── build_state_context()                           <1ms
  │
  ├── PHASE 2: Safety Pre-Check (local)                  1-2ms
  │   ├── affordance_scores(12 directions)                <1ms
  │   └── inject_scores_into_prompt()                     <1ms
  │
  ├── PHASE 3: VLM API Call (network + inference)        300-4,000ms ← DOMINANT
  │   ├── Network upload (~150KB payload)                 10-50ms
  │   ├── Queue + inference (model-dependent)             300-3,500ms
  │   └── Response download (~200-500 bytes)              5-20ms
  │
  ├── PHASE 4: Parse + Validate (local)                  1-5ms
  │   ├── JSON parse tool call                            <1ms
  │   ├── safety_layer.validate(decision)                 1-3ms
  │   └── update_exploration_memory()                     <1ms
  │
  ├── PHASE 5: Execute (local → ROS2)                    <1ms
  │   └── publish cmd_vel or send Nav2 goal               <1ms
  │
  └── PHASE 6: Wait for next cycle                       0-4,700ms
      └── sleep(max(0, decision_interval - elapsed))

  Total cycle: decision_interval (1-5s, configurable)
  Decision frequency: 0.2-1.0 Hz
```

**Key insight**: Phase 3 (VLM API call) dominates at 75-95% of total cycle time. All local processing (Phases 1, 2, 4, 5) combined takes only 7-32ms — negligible compared to network + inference latency.

#### 5.2 API Latency Benchmarks by Model (Feb 2026)

Latency data compiled from Artificial Analysis benchmarks, OpenAI community reports, and Anthropic documentation:

##### Time-to-First-Token (TTFT)

| Model | TTFT (median) | TTFT (p95) | Source |
|-------|---------------|------------|--------|
| **Gemini 2.0 Flash** | ~250-300ms | ~500ms | [Artificial Analysis](https://artificialanalysis.ai/models/gemini-2-0-flash/providers) |
| **Gemini 2.5 Flash** | ~250ms | ~450ms | [Artificial Analysis](https://artificialanalysis.ai/models/gemini-2-5-flash/providers) |
| **GPT-4o-mini** | ~300-500ms | ~800ms | [OpenAI Community](https://community.openai.com/t/whats-a-typical-first-token-time-for-gpt-4-when-streaming/619389) |
| **Claude Haiku 4.5** | ~400-600ms | ~1,000ms | [Anthropic Docs](https://platform.claude.com/docs/en/test-and-evaluate/strengthen-guardrails/reduce-latency) |
| **GPT-4o** | ~500-800ms | ~1,500ms | [OpenAI Community](https://community.openai.com/t/whats-a-typical-first-token-time-for-gpt-4-when-streaming/619389) |
| **Claude Sonnet 4.5** | ~800-2,000ms | ~3,000ms | [Anthropic Docs](https://platform.claude.com/docs/en/test-and-evaluate/strengthen-guardrails/reduce-latency) |

##### Total Response Time (for ~100-150 output tokens, with 640×480 image)

| Model | Median Total | p95 Total | Max Practical Frequency |
|-------|-------------|-----------|------------------------|
| **Gemini 2.0 Flash** | ~400-600ms | ~900ms | **~1.5 Hz** |
| **GPT-4o-mini** | ~500-900ms | ~1,400ms | **~1.2 Hz** |
| **Claude Haiku 4.5** | ~600-1,000ms | ~1,800ms | **~1.0 Hz** |
| **GPT-4o** | ~1,000-2,500ms | ~4,000ms | **~0.5 Hz** |
| **Claude Sonnet 4.5** | ~1,500-3,000ms | ~5,000ms | **~0.3 Hz** |

**For this robot**: At 0.15-0.18 m/s max speed, even 0.2 Hz (one decision every 5s) provides adequate coverage: the robot travels at most 0.9m between decisions, well within LiDAR safety range (0.3m emergency stop).

##### Output Speed (tokens/second)

| Model | Output Speed | Time for 100-token response |
|-------|-------------|---------------------------|
| Gemini 2.0 Flash | ~200+ tok/s | ~0.5s |
| GPT-4o-mini | ~120-160 tok/s | ~0.6-0.8s |
| Claude Haiku 4.5 | ~80-120 tok/s | ~0.8-1.2s |
| GPT-4o | ~80-100 tok/s | ~1.0-1.2s |
| Claude Sonnet 4.5 | ~60-80 tok/s | ~1.2-1.7s |

#### 5.3 Streaming Strategies for Partial Decisions

##### How Streaming Works for Tool/Function Calls

All three major providers support streaming tool calls, but with important differences:

**OpenAI** ([Streaming Docs](https://developers.openai.com/api/docs/guides/streaming-responses/)):
- Tool call arguments arrive as partial JSON chunks via `response.function_call_arguments.delta` events
- First chunk contains `id`, `function.name`, `type` — subsequent chunks contain argument fragments
- With `strict: true` (Structured Outputs), the JSON schema is guaranteed valid on completion
- Partial JSON cannot be safely parsed mid-stream — you must accumulate all chunks
- **TTFT includes the function name**: the model decides which tool to call early, but argument values stream out character by character

**Anthropic Claude** ([Streaming Docs](https://platform.claude.com/docs/en/build-with-claude/streaming)):
- Tool use blocks stream via `content_block_delta` events with `input_json_delta`
- Tool name is emitted in the `content_block_start` event (early)
- Arguments stream as partial JSON text
- Same limitation: partial JSON is not parseable until complete

**Google Gemini** ([Gemini Docs](https://ai.google.dev/gemini-api/docs/function-calling)):
- Function calls are returned as `functionCall` parts
- Streaming support exists but function call arguments are typically returned as a complete JSON object, not incrementally

##### Can Streaming Enable "Early Action"?

**Theoretical**: If the tool call schema puts the action direction first (e.g., `{"direction": "forward_left", "speed": "slow", ...}`), the robot could begin turning as soon as `"direction"` is parsed, before `"speed"` and `"reasoning"` arrive.

**Practical**: **No, this is not viable for tool/function calls**. The reasons:

1. **Tool calls stream as raw JSON text** — you get fragments like `{"dir` then `ecti` then `on":` then `"for` — these cannot be parsed until the full key-value pair arrives
2. **Key ordering is not guaranteed** — the model may emit `"reasoning"` before `"direction"`
3. **Structured Outputs guarantees schema but not key order** in the stream
4. **The latency savings are minimal** — for a 100-token response, streaming saves at most ~0.5-1.0s compared to non-streaming. The dominant latency is TTFT (model thinking time), not output generation

**Verdict**: Streaming is useful for **text responses** (user-facing observation descriptions) but provides **negligible benefit for tool-call action decisions**. The action JSON is short enough (~50-100 tokens) that the output generation time is small relative to TTFT.

##### Streaming Recommendation for This Robot

| Call Type | Streaming? | Rationale |
|-----------|-----------|-----------|
| Navigation decision (tool call) | **No** | Short output, can't parse partial JSON, TTFT dominates |
| Observation description (text) | **Optional** | Could speak first sentence while rest generates, but adds complexity |
| Complex reasoning (text) | **No** | Not applicable — nav decisions should be concise |

#### 5.4 Predictive Motion: What Does the Robot Do While Waiting?

The 1-5 second gap between VLM decisions is the critical architectural challenge. The robot cannot simply stop and wait — that would make exploration painfully slow and inefficient. Five strategies emerge from both robotics research and analysis of the current codebase:

##### Strategy 1: Continue Last Action (Dead Reckoning)

**Pattern**: After executing a VLM decision, continue the same velocity command until the next decision arrives or the LiDAR safety layer intervenes.

```
Decision at t=0: "move forward_left at slow speed"
  → cmd_vel: linear=0.12, angular=0.15
  → Robot continues this velocity from t=0 to t=3s (next decision)
  → LiDAR safety layer runs at 10-20 Hz, can override at any time

Distance traveled during gap: 0.12 m/s × 3s = 0.36m
Safety margin: LiDAR emergency stop at 0.25m, proportional slowdown at 0.50m
```

**Existing support**: The current `move()` function (line 2027) already implements this pattern — it loops at 20 Hz publishing the same velocity while checking obstacles. The exploration-loop `sleep(0.5)` between frontier decisions is functionally dead reckoning.

**Risk analysis**: At 0.12-0.18 m/s with 10 Hz LiDAR safety, the worst case is 0.018m of travel before an obstacle is detected. The robot's stopping distance is ~4cm (from Phase 4). Total reaction path: ~6cm — well within the 0.25m emergency stop zone.

**Verdict**: **This is the correct default strategy.** Simple, proven safe, and already supported by the existing safety infrastructure.

##### Strategy 2: Confidence-Based Speed Modulation

**Pattern**: Adjust the robot's speed between decisions based on environmental complexity detected by local sensors (no VLM needed).

```
Environmental Complexity Score (computed locally at 10 Hz):
  - LiDAR variance: high variance = complex geometry (many nearby obstacles)
  - Nearest obstacle distance: closer = more caution needed
  - Heading change rate: rapid turning = complex navigation
  - Time since last VLM decision: older = less confident

Speed modulation:
  CLEAR corridor (nearest > 2m, low LiDAR variance):
    → Run at full decision speed (0.18 m/s)
    → Long decision interval OK (3-5s)

  MODERATE complexity (nearest 1-2m, some obstacles):
    → Reduce to 0.12 m/s
    → Request faster VLM decisions if possible

  TIGHT space (nearest < 1m, high LiDAR variance):
    → Reduce to 0.06 m/s
    → Prioritize VLM decision (don't continue dead reckoning too long)
```

**Existing support**: `move()` already has proportional slowdown (line 2067): `speed_factor = max(0.3, (min_front - min_obstacle_dist) / (slow_dist - min_obstacle_dist))`. This could be extended to the between-decision gap.

**Verdict**: **High value, low effort.** Extend the existing proportional slowdown logic to run continuously between VLM decisions, not just within `move()` calls.

##### Strategy 3: Trajectory Extrapolation (Predict Next Decision)

**Pattern**: Use the last N VLM decisions to predict the likely next decision and pre-execute it.

**Analysis**: This is the approach used by **VLASH** (MIT HAN Lab, Nov 2025) — it rolls the robot state forward using the previously generated action chunk to predict the execution-time state. VLASH achieves up to 2.03× speedup and reduces reaction latency by 17.4×.

However, VLASH is designed for **fine-tuned VLA models** that output smooth action trajectories. For our system (API-based VLM with discrete direction choices), trajectory extrapolation is:
- **Simple case (straight corridor)**: Trivially correct — just keep going forward
- **Decision point (intersection/doorway)**: Prediction is unreliable — the VLM's reasoning about "which doorway to explore" is not predictable from past trajectory
- **The simple case is already handled by Strategy 1** (continue last action)

**Verdict**: **Not worth implementing for API-based VLM navigation.** Dead reckoning (Strategy 1) + speed modulation (Strategy 2) covers the cases where prediction would be accurate, and the VLM is specifically needed for the cases where prediction would fail.

##### Strategy 4: Asynchronous Inference (Overlapped Execution)

**Pattern**: Start preparing the NEXT sensor summary and VLM call while the CURRENT action is still executing. The VLM call runs in parallel with robot motion.

```
Timeline (overlapped):
  t=0.0s: VLM Decision A arrives → execute + START sensor prep for B
  t=0.1s: Sensor summary B ready → START VLM call for B (robot still executing A)
  t=0.5s: Robot finishes action A → continue dead reckoning
  t=2.1s: VLM Decision B arrives → execute + START sensor prep for C
  ...

Effective decision rate: 1 / (max(action_duration, api_latency))
vs. synchronous: 1 / (action_duration + api_latency)
```

This is the core pattern of **AsyncVLA** (arXiv 2602.13476, Feb 2025):
- Remote workstation runs large VLA, edge device runs lightweight adapter
- Communication delays up to 6 seconds handled gracefully
- 40% higher success rate than synchronous baselines

And **VLA-RAIL** (arXiv 2512.24673, Dec 2025):
- Client-server architecture with ZMQ protocol
- Trajectory Smoother filters jitter between action chunks
- Chunk Fuser ensures smooth transitions at decision boundaries

**Applicability to our system**: The key insight is that **VLM inference and robot motion should run in parallel, not sequentially**. The current `_observe_in_background()` (line 2329) already demonstrates this pattern — it runs GPT-4o vision in a background thread while navigation continues. The LLM decision loop should work the same way:

```python
# SYNCHRONOUS (bad — robot stops during API call):
while exploring:
    summary = prepare_sensors()     # 25ms
    decision = call_vlm(summary)    # 2000ms  ← robot idle!
    execute(decision)               # continues until next cycle

# ASYNCHRONOUS (good — robot never stops):
while exploring:
    if new_decision_available():
        execute(latest_decision)
    if not vlm_call_in_progress:
        summary = prepare_sensors()
        start_vlm_call_async(summary)  # non-blocking
    continue_current_motion()          # dead reckoning + safety
    sleep(0.05)                        # 20 Hz local loop
```

**Verdict**: **This is the correct architecture.** The VLM call must be non-blocking. The robot maintains a 20 Hz local control loop (matching current `move()` rate) with dead reckoning + safety, and VLM decisions are applied when they arrive asynchronously.

##### Strategy 5: Dual-Speed with Event Triggers

**Pattern**: Run the VLM at a low base frequency (0.2 Hz) but trigger immediate additional calls at navigation decision points detected by local sensors.

```
Event triggers for immediate VLM call:
  - Doorway detected (LiDAR gap ≥ 0.7m)  → "should I go through?"
  - T-intersection detected                → "which way?"
  - Dead end detected (all directions < 1m) → "what now?"
  - Novel object in camera (local YOLO)     → "what is this?"
  - Returned to previously visited location → "am I going in circles?"

Base rate: 0.2 Hz (one decision every 5s during straight corridors)
Triggered rate: immediate (as fast as API responds, ~0.5-3s)
```

**Existing support**: `_detect_doorways()` (line 1088) already identifies gaps ≥ 0.6m — this could trigger a VLM call. Frontier detection in `find_frontiers()` (line 645) identifies decision points.

**Verdict**: **Excellent strategy for cost optimization.** Reduces API calls by 50-70% in simple environments while maintaining responsiveness at decision points. Requires a local "complexity detector" that triggers VLM calls.

##### Recommended Combined Strategy

```
ARCHITECTURE: Asynchronous VLM with adaptive event-triggered calls

LOCAL LOOP (20 Hz, on exploration thread):
  1. Read LiDAR + obstacle distances
  2. Apply safety layer (emergency stop, proportional slowdown)
  3. Continue dead reckoning from last VLM decision
  4. Check event triggers (doorway, intersection, dead end)
  5. If trigger fired AND no VLM call pending → start async VLM call

VLM LOOP (async, on separate thread):
  1. Wait for trigger or base timer (5s max)
  2. Capture sensor snapshot (image + LiDAR + state)
  3. Call VLM API (blocking within this thread)
  4. Parse + validate response
  5. Post decision to shared queue
  6. Repeat

EXECUTION (on local loop, when new decision arrives):
  1. Read decision from queue
  2. Validate against safety layer
  3. Update cmd_vel target
  4. Log decision for exploration memory
```

#### 5.5 Decision Frequency Analysis

##### What Frequency Is Needed?

The robot moves at 0.15-0.18 m/s. At different decision frequencies:

| Frequency | Decision Interval | Distance Between Decisions | Scenario Coverage |
|-----------|-------------------|---------------------------|-------------------|
| 0.1 Hz | 10s | 1.5-1.8m | Corridors only — too slow for doorways |
| **0.2 Hz** | **5s** | **0.75-0.9m** | **Adequate for most indoor navigation** |
| **0.5 Hz** | **2s** | **0.30-0.36m** | **Good for complex environments** |
| 1.0 Hz | 1s | 0.15-0.18m | Unnecessary — LiDAR handles sub-meter |
| 2.0 Hz | 0.5s | 0.08-0.09m | Wasteful — no visual change in 0.5s |

**Key insight**: At 0.15 m/s, the visual scene changes slowly. A decision every 2-5 seconds is sufficient because:
- Doorways are ~0.8-1.0m wide → detected 2-4 decisions before reaching them
- Corridors are ~1.5-3m wide → multiple decisions available for course correction
- Furniture is static → one decision to avoid, then dead reckoning past it
- The LiDAR safety layer (10 Hz) handles all reactive avoidance

##### Adaptive Decision Frequency

Rather than fixed-rate VLM calls, use **environment-triggered adaptive frequency**:

| Environment | Base Rate | Trigger Rate | Rationale |
|-------------|-----------|-------------|-----------|
| Clear corridor | 0.2 Hz | On doorway detect | Straight line, nothing to decide |
| Room with furniture | 0.5 Hz | On obstacle proximity | More obstacles, more decisions |
| Intersection/junction | Immediate | N/A | Critical decision point |
| Dead end / stuck | Immediate | N/A | Need new strategy |
| Unknown / novel | 0.5-1.0 Hz | On scene change | Higher uncertainty |

**Cost impact of adaptive frequency**:
Using Gemini 2.0 Flash at $0.000156/call:
- Fixed 0.5 Hz: 1,800 calls/hr = $0.28/hr
- Adaptive (0.2 Hz base + triggers): ~800 calls/hr = $0.12/hr
- **Savings: ~57% fewer calls** with no loss in navigation quality

##### Research Paper Frequencies

| Project | Decision Frequency | Robot Speed | Context |
|---------|-------------------|-------------|---------|
| **NaVid** (RSS 2024) | ~0.7-0.8 Hz | ~0.2 m/s | Video-based VLN, cloud inference |
| **RT-2** (Google, 2023) | ~3 Hz | ~0.1 m/s | On-device TPU, end-to-end VLA |
| **VLM-Social-Nav** (GMU, 2024) | ~0.5 Hz | ~0.3 m/s | LiDAR + VLM, social constraints |
| **SayCan** (Google, 2022) | ~0.1 Hz | Variable | High-level planning, not continuous |
| **DP-VLA** (Oct 2024) | ~10 Hz (S-Sys1), ~0.1 Hz (L-Sys2) | Manipulation | Dual-process, fast+slow |
| **AsyncVLA** (Feb 2025) | ~2 Hz (edge), ~0.2 Hz (cloud) | ~0.3 m/s | Edge adapter + remote VLA |
| **VLASH** (MIT, Nov 2025) | ~2× base speed | Variable | Future-state-aware async |

**Conclusion**: 0.2-0.5 Hz is consistent with the research literature for cloud-based VLM navigation. Higher frequencies (1-10 Hz) are achieved only with on-device models or fine-tuned VLAs.

#### 5.6 Prompt Caching: Cost and Latency Optimization

All three major providers now offer prompt caching. The system prompt (~500-600 tokens) is identical every VLM call, making it a prime caching candidate.

##### Provider Caching Comparison

| Provider | Mechanism | Min Cacheable | Cache Duration | Read Discount | Write Cost | Source |
|----------|-----------|--------------|----------------|---------------|-----------|--------|
| **OpenAI** | Automatic | 1,024 tokens | ~5-10 min (auto) | **50% off input** | Free (auto) | [OpenAI Blog](https://openai.com/index/api-prompt-caching/) |
| **Anthropic** | Explicit (`cache_control`) | 1,024 tokens (Haiku), 2,048 (Sonnet) | 5 min or 1 hr | **90% off input** | 1.25× (5min) or 2× (1hr) | [Anthropic Docs](https://platform.claude.com/docs/en/build-with-claude/prompt-caching) |
| **Google Gemini** | Explicit + Implicit (auto since May 2025) | Varies | Configurable TTL | **75-90% off input** | Storage cost per hour | [Google Docs](https://ai.google.dev/gemini-api/docs/caching) |

##### Caching Impact on Navigation Decision Costs

Our system prompt + tool definitions are ~500-600 tokens. With a total input of ~1,000-1,500 tokens per call, the cacheable fraction is ~35-50%.

| Model | Base Cost/Call | With Caching | Savings | Hourly (0.5 Hz) |
|-------|---------------|-------------|---------|-----------------|
| **Gemini 2.0 Flash** | $0.000156 | ~$0.000120 | 23% | **$0.22** |
| **GPT-4o-mini** | $0.000208 | ~$0.000165 | 21% | **$0.30** |
| **Claude Haiku 4.5** | $0.001810 | ~$0.001150 | 36% | **$2.07** |
| **GPT-4o** | $0.003463 | ~$0.002900 | 16% | **$5.22** |
| **Claude Sonnet 4.5** | $0.005430 | ~$0.003400 | 37% | **$6.12** |

**Anthropic benefits most** from caching (90% read discount on ~40% of input = ~36% total savings), but remains the most expensive option for vision-heavy workloads due to higher base image token costs.

**OpenAI caching is automatic** — no code changes needed. For Anthropic, the system prompt must be wrapped in `cache_control: {"type": "ephemeral"}` blocks.

##### Caching Strategy for Navigation

```
ALWAYS CACHED (identical every call):
  - System prompt: ~500 tokens
  - Tool/function definitions: ~300 tokens
  - Safety rules: ~100 tokens
  Total cacheable: ~900 tokens

NEVER CACHED (changes every call):
  - Sensor summary: ~200 tokens
  - Camera image: 85-410 tokens
  - Exploration memory: ~200 tokens
  - Affordance scores: ~50 tokens
  Total dynamic: ~535-860 tokens
```

**OpenAI caveat**: Automatic caching requires the prompt to start with at least 1,024 identical tokens. Our system prompt + tools (~900 tokens) is just under this threshold. **Add a padding comment or expand tool descriptions** to reach 1,024 tokens, or the caching won't activate.

**Anthropic advantage**: Explicit caching with `cache_control` works at any size, and the 1-hour TTL option (`"ttl": "1h"`) is perfect for exploration sessions where the system prompt is static for the entire run.

#### 5.7 Context Window Management: Stateless vs. Conversational

##### Option A: Stateless Calls (Fresh Context Each Time)

Each VLM call includes the full system prompt + current sensor data + exploration memory summary. No conversation history.

```python
messages = [
    {"role": "system", "content": system_prompt + tool_defs},
    {"role": "user", "content": sensor_summary + exploration_memory + "Navigate."}
]
```

**Pros**:
- Simple to implement — no state management
- Prompt caching works optimally (static prefix)
- No context window growth over time
- Resilient to API failures (no state to lose)

**Cons**:
- Exploration memory must be explicitly serialized each call (~200 tokens)
- No implicit "I just saw a door on the left" continuity

##### Option B: Rolling Conversation (Last N Exchanges)

Maintain the last 2-4 VLM decision exchanges as conversation history.

```python
messages = [
    {"role": "system", "content": system_prompt + tool_defs},
    *conversation_history[-4:],  # Last 2 exchanges
    {"role": "user", "content": current_sensor_summary}
]
```

**Pros**:
- VLM has implicit temporal context ("I just turned left, now I see...")
- Reduces need for explicit exploration memory
- More natural reasoning flow

**Cons**:
- Context window grows (~300-500 tokens per exchange)
- Prompt caching less effective (prefix changes as history rotates)
- Must handle conversation corruption on API errors
- Current `voice_mapper.py` conversation management (line 2182-2185) is already fragile

##### Recommendation: Stateless with Structured Memory

**Use stateless calls** with an explicit `exploration_memory` JSON block that captures the last 3-5 decisions and their outcomes. This gives the VLM temporal context without the complexity of conversation management:

```json
{
  "recent_actions": [
    {"t": -10, "action": "move_forward", "speed": "medium", "result": "success", "observation": "corridor continues"},
    {"t": -5, "action": "turn_left", "speed": "slow", "result": "safety_override", "reason": "obstacle_left", "observation": "wall at 0.4m"},
    {"t": 0, "action": "pending", "observation": "doorway detected at 315°"}
  ],
  "rooms_explored": 2,
  "dead_ends": ["(1.2, -0.5)", "(3.1, 0.8)"],
  "total_distance": 23.4,
  "time_exploring": "4m 32s"
}
```

Token cost: ~150-200 tokens, providing equivalent context to 4-message conversation history.

#### 5.8 Image Caching and Frame Skipping

##### Problem: Redundant VLM Calls

If the robot is stationary or moving through a featureless corridor, consecutive camera frames are nearly identical. Sending the same visual scene to the VLM wastes tokens and money.

##### Solution: SSIM-Based Frame Skipping

Use Structural Similarity Index (SSIM) to detect significant visual change before triggering a VLM call:

```
Frame comparison (local, <5ms):
  1. Downsample current frame to 160×120 grayscale
  2. Compare to last-sent frame using SSIM
  3. If SSIM > 0.92 → skip (scene hasn't changed meaningfully)
  4. If SSIM < 0.92 → send to VLM (new visual information)

Exceptions (always send regardless of SSIM):
  - Event triggers (doorway, intersection)
  - Timer exceeded (max 10s without VLM call, safety backstop)
  - Robot was recently stuck or overridden
```

**Cost savings**: In a typical home exploration scenario:
- ~40% of time is spent in corridors with minimal visual change
- SSIM skipping could reduce VLM calls by 30-40%
- At 0.5 Hz base rate: reduces from $0.28/hr to ~$0.17/hr (Gemini 2.0 Flash)

##### Text-Only Fallback for Clear Corridors

When the camera shows a clear, featureless corridor (detected by low SSIM change + high LiDAR clearance), the VLM call could omit the image entirely:

```
IF nearest_obstacle > 3m AND ssim_vs_last_sent > 0.95:
  → Send text-only call (no image)
  → Token cost: ~900 tokens (vs. 985-1310 with image)
  → Savings: 9-31% per call
```

This is a minor optimization but demonstrates the principle of adaptive information density.

#### 5.9 Local VLM as Continuous Pre-Filter

##### Architecture: Two-Tier VLM System

```
┌─────────────────────────────────────────────────────────┐
│ TIER 1: Local VLM (Continuous, On-Device)               │
│   Model: Moondream2 (1.86B) or Florence-2-base (0.23B) │
│   Rate: 1-2 Hz continuous                               │
│   Output: Structured scene description (text)           │
│   Purpose: "What am I looking at?"                      │
│   VRAM: ~1.5GB (Moondream2) or ~1GB (Florence-2)       │
│                                                         │
│   Output example:                                       │
│   {"scene": "corridor", "objects": ["door_left",        │
│    "wall_right"], "floor": "clear", "lighting": "dim",  │
│    "novelty": 0.3}                                      │
├─────────────────────────────────────────────────────────┤
│ TIER 2: Cloud VLM (Triggered, API-Based)                │
│   Model: Gemini 2.0 Flash / GPT-4o-mini                 │
│   Rate: 0.2-0.5 Hz (event-triggered)                    │
│   Input: Camera image + LiDAR text + Tier 1 description │
│   Output: Navigation decision (tool call)               │
│   Purpose: "Where should I go and why?"                  │
└─────────────────────────────────────────────────────────┘
```

##### Tier 1 as Escalation Trigger

The local VLM's `novelty` score drives the adaptive decision frequency:

| Novelty Score | Meaning | Cloud VLM Trigger |
|---------------|---------|-------------------|
| 0.0-0.2 | Same scene, no change | No call (continue dead reckoning) |
| 0.2-0.5 | Minor change (new furniture, wall texture) | Base rate call (5s timer) |
| 0.5-0.8 | Significant change (doorway, intersection) | Immediate call |
| 0.8-1.0 | Major change (new room, person, obstacle) | Immediate call + detailed image |

##### Jetson Orin Nano 8GB Memory Budget

| Component | VRAM/RAM | Notes |
|-----------|----------|-------|
| ROS2 + Nav2 + SLAM | ~1.5-2GB | Occupancy grid, costmaps, particle filter |
| OAK-D driver (depthai) | ~0.5-1GB | USB3 + stereo pipeline |
| Moondream2 (4-bit quantized) | ~1.0-1.5GB | TensorRT optimized |
| Python runtime + buffers | ~0.5-1GB | Image buffers, LiDAR arrays, misc |
| **Total** | **~3.5-5.5GB** | **Leaves 2.5-4.5GB free** |

**Feasibility**: Moondream2 fits alongside SLAM + Nav2 + OAK-D on 8GB, but **Florence-2-base (~1GB) is safer** for margin. Quantized models (INT4/INT8) via TensorRT are essential.

**Benchmarks on Jetson Orin Nano** (from [NVIDIA Jetson AI Lab](https://developer.nvidia.com/embedded/jetson-benchmarks) and community reports):
- Quantized 1B-3B models: **28-55 tokens/sec**
- Moondream2 (1.86B, INT4): estimated **15-25 tokens/sec** (extrapolated from similar-size models)
- Florence-2-base (0.23B): estimated **30-50 tokens/sec** (very small model)
- Latency per inference: **100-400ms** for scene description (50-100 output tokens)

**Verdict**: A local VLM at 1-2 Hz is feasible on the Jetson Orin Nano 8GB, providing continuous scene awareness that bridges the gaps between cloud VLM decisions.

#### 5.10 Network Resilience and Offline Fallback

##### WiFi Performance in Home Environments

Based on robotics network research ([Frontiers in Robotics and AI](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2023.1168694/full)):

| Metric | Home WiFi (2.4/5 GHz) | Notes |
|--------|----------------------|-------|
| Median latency (LAN to internet) | 15-50ms | Plus API server processing |
| p95 latency | 100-300ms | Varies with congestion |
| p99 latency | 500-1,300ms | Spikes during interference |
| Packet loss | 0.5-2% typical, up to 12% in EMI | Metal shelving, microwave ovens |
| Throughput needed | ~150KB per VLM call | Well within WiFi capability |
| Signal degradation through walls | -3 to -10 dBm per wall | Robot may enter weak zones |

**Key risk**: The robot explores rooms far from the WiFi access point. Signal strength drops, latency increases, and API calls may timeout. The system must degrade gracefully.

##### 4-Tier Graceful Degradation (Refined from Phase 4)

```
TIER 0: NORMAL OPERATION
  Condition: API responds within 3s, success rate > 95%
  Behavior: Full LLM-driven navigation at adaptive frequency

TIER 1: ELEVATED LATENCY (3-6s API response)
  Condition: Rolling 5-call average > 3s
  Behavior:
    - Reduce decision frequency to 0.2 Hz (save budget for reliable calls)
    - Increase dead reckoning reliance
    - Slow robot to 0.12 m/s (more time between decisions)
    - Log warning: "API latency elevated"

TIER 2: DEGRADED (API timeout or 3 consecutive failures)
  Condition: No successful API response in 10s
  Behavior:
    - Stop robot (safety first)
    - Switch to local VLM if available (Moondream2 → text descriptions → simple rules)
    - If no local VLM: switch to existing frontier exploration (proven fallback)
    - Announce: "Network issues, switching to local navigation"

TIER 3: OFFLINE (No API response in 30s)
  Condition: All retry attempts exhausted
  Behavior:
    - Continue local/frontier exploration if in progress
    - Begin return-to-start using Nav2 (if pose is reliable)
    - Announce: "Connection lost, returning to start"
    - Periodic reconnection attempts (every 30s, not exponential backoff)
```

**Important**: Exponential backoff is **wrong** for robot control (from Phase 4 findings). Fixed-interval retry (every 30s) ensures the robot reconnects as soon as possible without wasting compute on rapid retries.

##### Network Quality Monitoring

Monitor WiFi health proactively to anticipate degradation:

```python
class NetworkMonitor:
    """Track API call success rate and latency for tier decisions."""

    def __init__(self):
        self.call_history = deque(maxlen=20)  # Last 20 calls

    def record_call(self, latency_ms, success):
        self.call_history.append((latency_ms, success, time.time()))

    @property
    def current_tier(self):
        if len(self.call_history) < 3:
            return 0  # Not enough data

        recent = list(self.call_history)[-5:]
        avg_latency = mean(lat for lat, _, _ in recent if lat > 0)
        success_rate = sum(1 for _, s, _ in recent if s) / len(recent)
        last_success_ago = time.time() - max(
            (t for _, s, t in self.call_history if s), default=0
        )

        if last_success_ago > 30:
            return 3  # OFFLINE
        if last_success_ago > 10 or success_rate < 0.4:
            return 2  # DEGRADED
        if avg_latency > 3000:
            return 1  # ELEVATED
        return 0      # NORMAL
```

##### Integration with Existing Sensor Monitor

The `_sensor_monitor_loop()` (line 2722) already checks for LiDAR/odom/camera loss every 2s. Network quality monitoring should follow the same pattern — a lightweight check thread that sets a `self.network_tier` flag read by the decision loop.

#### 5.11 Complete Decision Loop Architecture

Bringing all findings together into a unified architecture:

```
┌─────────────────────────────────────────────────────────────┐
│ MAIN THREAD (10 Hz): rclpy.spin_once()                     │
│   ROS2 callbacks: camera, depth, LiDAR, odom, map          │
├─────────────────────────────────────────────────────────────┤
│ VOICE THREAD: voice_loop()  [unchanged]                     │
│   User commands → think() → execute()                       │
│   Voice can interrupt/override LLM navigation               │
├─────────────────────────────────────────────────────────────┤
│ LOCAL CONTROL LOOP (20 Hz): navigation_control_loop()       │
│   ├── Read latest sensor data                               │
│   ├── Apply safety layer (emergency stop, proportional      │
│   │   slowdown, obstacle avoidance)                         │
│   ├── Execute current velocity command (dead reckoning)     │
│   ├── Check event triggers (doorway, intersection, stuck)   │
│   ├── Check for new VLM decision in queue                   │
│   │   └── If available: validate + update velocity target   │
│   ├── Confidence-based speed modulation                     │
│   └── Publish cmd_vel                                       │
├─────────────────────────────────────────────────────────────┤
│ VLM DECISION THREAD: vlm_decision_loop()  [NEW]            │
│   ├── Wait for trigger (event or base timer, 2-5s)          │
│   ├── Check network tier                                    │
│   │   ├── Tier 0-1: Cloud VLM call                          │
│   │   ├── Tier 2: Local VLM or frontier fallback            │
│   │   └── Tier 3: Return to start                           │
│   ├── Prepare sensor snapshot                               │
│   │   ├── Annotated camera image (depth + heading)          │
│   │   ├── 12-sector LiDAR text summary                      │
│   │   ├── Robot state (position, heading, speed)            │
│   │   ├── Exploration memory (last 5 actions + outcomes)    │
│   │   └── Affordance scores (direction feasibility)         │
│   ├── (Optional) SSIM check — skip if scene unchanged       │
│   ├── Call VLM API (blocking within this thread)            │
│   ├── Parse tool call response                              │
│   ├── Post decision to shared queue                         │
│   └── Update exploration memory                             │
├─────────────────────────────────────────────────────────────┤
│ LOCAL VLM THREAD (optional): local_vlm_loop()  [NEW]       │
│   ├── Runs Moondream2/Florence-2 at 1-2 Hz                  │
│   ├── Produces structured scene descriptions                │
│   ├── Computes novelty score for adaptive triggering        │
│   └── Feeds text descriptions to VLM decision thread        │
├─────────────────────────────────────────────────────────────┤
│ SENSOR MONITOR THREAD (0.5 Hz): _sensor_monitor_loop()     │
│   ├── LiDAR/odom/camera health check [existing]             │
│   └── Network quality monitoring [NEW]                      │
└─────────────────────────────────────────────────────────────┘
```

**Thread count**: 5-6 threads (up from current 4). The Python GIL is not a concern because:
1. VLM API calls are I/O-bound (network wait), releasing GIL
2. Local VLM inference uses C extensions (TensorRT/ONNX), releasing GIL
3. ROS2 spin uses C extensions, releasing GIL
4. The 20 Hz local control loop is computationally trivial (<1ms per cycle)

#### 5.12 Key Findings Summary

1. **API latency dominates the decision cycle** — VLM inference is 75-95% of cycle time. Local processing (sensor fusion, safety checks, parsing) totals only 7-32ms. Optimizing local code provides negligible benefit; optimizing API call frequency and timing is critical.

2. **Streaming is not useful for tool-call decisions** — Partial JSON cannot be parsed for early action execution. TTFT (model thinking time) is the bottleneck, not output generation speed. Streaming only helps for text-based observation descriptions.

3. **Asynchronous inference is mandatory** — The VLM call must run on a separate thread while the robot continues moving. Synchronous "stop-think-move" would make exploration 3-5× slower. This pattern is validated by AsyncVLA, VLA-RAIL, and VLASH research (2025).

4. **Dead reckoning + LiDAR safety is sufficient between decisions** — At 0.15 m/s with 10 Hz safety layer, the robot can safely continue its last commanded velocity for 2-5 seconds. Stopping distance is ~4cm; emergency stop zone is 25cm. The safety margin is >6× adequate.

5. **Adaptive event-triggered frequency saves 50-70% on API costs** — Fixed-rate calling at 0.5 Hz wastes calls in corridors. Triggering on doorways/intersections/novelty reduces calls to ~800/hr (from ~1,800/hr) with no loss in navigation quality.

6. **0.2-0.5 Hz is the correct decision frequency range** — Consistent with NaVid (0.7 Hz), VLM-Social-Nav (0.5 Hz), and AsyncVLA cloud tier (0.2 Hz). At 0.15 m/s, this provides 3-6 decisions per corridor length.

7. **Gemini 2.0 Flash is the latency winner** — 250-300ms TTFT, ~400-600ms total response, enabling up to 1.5 Hz decisions. GPT-4o-mini is second (300-500ms TTFT). Claude Haiku 4.5 is third (400-600ms TTFT).

8. **Prompt caching provides 16-37% cost reduction** — Anthropic benefits most (90% read discount) but starts from a higher base. OpenAI caching is automatic but requires ≥1,024 tokens of static prefix. Google offers 75-90% discount with explicit caching.

9. **Stateless calls with structured exploration memory outperform rolling conversation** — Explicit JSON memory (~200 tokens) provides equivalent temporal context to 4-message conversation history, with better prompt caching, simpler error handling, and no context window growth.

10. **A local VLM (Moondream2/Florence-2) is feasible as a continuous pre-filter** — Fits in ~1-1.5GB alongside SLAM/Nav2/OAK-D on 8GB Jetson. Provides 1-2 Hz scene descriptions + novelty scoring to drive adaptive cloud VLM triggering.

11. **4-tier network degradation handles WiFi variability** — Normal → Elevated (slow down) → Degraded (local/frontier fallback) → Offline (return home). Fixed-interval retry, not exponential backoff. Network quality monitored proactively via rolling API success rate.

12. **The complete architecture requires 5-6 threads** — Main (ROS2), Voice (unchanged), Local Control (20 Hz, NEW), VLM Decision (async, NEW), optional Local VLM (1-2 Hz), Sensor Monitor (extended). Python GIL is not a concern due to I/O-bound and C-extension workloads.

---

## Phase 6: Prior Art, Frameworks & Implementation Path

**Scope**: Survey existing work and design concrete implementation steps:

- **Academic/Industry Projects**:
  - Google SayCan / PaLM-E — grounded language models for robots
  - RT-2 (Robotic Transformer) — vision-language-action models
  - LM-Nav — LLM-based outdoor navigation
  - VoxPoser — LLM-driven manipulation
  - CoW (CLIP on Wheels) — object-goal navigation
  - TidyBot — LLM for household robot planning
  - NaVid — vision-language navigation
- **ROS2 Integration Patterns**: How other projects integrate LLMs with ROS2
- **Frameworks**: LangChain + ROS2, ROS2-LLM bridges, custom approaches
- **Concrete Implementation Plan**: Phased build on `voice_mapper.py`
  - Phase A: Continuous perception loop (camera snapshots at fixed interval)
  - Phase B: Sensor summary generation (pre-process for LLM consumption)
  - Phase C: LLM decision loop (replace frontier exploration with LLM decisions)
  - Phase D: Safety layer integration
  - Phase E: Memory and map annotation
  - Phase F: Multi-model architecture (fast local + smart cloud)

**Key Questions**:
- Which existing framework is closest to what we need?
- What's the minimum viable version we can build first?
- How to A/B test LLM exploration vs. frontier-based exploration?

**Status**: ✅ Complete

### Findings

#### 6.1 Academic/Industry Prior Art — Navigation & Exploration Focus

The following projects represent the state-of-the-art in LLM/VLM-driven robot navigation. Projects are ordered by relevance to our use case (autonomous exploration with VLM decision-making on a mobile robot).

##### 6.1.1 VLFM — Vision-Language Frontier Maps (Most Relevant)

- **Paper**: "VLFM: Vision-Language Frontier Maps for Zero-Shot Semantic Navigation" (ICRA 2024)
- **GitHub**: https://github.com/bdaiinstitute/vlfm (682 stars, 84 forks, MIT license)
- **Authors**: Yokoyama et al. (Boston Dynamics AI Institute)
- **Architecture**: Builds occupancy maps from depth observations to identify frontiers, then uses RGB observations + a pre-trained VLM (BLIP-2, GPT-4V) to generate a language-grounded value map that scores each frontier by relevance to the target object. The highest-scoring frontier is selected for exploration.
- **Key insight**: VLM scores frontier attractiveness from RGB images — the robot uses classical frontier detection but lets the VLM decide *which* frontier to explore. This is exactly the hybrid pattern our Phase 4 "SayCan affordance scoring" envisions.
- **Real-world deployment**: Demonstrated on Boston Dynamics Spot in an office building, zero-shot, no prior environment knowledge.
- **Relevance**: **Very high** — same concept as our "LLM picks where to go next among frontier candidates." Differences: VLFM targets object-goal navigation (find the couch), not open-ended exploration. Adapting it to curiosity-driven exploration (explore the most interesting/novel frontier) is a natural extension.

##### 6.1.2 Berkeley Frontier+VLM Exploration (2025)

- **Paper**: "Autonomous Frontier-Based Exploration with High-Level VLM Guidance" (EECS-2025-172, UC Berkeley)
- **URL**: https://www2.eecs.berkeley.edu/Pubs/TechRpts/2025/EECS-2025-172.pdf
- **Architecture**: Outsources high-level planning to a VLM which guides a frontier-based exploration agent through:
  - Structured chain-of-thought prompting
  - Dynamic inclusion of the agent's recent action history (prevents getting stuck)
  - Interpretation of top-down obstacle maps alongside first-person RGB views
- **Key insight**: The VLM receives both a bird's-eye obstacle map AND the first-person camera view, combining spatial awareness with visual understanding. Chain-of-thought prompting forces the VLM to reason about exploration strategy before outputting a direction.
- **Relevance**: **Very high** — directly addresses autonomous exploration (not just object search). The dual-view approach (top-down map + first-person camera) matches our Phase 2 sensor fusion strategy. The chain-of-thought + action history matches our Phase 3 prompt design.

##### 6.1.3 SayCan / PaLM-SayCan (Google, 2022)

- **Paper**: "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances" (CoRL 2022)
- **URL**: https://say-can.github.io/
- **Architecture**: `score = P_LLM(action|task) × P_affordance(feasibility|state)`. The LLM proposes actions ranked by task relevance, and an affordance model (learned value functions) filters by physical feasibility. The highest joint-scored action is executed.
- **Key insight**: The LLM never directly controls motors. It scores candidate actions, and the affordance model grounds them in reality. This is the pattern our Phase 4 safety architecture adopted.
- **Performance**: 84% correct action sequences, 74% successful execution.
- **Relevance**: **High** — the affordance-scoring pattern is already incorporated into our design. SayCan focuses on tabletop manipulation, not navigation, but the scoring principle is universal. Our LiDAR feasibility scores serve the same role as SayCan's learned value functions.

##### 6.1.4 NaVid — Video-based VLM Navigation (RSS 2024)

- **Paper**: "NaVid: Video-based VLM Plans the Next Step for Vision-and-Language Navigation"
- **GitHub**: https://github.com/jzhzhang/NaVid-VLN-CE
- **Architecture**: Fine-tuned VLM (based on LLaMA-VID) that takes a video stream from a monocular RGB camera and outputs next-step actions. Represents each video frame with instruction-queried tokens (task-relevant) + instruction-agnostic tokens (scene understanding). Frames sampled at 1 FPS.
- **Key insight**: Video context (multiple frames) helps the VLM understand motion and spatial layout better than single images. No maps, no odometry, no depth required — pure RGB video to action.
- **Decision frequency**: ~1 FPS (0.7-1 Hz), matching our Phase 5 target of 0.2-0.5 Hz.
- **Relevance**: **Medium-high** — demonstrates that VLM-only navigation is feasible at ~1 Hz. However, NaVid requires fine-tuning (510k navigation samples), which we cannot do with cloud API models. The video-frame sampling strategy is relevant to our perception loop design.

##### 6.1.5 LM-Nav — Outdoor LLM Navigation (CoRL 2022)

- **Paper**: "LM-Nav: Robotic Navigation with Large Pre-Trained Models of Language, Vision, and Action"
- **URL**: https://sites.google.com/view/lmnav
- **Architecture**: Three-model pipeline: GPT-3 extracts textual landmarks from instructions → CLIP grounds landmarks in visual observations (from a pre-built topological graph) → ViNG (visual navigation model) executes goal-conditioned navigation between graph nodes.
- **Key insight**: Pure zero-shot composition of pre-trained models (no fine-tuning). The LLM never directly controls the robot — it decomposes instructions into landmark sequences.
- **Relevance**: **Medium** — demonstrates that LLM + VLM + navigation policy can work zero-shot for outdoor navigation. However, LM-Nav requires a pre-built topological graph of the environment (walked beforehand), which contradicts our exploration-of-unknown-environments goal.

##### 6.1.6 CoW — CLIP on Wheels (CVPR 2023)

- **GitHub**: https://github.com/real-stanford/cow (150 stars)
- **Architecture**: Decomposes object-goal navigation into: (1) CLIP-based object localization from RGB images, and (2) frontier-based exploration when the target is not yet visible. Switches between "explore" and "navigate-to-target" modes based on CLIP confidence.
- **Key insight**: Modular — classic frontier exploration handles spatial coverage, VLM handles semantic recognition. No fine-tuning needed.
- **Relevance**: **Medium** — the explore/exploit switch pattern is useful. For our open-ended exploration, we do not have a target object, but the pattern of "use VLM confidence to decide when to switch behavior" applies (e.g., "this area looks interesting, investigate" vs. "this is boring, move to next frontier").

##### 6.1.7 VoxPoser — LLM-Driven Manipulation (CoRL 2023)

- **GitHub**: https://github.com/huangwl18/VoxPoser
- **Architecture**: LLM generates Python code that creates 3D value maps (affordance maps, avoidance maps, velocity maps) in the robot's observation space. These maps serve as cost functions for model-predictive control (MPC) trajectory optimization.
- **Key insight**: LLM writes code to create spatial value functions rather than directly outputting actions. This enables composable, grounded behaviors.
- **Relevance**: **Low for navigation** — focused on tabletop manipulation. However, the concept of "LLM generates a value map, planner executes on it" is related to VLFM's frontier scoring approach. The code-generation pattern is too complex for our real-time loop.

##### 6.1.8 OrionNav — LLM + Scene Graphs + ROS2 Nav2 (2024)

- **Paper**: "OrionNav: Towards Vision-Language-Driven Multi-Robot Navigation" (arXiv 2025)
- **Architecture**: Hierarchical scene graph (rooms → objects → relationships) built from RGB-D + LiDAR, queried by an LLM (GPT-4) to generate navigation plans. Uses ROS2 Nav2 for execution with Jetson Orin hardware. Three discrete navigation primitives: `goto(room/object)`, `search_room(target)`, `explore_globally()`.
- **Key insight**: Scene graphs provide structured spatial context to the LLM — more interpretable and tokenefficient than raw sensor data or maps. The LLM reasons over a symbolic graph representation, not pixels.
- **Hardware**: Jetson Orin + LiDAR + RGB-D (nearly identical to our setup)
- **Relevance**: **Very high** — closest hardware/software match to our system. The scene graph approach is complementary to our annotated-image approach; both could coexist. The three navigation primitives are a subset of our 7-tool design.

##### 6.1.9 COME-Robot — Chain-of-Modality Enhancement (2024)

- **Paper**: "Chain-of-Modality Enhancement: LLM-Based Robot Navigation" (arXiv 2024)
- **Architecture**: System prompt with structured API documentation + chain-of-thought + Python code generation + closed-loop action feedback. The LLM receives multimodal inputs (RGB + depth + occupancy) formatted as structured text + images, reasons step-by-step, then calls navigation APIs.
- **Key insight**: The prompt architecture pattern — providing the LLM with "API documentation" for robot capabilities — maps directly to tool/function calling. The chain-of-thought + execution feedback loop is what our Phase 3 design implements.
- **Relevance**: **High** — validates our prompt architecture. Already referenced in Phase 3.

##### 6.1.10 AsyncVLA — Asynchronous Vision-Language-Action (2024-2025)

- **Paper**: "Asynchronous Large Language Model Enhanced Planner for Autonomous Driving" + navigation variants (2024-2025)
- **Architecture**: Cloud VLM runs asynchronously while a fast local controller handles real-time execution. The VLM provides high-level guidance (direction, strategy) at 0.2 Hz, while the local policy handles obstacle avoidance at 10+ Hz. Tested with delays up to 6 seconds.
- **Key insight**: **Multi-second VLM delays are acceptable** for indoor navigation when paired with a fast reactive controller. Achieved 85% success rate even with 6-second VLM latency.
- **Relevance**: **Very high** — directly validates our core timing assumption (0.2-0.5 Hz VLM + 20 Hz local safety loop). The dual-system pattern (slow cloud + fast local) is exactly our Phase 5 architecture.

##### 6.1.11 TidyBot — LLM for Household Robot Planning (Princeton, 2023)

- **Paper**: "TidyBot: Personalized Robot Assistance with Large Language Models" (IROS 2023)
- **GitHub**: https://github.com/jimmyyhwu/tidybot
- **Architecture**: LLM (GPT-4) receives natural language descriptions of household objects and generates personalized tidying plans based on user preferences learned from a few examples. Plans are sequences of pick-place operations executed by a mobile manipulator.
- **Key insight**: LLMs can learn user-specific preferences from minimal examples and apply them to novel objects. The "few-shot preference learning" pattern is applicable to personalized exploration (e.g., "this user is interested in electronics, prioritize rooms with screens/cables").
- **Relevance**: **Low-medium** — manipulation-focused, but the personalized planning pattern and household spatial reasoning are transferable to exploration preferences.

##### 6.1.12 PaLM-E & RT-2 — Foundation Models for Embodied AI (Google, 2023)

- **PaLM-E** (562B): Injects images and robot state directly into LLM embedding space for joint multimodal reasoning. Achieves 94.9% TAMP success. Validates that web-trained vision-language knowledge transfers to robotic reasoning.
- **RT-2** (55B): Vision-Language-Action model that outputs tokenized motor commands at 1-5 Hz. Represents actions as text tokens — the conceptual ancestor of our tool-calling approach. Demonstrates 2× improvement on novel objects via web knowledge transfer.
- **Relevance**: **Medium** — both are manipulation-focused with custom-trained models too large for our use. Key takeaways: (1) web-trained VLMs transfer knowledge to robot control (validates using off-the-shelf cloud VLMs), (2) discrete/tokenized actions outperform continuous for prompted models, (3) multimodal input dramatically improves reasoning vs. text-only.

#### 6.2 ROS2 + LLM Integration Projects

The following are open-source ROS2 packages specifically built to bridge LLMs with robot control systems.

##### 6.2.1 ROSA — Robot Operating System Agent (NASA JPL)

- **GitHub**: https://github.com/nasa-jpl/rosa (1,405 stars, 144 forks)
- **Created**: Aug 2024 | **Last updated**: Feb 2025 | **Language**: Python
- **ROS2 compatibility**: ROS1 and ROS2 (Humble confirmed)
- **LLM backend**: Any LangChain-supported model (OpenAI, Anthropic, etc.)
- **Architecture**:
  - Built on LangChain as a ReAct agent
  - Python functions decorated with `@tool` become callable actions for the LLM
  - Separate tool modules for ROS1 (`ros1.py`) and ROS2 (`ros2.py`)
  - The agent can: list topics/services, publish messages, call services, read parameters, inspect TF tree
  - Safety mechanisms: parameter validation, constraint enforcement
- **Integration pattern**: LangChain ReAct agent → `@tool`-decorated Python functions → ROS2 CLI/API calls
- **Exploration/navigation use**: **No** — ROSA is a diagnostic/operator tool ("tell me what topics are active," "call this service"), not a navigation controller. However, it demonstrates the LangChain + ROS2 `@tool` pattern cleanly.
- **Relevance to our project**: **Medium** — the `@tool` pattern could inform how we expose move/look/explore as LLM-callable tools. ROSA is too high-level (operating on ROS2 infrastructure, not sensor data) for direct use.

##### 6.2.2 llama_ros — Local LLM/VLM in ROS2

- **GitHub**: https://github.com/mgonzs13/llama_ros (246 stars, 43 forks)
- **Created**: Apr 2023 | **Last updated**: Feb 2025 | **Language**: C++
- **ROS2 compatibility**: Humble, Iron, Jazzy
- **Architecture**:
  - Core `Llama` C++ class wraps llama.cpp (independent of ROS2)
  - `LlamaNode` exposes ROS2 interfaces: an **action** for text generation, **services** for tokenization and embeddings
  - Includes `LlamaClientNode` example for using from other nodes
  - LangChain integration for prompt engineering
  - Supports GGUF-based LLMs and VLMs (llava.cpp for vision)
- **Integration pattern**: **ROS2 Action** for inference (long-running, with feedback), **ROS2 Services** for tokenization/embeddings
- **Key design decision**: Action (not service) for generation — correct choice because LLM inference is long-running and benefits from streaming feedback
- **Exploration/navigation use**: Not directly — provides the inference infrastructure. A separate node would consume the action to make navigation decisions.
- **Relevance to our project**: **High** — if we run a local VLM (Moondream2) on the Jetson, llama_ros provides the ROS2 integration pattern. The Action-based interface is the right model for our async VLM decision loop. Could serve as local pre-filter node in our multi-model architecture.

##### 6.2.3 ros2_nanollm — NVIDIA Jetson LLM/VLM Nodes

- **GitHub**: https://github.com/NVIDIA-AI-IOT/ros2_nanollm (184 stars, 6 forks)
- **Created**: Jul 2024 | **Last updated**: Feb 2025 | **Language**: Python
- **ROS2 compatibility**: Humble (via Jetson containers)
- **Architecture**:
  - ROS2 nodes optimized for NVIDIA Jetson Orin (our platform)
  - Provides LLM, VLM, and VLA inference nodes
  - VLM node subscribes to camera topics (RTP video stream), publishes scene descriptions
  - Runs inside `jetson-containers` Docker environment
  - Uses NanoLLM library with TensorRT optimization
- **Integration pattern**: **ROS2 Topics** — subscribes to image topics, publishes text descriptions
- **Exploration/navigation use**: VLM scene description could feed into navigation decisions
- **Relevance to our project**: **Very high** — this runs on our exact hardware (Jetson Orin Nano). The VLM scene description node could serve as our local pre-filter (Phase F of implementation). However, 8GB VRAM is tight — need to verify model fits alongside SLAM/Nav2/OAK-D driver.

##### 6.2.4 ROS-LLM — Embodied AI Framework (Auromix)

- **GitHub**: https://github.com/Auromix/ROS-LLM (743 stars, 92 forks)
- **Created**: Jun 2023 | **Last updated**: Feb 2025 | **Language**: Python
- **ROS2 compatibility**: Humble (separate `ros2-humble` branch)
- **Paper**: arXiv:2406.19741 (2024)
- **Architecture**:
  - AI agent connected to LLMs (GPT-4, ChatGPT, open-source models)
  - Atomic actions implemented as **ROS2 Actions or Services**, described in JSON files (name, type, description, input/output)
  - Three behavior execution modes: **sequence**, **behavior tree**, **state machine**
  - LLM extracts behavior from its output → framework maps to ROS2 action/service calls
  - Imitation learning for adding new robot skills
  - LLM reflection via human + environment feedback (closed-loop)
- **Integration pattern**: JSON action descriptions → LLM selects actions → framework calls ROS2 actions/services → result fed back to LLM
- **Key feature**: Behavior tree support — LLM can compose multi-step behaviors as trees, not just flat sequences
- **Exploration/navigation use**: Framework supports navigation actions but focuses on manipulation demos
- **Relevance to our project**: **High** — the JSON action description pattern matches our Phase 3 tool design. The behavior tree execution mode could handle multi-step exploration plans. The feedback loop matches our Phase 3 requirement for closed-loop action results. However, the framework adds significant complexity — we may prefer a lighter custom approach.

##### 6.2.5 ROSGPT — ChatGPT + ROS2 (Koubaa et al.)

- **GitHub**: https://github.com/aniskoubaa/rosgpt (446 stars, 88 forks)
- **Created**: Apr 2023 | **Last updated**: Jan 2026 | **Language**: Python
- **Paper**: Preprints.org 202304.0827
- **Architecture**:
  - ChatGPT API translates natural language → JSON-serialized robot commands
  - Ontology-based prompt engineering converts unstructured language to structured ROS commands
  - JSON commands interpreted by ROS2 nodes for execution (Turtlesim, Turtlebot3)
  - Evaluated across 5 LLMs (LLaMA-7b, LLaMA2-7b, LLaMA2-70b, GPT-3.5, GPT-4) on 3,000 commands
- **Integration pattern**: Natural language → LLM → JSON → ROS2 topic publish
- **Key finding**: F1-score of 0.913 for full prompt transformation, 0.975 for individual commands (GPT-4)
- **Exploration/navigation use**: Basic Turtlebot3 navigation commands only
- **Relevance to our project**: **Low-medium** — demonstrates the NL→JSON→ROS pattern but is too simple for our needs. No sensor feedback loop, no multi-step planning, no vision. The quantitative LLM comparison across models is useful reference data.

##### 6.2.6 ROS-MCP-Server — Model Context Protocol Bridge

- **GitHub**: https://github.com/robotmcp/ros-mcp-server (1,031 stars, 152 forks) — also https://github.com/lpigeon/ros-mcp-server (same repo)
- **Created**: Apr 2025 | **Last updated**: Feb 2025 | **Language**: Python
- **ROS2 compatibility**: ROS1 and ROS2
- **Architecture**:
  - Bridges MCP (Model Context Protocol) with ROS via rosbridge
  - LLM (Claude Desktop, ChatGPT, Gemini) connects via MCP protocol
  - Can: list topics/services/message types, publish/subscribe topics, call services, read sensor data
  - No robot code changes required — only add rosbridge node
  - Also: https://github.com/kakimochi/ros2-mcp-server (75 stars) — FastMCP-based, runs as ROS2 node, publishes cmd_vel directly
- **Integration pattern**: MCP protocol → rosbridge → ROS2 topics/services
- **Exploration/navigation use**: Can send cmd_vel commands; primarily for debugging/operator interaction
- **Relevance to our project**: **Low** — MCP is designed for interactive AI assistants, not autonomous real-time control. Too much protocol overhead for a 0.2-0.5 Hz decision loop. Interesting as a debugging/monitoring interface alongside the main control loop.

##### 6.2.7 Other Notable Projects

| Project | GitHub | Stars | Description | Relevance |
|---------|--------|-------|-------------|-----------|
| robochain | NoneJou072/robochain | 131 | ROS2 + LangChain simulation framework, prompt-to-code execution | Low — simulation-focused |
| LLM-Robot | ksDreamer/LLM-Robot | 46 | LLM parses voice commands to ROS control code | Low — voice command only |
| langchain_agent_robot_controller_ros2 | ATh0ft/... | 6 | LangChain + ChatGPT bimanual robot control | Low — manipulation focus |
| KIOS | ProNeverFake/kios | 73 | LLM + behavior trees for task planning | Medium — BT generation is interesting |
| OperateLLM | (IEEE paper) | N/A | DeepSeek Coder + rclpy ReAct agent for ROS2 dev | Low — development tool, not runtime |

#### 6.3 Integration Patterns — Taxonomy

Based on the survey above, ROS2 + LLM integration falls into four distinct patterns:

##### Pattern 1: LLM as Action Server (llama_ros pattern)

```
[Sensor Nodes] --topics--> [Decision Node] --action goal--> [LLM Action Server]
                                                     <--action feedback/result--
                            [Decision Node] --cmd_vel/nav goal--> [Motor/Nav2]
```

- **ROS2 primitive**: Action (long-running, with feedback)
- **Threading**: LLM inference runs in action server's executor thread
- **Latency handling**: Action client sends goal, continues processing while waiting for result
- **Used by**: llama_ros (local inference), could be adapted for cloud API
- **Pros**: Native ROS2 pattern, supports cancellation, feedback streaming
- **Cons**: Action overhead for simple request/response; overkill if no intermediate feedback needed

##### Pattern 2: LLM as Service (ROSGPT / simple API call pattern)

```
[Controller Node] --service request--> [LLM Service Node] --HTTP--> [Cloud API]
                  <--service response--                    <--------
```

- **ROS2 primitive**: Service (synchronous request/response)
- **Threading**: Blocks the calling thread during LLM inference (1-3 seconds)
- **Latency handling**: **Must use async service client** to avoid blocking ROS2 callbacks
- **Used by**: ROSGPT, simple integration projects
- **Pros**: Simple, maps directly to API request/response
- **Cons**: Blocking if not async; no intermediate feedback; no cancellation

##### Pattern 3: LLM on Separate Thread with Topic Bridge (our proposed pattern)

```
[Sensor Nodes] --topics--> [Sensor Aggregator] --queue--> [LLM Thread (async)]
                                                           |
                                                           v
[Safety Layer] <--topic-- [Command Publisher] <-- [LLM Response Parser]
```

- **ROS2 primitive**: Topics for input/output, internal threading for LLM call
- **Threading**: Dedicated Python thread for async API calls, publishes results to topics
- **Latency handling**: Robot continues moving on previous command while LLM thinks
- **Used by**: Custom implementations, voice_mapper.py's current exploration_loop
- **Pros**: Decoupled from ROS2 callback model; natural for async cloud API; robot never blocks
- **Cons**: Must handle thread safety manually; no ROS2-native cancellation/feedback

##### Pattern 4: LangChain ReAct Agent (ROSA / ROS-LLM pattern)

```
[User/Trigger] --> [LangChain Agent] --@tool calls--> [ROS2 Functions]
                                     <--tool results--
                   [LangChain Agent] --@tool calls--> [ROS2 Functions]
                   ...repeats until agent "done"...
```

- **ROS2 primitive**: None directly — Python functions called by LangChain, which internally use rclpy
- **Threading**: LangChain manages the agent loop; ROS2 calls happen within tool functions
- **Latency handling**: Agent loop runs until complete; each tool call is a separate LLM inference
- **Used by**: ROSA, robochain, KIOS
- **Pros**: Rich reasoning (multi-step ReAct); tool composition; memory management via LangChain
- **Cons**: Multiple LLM calls per decision (expensive, slow); LangChain adds abstraction overhead; harder to meet real-time constraints

##### Pattern Selection for Our Project

| Criterion | Pattern 1 (Action) | Pattern 2 (Service) | **Pattern 3 (Thread+Topic)** | Pattern 4 (ReAct) |
|-----------|-------------------|---------------------|---------------------------|-------------------|
| Matches async cloud API | Possible | Poor (blocking) | **Best** | Possible |
| Latency tolerance | Good | Poor | **Best** | Poor (multi-call) |
| Robot continues moving | Yes (with effort) | No | **Yes (natural)** | No |
| ROS2-native | Yes | Yes | Partial | No |
| Implementation complexity | Medium | Low | **Medium** | High |
| Cancellation support | Yes | No | Manual | Via LangChain |
| Matches voice_mapper.py | No (different arch) | No | **Yes (extends existing threads)** | No (rewrite) |

**Recommendation**: **Pattern 3 (Thread+Topic)** — extends `voice_mapper.py`'s existing threaded architecture naturally. The VLM decision thread runs async API calls while the robot moves on the last command. Results are published to an internal queue or topic consumed by the safety executor. This is what Phase 5 already concluded with the "5-6 thread" architecture.

Pattern 1 (Action) is a good alternative if we later extract the LLM decision-making into a separate ROS2 node. Pattern 4 (ReAct) is overkill for a 0.2-0.5 Hz control loop but could be useful for higher-level mission planning (multi-room exploration strategy).

#### 6.4 Sensor Data Packaging for LLMs — How Projects Do It

| Project | Sensor Input to LLM | Format | Tokens/Call |
|---------|---------------------|--------|-------------|
| VLFM | RGB image + frontier locations | Image + text list of frontier coords | ~500-1000 |
| Berkeley Frontier+VLM | Top-down obstacle map + first-person RGB | Two images + text prompt | ~1000-2000 |
| SayCan | Text state description | Structured text (objects, locations) | ~200-500 |
| NaVid | Video frames (multiple RGB) | Multi-image sequence | ~1000-3000 |
| LM-Nav | CLIP embeddings (not raw images) | Embedding vectors (not tokens) | N/A (embedding space) |
| CoW | Single RGB image | Image + object name text | ~500-800 |
| VoxPoser | RGB-D point cloud → code | Text description + code prompt | ~500-1000 |
| OrionNav | Scene graph from RGB-D + LiDAR | Structured text (nodes/edges) | ~300-800 |
| COME-Robot | RGB + depth + occupancy map | Images + structured text + API docs | ~1000-2000 |
| AsyncVLA | RGB camera image | Image + text instruction | ~500-1000 |
| ROSGPT | None (text commands only) | Pure text | ~100-300 |
| ROS-LLM | JSON action descriptions | Structured JSON | ~200-400 |
| **Our design (Phase 2-3)** | **Annotated RGB (depth overlay) + 12-sector LiDAR + odometry + memory** | **Image + structured text** | **~900-1,700** |

**Key observation**: Our approach of annotating the RGB image with depth overlay points and combining with a text-based LiDAR sector summary is **novel** — no surveyed project fuses camera images with text-format LiDAR data. VLFM and Berkeley use separate images (top-down map), while others use either pure text or pure vision. Our approach is cost-effective (one image + ~200 tokens of structured text) and information-dense.

#### 6.5 Frameworks and Libraries

##### 6.5.1 LangChain for Robotics

- **Used by**: ROSA (NASA JPL), robochain, langchain_agent_robot_controller_ros2, KIOS
- **Key features for robotics**:
  - `@tool` decorator turns Python functions into LLM-callable tools
  - ReAct agent pattern for multi-step reasoning
  - Memory (conversation buffer, summary, entity) for maintaining context
  - Structured output parsing with Pydantic models
  - Supports OpenAI, Anthropic, Google, local models
- **Pros**: Rich ecosystem, well-documented, active community
- **Cons**: Abstraction overhead (3-5 layers deep), version churn (breaking changes), not designed for real-time robotics, ReAct pattern requires multiple LLM calls per decision
- **Assessment for our project**: **Not recommended for the main control loop** (too heavy, too many LLM round-trips). Could be useful for a separate mission-planning node that operates at a lower frequency (once per room, not once per decision).

##### 6.5.2 LangGraph for Robotics

- **What it is**: LangChain's framework for stateful, multi-step agent workflows as directed graphs
- **Architecture**: State (shared data) → Nodes (Python functions) → Edges (conditional transitions)
- **Key features**: State machine semantics, branching logic, memory injection, tool chaining
- **Robotics applicability**: Could model exploration state machine (exploring → investigating → backtracking → mapping) as a LangGraph graph
- **Pros**: Clean state machine model, visual debugging, built-in checkpointing
- **Cons**: Same overhead as LangChain (it extends LangChain), requires LLM call at each node transition
- **Assessment for our project**: **Interesting for exploration strategy management**, but too heavy for the 0.2-0.5 Hz decision loop. Better suited as an outer planning layer that updates exploration goals every 30-60 seconds.

##### 6.5.3 Native Tool/Function Calling (Direct API)

- **What it is**: Using OpenAI/Anthropic/Google API tool-calling natively without a framework
- **Used by**: voice_mapper.py (current), ROSGPT-style projects
- **Key features**:
  - OpenAI: `tools` parameter with JSON schema, `tool_choice` for forcing calls
  - Anthropic: `tools` parameter with `input_schema`, supports `strict: true` for guaranteed schema compliance
  - Google: `function_declarations` with automatic grounding
  - All support structured output with schema validation
- **Pros**: Zero framework overhead, direct control over prompts/responses, lowest latency, easiest to debug
- **Cons**: Must implement retry logic, memory management, multi-step orchestration manually
- **Assessment for our project**: **Recommended for the main decision loop**. Already proven in `voice_mapper.py`. Anthropic's `strict: true` structured outputs guarantee valid JSON schema for robot commands — eliminates parsing failures. The 7-tool design from Phase 3 maps directly to native function calling.

##### 6.5.4 Anthropic Claude Structured Outputs for Robot Commands

Particularly relevant to our project: Claude's structured output mode with `strict: true` guarantees that tool call parameters always match the defined JSON schema exactly. This means:
- A `navigate` tool with `direction: enum["forward", "left", "right", "back"]` and `speed: enum["slow", "medium", "fast"]` will **never** produce invalid values
- No parsing fallbacks needed — eliminates an entire failure mode
- Type-safe from API response to robot command execution
- Works with Pydantic models for Python integration

This is a significant advantage over prompt-instructed JSON (used by ROSGPT and others), where the LLM might produce malformed JSON or out-of-schema values.

##### 6.5.5 No Robotics-Specific LLM Orchestration Framework Exists

Despite thorough searching, **there is no established framework specifically designed for real-time LLM-driven robot control loops**. The closest candidates are:
- **ROS-LLM** (Auromix) — most complete, but focused on task execution rather than continuous control
- **ROSA** — diagnostic tool, not a control framework
- **LangChain** — general-purpose, not robotics-optimized

This gap confirms that our custom approach (async thread + native tool calling + safety executor) is the correct path. A lightweight, purpose-built integration is better than shoehorning a general framework into a real-time control loop.

#### 6.6 Key Design Decisions — Resolved

##### Decision 1: Separate LLM Node vs. Embedded in Controller?

| Approach | Projects Using It | Pros | Cons |
|----------|------------------|------|------|
| Separate ROS2 node | llama_ros, ros2_nanollm, ROS-LLM | Clean separation, reusable, independent lifecycle | Extra IPC latency (~1-5ms), more complex deployment |
| Embedded in controller | voice_mapper.py, ROSGPT | Simple, low latency, shared state | Monolithic, harder to test, blocks if not threaded |

**Recommendation**: **Start embedded (Phase A-C), extract later (Phase D+)**. The existing `voice_mapper.py` already has the sensor subscriptions, Nav2 integration, and exploration logic. Adding a VLM decision thread is easier than creating a new node and wiring all the topics. Once stable, the VLM logic can be extracted into a standalone `llm_navigator` node for cleaner architecture.

##### Decision 2: Action vs. Service vs. Topic for LLM Communication?

Already resolved in Pattern analysis (section 6.3): **Internal thread + topic/queue** for the main loop, with the option to expose as an **Action server** when extracting to a separate node.

If extracting to a separate node, Action is the correct choice because:
- LLM inference is long-running (1-3 seconds)
- Supports cancellation (critical if the robot encounters an obstacle during inference)
- Supports feedback (could stream partial reasoning or confidence levels)
- llama_ros proves this pattern works well

##### Decision 3: How to Handle LLM Latency in Real-Time ROS2?

Resolved across Phases 4-5, confirmed by prior art:

1. **Asynchronous inference** — LLM runs on a separate thread/process (Pattern 3). The robot continues moving on the last command. Validated by AsyncVLA, VLA-RAIL, NaVid.
2. **Reactive safety layer runs independently** — Collision Monitor at 10-20 Hz, emergency stop at 0.25m, regardless of LLM state. This is the Simplex Architecture pattern.
3. **Dead reckoning between decisions** — at 0.15 m/s with 2-5 second decision intervals, the robot travels 0.3-0.75m per decision. LiDAR safety ensures this is safe.
4. **Adaptive frequency** — trigger VLM calls on events (doorway detection, intersection, novelty) rather than fixed-rate. Saves 50-70% API costs.
5. **Graceful degradation** — if API latency exceeds threshold, fall back to frontier exploration (proven to work already).

##### Decision 4: Cloud API vs. Local Model vs. Hybrid?

| Approach | Latency | Cost | Quality | Used By |
|----------|---------|------|---------|---------|
| Cloud API only | 400-1500ms | $0.11-0.30/hr | Best (GPT-4o, Claude, Gemini) | ROSGPT, ROS-LLM |
| Local only | 50-200ms | $0 (power only) | Limited (small models) | llama_ros, ros2_nanollm |
| **Hybrid** | 50-200ms local + 400ms cloud | ~$0.08-0.15/hr | Best of both | **Our design** |

**Recommendation**: **Hybrid (Phase F)** — local VLM (Moondream2 via ros2_nanollm or llama_ros) provides 1-2 Hz scene descriptions and novelty scoring. Cloud VLM (Gemini 2.0 Flash or GPT-4o-mini) makes strategic decisions at 0.2-0.5 Hz, triggered when local VLM detects something interesting. This matches Phase 5's conclusion and is uniquely enabled by our Jetson Orin Nano hardware.

#### 6.7 Closest Existing Framework to Our Needs

Ranking projects by overall relevance to "LLM-driven autonomous exploration on a ROS2 mobile robot":

| Rank | Project | Why Close | What's Missing |
|------|---------|-----------|----------------|
| 1 | **VLFM** | VLM scores frontiers for navigation decisions; zero-shot; real-world tested | Object-goal only (not open exploration); no LiDAR fusion; no ROS2 package |
| 2 | **OrionNav** | ROS2 Nav2 + Jetson Orin + LiDAR + RGB-D + LLM; nearly identical hardware/software | Scene graph focus, not VLM vision; multi-robot scope adds complexity |
| 3 | **Berkeley Frontier+VLM** | VLM-guided exploration; dual-view (map+camera); chain-of-thought | 2025 tech report, no public code yet |
| 4 | **AsyncVLA** | Validates async VLM timing (0.2 Hz + fast local controller); 85% success at 6s delay | Navigation edition is driving-focused; no exploration/mapping |
| 5 | **ROS-LLM (Auromix)** | Full framework with actions/services, behavior trees, feedback | Framework overhead; not tuned for continuous navigation |
| 6 | **llama_ros** | ROS2-native LLM/VLM node; Action-based; runs on our hardware | Infrastructure only — no navigation logic |
| 7 | **ros2_nanollm** | Jetson Orin optimized; ROS2 VLM nodes | Basic topic-based; no decision-making logic |

**No single project does what we need.** The closest approach is combining VLFM's "score frontiers with a VLM" concept with our existing `voice_mapper.py` architecture, using llama_ros or ros2_nanollm for local inference and native API tool calling for cloud decisions.

#### 6.8 Concrete Implementation Plan

Based on all six phases of research, here is the phased implementation plan for LLM-driven autonomous exploration:

##### Phase A: Continuous Perception Loop (Week 1)

**Goal**: Replace the current 15-second `observe_and_speak` timer with a continuous sensor aggregation pipeline.

**Changes to `voice_mapper.py`**:
1. Create `SensorSnapshot` dataclass: annotated RGB image (640x480 with depth overlay), 12-sector LiDAR summary, odometry (x, y, heading), timestamp
2. New `perception_thread` runs at 2 Hz, assembles `SensorSnapshot` from latest subscribed data
3. Depth-on-RGB annotation: sample 7 depth points from stereo, overlay as colored circles on camera image (~5ms processing)
4. 12-sector LiDAR summary: divide 360° into 30° sectors, report min distance per sector + doorway gap detection
5. Snapshots placed in a `queue.Queue(maxsize=1)` (latest-only) for VLM consumption

**Validation**: Log sensor snapshots, verify image annotation quality, measure processing latency.

##### Phase B: VLM Decision Loop (Week 2)

**Goal**: Add async VLM decision-making that produces navigation commands from sensor data.

**Changes to `voice_mapper.py`**:
1. New `vlm_decision_thread` reads from the perception queue, calls cloud VLM API
2. System prompt from Phase 3: 7-section format with exploration objective, tool definitions, chain-of-thought requirement
3. 7 tools via native function calling: `navigate(direction, speed)`, `turn(angle)`, `investigate()`, `mark_area(label)`, `observe_scene()`, `report_finding(text)`, `request_help(reason)`
4. Structured output: `{"reasoning": "...", "tool_calls": [...], "exploration_memory_update": {...}}`
5. Start with Gemini 2.0 Flash (lowest latency, cheapest) as default, GPT-4o-mini as fallback
6. Fixed 0.2 Hz initially (one decision every 5 seconds)

**Validation**: Run in "shadow mode" — VLM makes decisions but frontier exploration still controls the robot. Log VLM decisions vs. actual frontier choices to compare.

##### Phase C: Safety Executor Integration (Week 3)

**Goal**: Wire VLM decisions through the safety layer before execution.

**New component**: `SafetyExecutor` class (not a separate node — embedded in `voice_mapper.py`):
1. Receives VLM navigation commands
2. Validates against LiDAR data (SayCan-style affordance check: is the commanded direction clear?)
3. Applies speed limits based on proximity to obstacles
4. Translates to `move()` / Nav2 goal calls
5. Reports execution result (success/blocked/modified) back to VLM context
6. Blocked-action memory: hash(direction+tool), 15s cooldown, max 2 retries

**Switch-over**: Add `--exploration-mode` flag: `frontier` (default, current) vs. `llm` (new VLM-driven). This enables A/B testing.

##### Phase D: Exploration Memory (Week 4)

**Goal**: Give the LLM persistent memory of explored areas and discoveries.

**New component**: `ExplorationMemory` class:
1. Room/area tracking: detected rooms, their contents, exploration completeness
2. Frontier history: which frontiers were explored, what was found
3. Dead-end tracking: areas that led nowhere (avoid re-visiting)
4. Interesting findings: objects, features, unusual observations
5. Serialized as JSON (~200 tokens), included in every VLM prompt
6. Persisted to disk for session resume

**Validation**: Run 10-minute exploration sessions, compare area coverage and discovery quality vs. frontier-only.

##### Phase E: Adaptive Frequency & Event Triggers (Week 5)

**Goal**: Switch from fixed-rate VLM calls to event-triggered decisions.

**Triggers for VLM call**:
1. Doorway/intersection detected (LiDAR sector analysis shows opening)
2. Novel scene detected (local VLM or image similarity threshold)
3. Navigation blocked or goal reached
4. Timer-based fallback if no trigger for 5 seconds
5. Voice command received

**Expected result**: ~800 VLM calls/hr instead of 1,800/hr at fixed 0.5 Hz. Estimated 50-70% cost reduction.

##### Phase F: Multi-Model Architecture (Week 6+)

**Goal**: Add local VLM for fast scene awareness, using cloud VLM only for strategic decisions.

**Architecture**:
1. Local: Moondream2 (1.86B) via llama_ros or ros2_nanollm, running at 1-2 Hz
   - Scene description: "hallway with doors on left, open area ahead"
   - Novelty scoring: 0.0-1.0 (triggers cloud VLM when high)
   - Obstacle classification: "chair," "wall," "door" (richer than LiDAR alone)
2. Cloud: Gemini 2.0 Flash or GPT-4o-mini, triggered by local novelty or events
   - Strategic decisions: which direction to explore, what to investigate
   - Higher-level reasoning about exploration progress
3. Fallback: If cloud unavailable >10s, local VLM provides basic direction decisions

**VRAM budget**: Moondream2 (~1.5GB) + SLAM (~0.5GB) + Nav2 (~0.3GB) + OAK-D driver (~0.5GB) = ~2.8GB of 8GB. Feasible with margin.

#### 6.9 A/B Testing: LLM vs. Frontier Exploration

**Metrics**:
1. **Area coverage rate** (m²/minute) — how fast does the robot map the environment?
2. **Coverage completeness** (% of reachable area mapped in 10 minutes)
3. **Discovery quality** — number of distinct objects/features identified (LLM only)
4. **Stuck rate** — how often does the robot get stuck or revisit the same area?
5. **Cost** — API spend per exploration session
6. **Human preference** — blind evaluation of exploration videos by 3+ reviewers

**Protocol**:
1. Same environment, same starting position, 10-minute sessions
2. Three modes: (A) pure frontier, (B) LLM-driven, (C) hybrid (LLM picks among frontier candidates)
3. Five runs per mode (to account for stochastic LLM behavior)
4. Record: full ROS2 bag, VLM call logs, final map, exploration trace

**Hypothesis**: Mode C (hybrid) will match or exceed Mode A's coverage rate while producing richer environment descriptions, at the cost of ~$0.15-0.30 per session.

---

## Overview

This research established the complete technical foundation for LLM-driven autonomous robot exploration across six phases. The system architecture combines a cloud VLM (Gemini 2.0 Flash or GPT-4o-mini) making strategic navigation decisions at 0.2-0.5 Hz with a reactive safety layer running at 10-20 Hz, following the three-tier architecture (Deliberative/Sequencer/Controller). Sensor data is presented to the VLM as an annotated camera image (with depth overlay) plus a 12-sector LiDAR text summary — a novel fusion approach not found in existing projects. Navigation decisions use native tool/function calling with 7 discrete tools, chain-of-thought reasoning, and structured exploration memory. A local VLM (Moondream2 on Jetson) provides fast scene awareness as a pre-filter. The safety executor validates all LLM commands against real-time sensor data before execution, with graceful degradation to proven frontier exploration when the cloud API is unavailable. No existing framework fully addresses this use case — the implementation will be custom-built on the existing `voice_mapper.py` architecture, with VLFM's frontier-scoring concept and ROS-LLM's action-description pattern as key inspirations.

## Key Findings

### Phase 1 Discoveries
- The LLM brain never sees camera images — it only gets text context with 3 LiDAR floats
- Exploration observations (GPT-4o vision every 15s) are spoken aloud but never influence navigation
- Three independent GPT-4o call sites exist with no shared context or coordination
- Frontier exploration is effective but cannot leverage visual reasoning
- `llm_robot_brain.py` already has multi-provider support (Anthropic, Ollama) but is unused
- Image cost is only 85 tokens per call at `detail: "low"` — frequent vision calls are feasible
- The exploration_loop thread is the natural insertion point for an LLM decision loop

### Phase 2 Discoveries
- **Annotated images >> raw images** for VLM navigation — overlaying depth + heading costs <5ms and zero extra tokens
- **3-float LiDAR summary is severely inadequate** — expanding to 12 sectors + doorway gaps (+85 tokens) provides 10× more spatial info
- **Depth-on-RGB is optimal**: sample 7 depth points, overlay on camera image = free distance grounding
- **Gemini 2.0 Flash ($0.11/hr) and GPT-4o-mini ($0.15/hr)** are the practical choices at 0.2 Hz
- **Claude images cost 5× more than OpenAI `detail:low`** (410 vs 85 tokens for 640×480)
- **0.2–0.5 Hz** is realistic for cloud VLMs; API latency is the bottleneck
- **Moondream2** (1.86B, ~1.5GB VRAM) is feasible as local pre-filter on Jetson Orin Nano
- **Total budget: ~900–1,700 tokens/cycle** — economically viable across all frequency targets
- **Key gap in open source**: no projects fuse LiDAR text + annotated camera images — this is a novel, high-potential approach

### Phase 3 Discoveries
- **Native tool/function calling >> prompt-instructed JSON** — strict schema enforcement eliminates parsing failures, supported by all 3 major providers
- **Discrete actions >> continuous values** — 7 out of 12 surveyed projects use discrete/enumerated action spaces for non-fine-tuned VLMs
- **Chain-of-thought is mandatory** — Social Nav (2026) proves reasoning prompts consistently outperform action-only, confirmed by 6 other projects
- **7 tools (4 action + 2 perception + 1 communication)** is the optimal scope, matching COME-Robot and VoxPoser API patterns
- **Current voice_mapper.py prompt has 5 critical gaps**: no reasoning, no camera in brain, continuous velocities, no exploration strategy, no safety rules
- **Exploration memory replaces conversation history** — structured JSON (rooms, frontiers, dead ends, action outcomes) instead of 4-message text window
- **Closed-loop feedback is essential** — action results (success/failure/override) must be fed back to the LLM every cycle
- **PIVOT's candidate arrow annotation** is a high-potential approach for directional decisions
- **Provider abstraction is feasible** — same 7 tools in OpenAI/Anthropic/Gemini formats with only wire-format differences
- **Total prompt budget ~1,015–1,340 tokens/cycle** — well within Phase 2's ~900–1,700 token envelope

### Phase 4 Discoveries
- **Three-layer architecture (3T) is the correct framework** — Deliberative (LLM at 0.2–0.5 Hz) → Sequencer (SafetyExecutor at 1–10 Hz) → Controller (Nav2 + Collision Monitor at 10–100 Hz)
- **Existing 4-layer safety stack is almost sufficient** — emergency stop (L0), move() avoidance (L1), Nav2 costmap (L2), Collision Monitor (L3) all exist. The missing piece is a Sequencer layer between the LLM and these controllers
- **SayCan affordance scoring adapts perfectly** — `score = P_LLM(action|task) × V(feasibility|LiDAR)` pre-filters the LLM toward safe directions without hard constraints. Include direction feasibility scores (0.1–1.0) in LLM context
- **Speed safety is a non-issue** — at 0.18 m/s with 10 Hz reactive layer, total stopping distance is ~4 cm. Braking physics are negligible for a ~3 kg robot. Current speed tiers (0.06/0.12/0.18) are well-justified
- **LLM latency ≠ safety risk** — the 1–3s decision time is irrelevant because Collision Monitor + LiDAR emergency stop run continuously at 10–20 Hz. Without reactive layer, max safe speed drops to ~0.06 m/s
- **Tiered degradation handles LLM unavailability** — stop (3s), local nav (10s), return home (30s). Exponential backoff is wrong for robot control. Existing frontier exploration is a proven Tier 3 fallback
- **Blocked-action memory prevents retry loops** — hash(direction+tool), 15s cooldown, max 2 retries. Critical for preventing the LLM from wasting 3+ decision cycles on the same blocked direction
- **Collision Monitor needs expansion** — add EmergencyStop (0.25m, hard stop) and SlowdownZone (0.50m, 40%) as config-only changes. Increase FootprintApproach TTC from 1.2s to 2.0s for LLM latency margin
- **Override feedback must be structured** — include original vs. executed command, reason, sensor state, and suggested alternatives in LLM result JSON
- **Geofencing via Nav2 keepout filters** — costmap filter plugin with LLM-definable zones. Cliff/stair detection requires hardware IR sensors ($2–5 each)
- **Simplex Architecture** (Sha, 2001) formalizes the "LLM proposes, safety disposes" switching — if LLM output leaves safety envelope, proven-safe controller takes over
- **No off-the-shelf LLM safety wrapper for ROS2 exists** — SafetyExecutor must be built as a new component

### Phase 5 Discoveries
- **API latency dominates the decision cycle (75-95%)** — local processing (sensor fusion + safety + parsing) totals only 7-32ms. Optimizing local code is negligible; optimizing API call frequency and timing is critical
- **Streaming is NOT useful for tool-call decisions** — partial JSON can't be parsed for early action; TTFT (model thinking time) is the bottleneck, not output speed
- **Asynchronous inference is mandatory** — VLM call must run on a separate thread while robot continues moving. Validated by AsyncVLA, VLA-RAIL, and VLASH (2024-2025)
- **Dead reckoning + LiDAR safety is sufficient between decisions** — at 0.15 m/s with 10 Hz safety, stopping distance is ~4cm vs. 25cm emergency zone. Safety margin is >6×
- **Adaptive event-triggered frequency saves 50-70% on API costs** — trigger on doorways/intersections/novelty instead of fixed-rate polling. ~800 calls/hr vs. 1,800/hr
- **0.2-0.5 Hz is the correct decision frequency** — consistent with NaVid (0.7 Hz), VLM-Social-Nav (0.5 Hz), AsyncVLA (0.2 Hz cloud tier)
- **Gemini 2.0 Flash wins on latency** — 250-300ms TTFT, ~400-600ms total, enabling up to 1.5 Hz. GPT-4o-mini second (300-500ms TTFT). Claude Haiku 4.5 third (400-600ms TTFT)
- **Prompt caching: 16-37% cost reduction** — Anthropic 90% read discount (best %), OpenAI 50% automatic (easiest), Google 75-90% with explicit caching
- **Stateless calls + structured memory >> rolling conversation** — explicit JSON memory (~200 tokens) beats 4-message history for caching, error handling, and simplicity
- **Local VLM pre-filter is feasible** — Moondream2 (~1.5GB) or Florence-2 (~1GB) fits alongside SLAM/Nav2/OAK-D on 8GB Jetson, providing 1-2 Hz scene awareness + novelty scoring
- **4-tier network degradation refined** — Normal → Elevated (slow down) → Degraded (local/frontier) → Offline (return home). Monitor via rolling API success rate, not just timeouts
- **Complete architecture: 5-6 threads** — Main (ROS2), Voice, Local Control (20 Hz), VLM Decision (async), optional Local VLM, Sensor Monitor. GIL not a concern (I/O + C extensions)

### Phase 6 Discoveries
- **No existing framework fully addresses LLM-driven autonomous exploration** — the closest are VLFM (frontier scoring) and ROS-LLM (action descriptions + feedback), but none combine VLM navigation decisions with real-time safety in a ROS2 mobile robot
- **VLFM's "score frontiers with a VLM" pattern is the most relevant prior art** (682 stars, ICRA 2024, deployed on Spot robot) — adapting from object-goal to curiosity-driven exploration is our key innovation
- **Berkeley 2025 paper validates our dual-view approach** — combining top-down obstacle maps + first-person camera views, with chain-of-thought prompting, for frontier exploration
- **Four ROS2+LLM integration patterns exist**: Action Server, Service, Thread+Topic, LangChain ReAct — **Thread+Topic (Pattern 3) is correct** for our async cloud API + continuous robot motion requirement
- **llama_ros (246 stars) and ros2_nanollm (184 stars) provide local VLM infrastructure for Jetson** — Action-based (llama_ros) is architecturally cleaner; Topic-based (ros2_nanollm) is simpler
- **ROSA (NASA JPL, 1,405 stars) demonstrates the @tool pattern cleanly** but is diagnostic, not a navigation controller
- **ROS-MCP-Server (1,031 stars) is the fastest-growing project** but MCP protocol is for interactive assistants, not real-time control
- **Native tool/function calling beats LangChain for our use case** — zero framework overhead, direct prompt control, Anthropic's `strict: true` guarantees valid schemas
- **LangChain/LangGraph are overkill for the 0.2-0.5 Hz decision loop** — multiple LLM round-trips per ReAct step is too slow and expensive; useful only for higher-level mission planning
- **Our sensor fusion approach (annotated RGB + text LiDAR) is novel** — no surveyed project combines camera images with text-format LiDAR sector data
- **Start embedded in voice_mapper.py, extract later** — adding a VLM decision thread is simpler than creating a new node; extraction to standalone node can happen after stabilization
- **OrionNav validates our hardware stack** — Jetson Orin + LiDAR + RGB-D + ROS2 Nav2 with LLM planning is proven. Their scene graph approach is complementary to our annotated-image approach
- **AsyncVLA proves multi-second VLM delays are safe** — 85% navigation success even with 6-second VLM latency when paired with a fast local controller. Directly validates our async architecture
- **COME-Robot confirms our prompt architecture** — system prompt with API docs + chain-of-thought + execution feedback is the established pattern for VLM-driven navigation
- **PaLM-E/RT-2 validate web knowledge transfer** — web-trained VLMs successfully transfer spatial reasoning to robot control; discrete/tokenized actions outperform continuous for prompted models
- **The dual-system pattern (slow VLM + fast local) is the dominant 2024-2025 architecture** — confirmed across AsyncVLA, NaVid, VLFM, OrionNav, and our design
- **6-phase implementation plan defined**: Perception Loop → VLM Decision Loop → Safety Executor → Exploration Memory → Adaptive Frequency → Multi-Model Architecture — minimum viable at Phase C (week 3)
- **A/B testing protocol established** — compare frontier-only vs. LLM-only vs. hybrid (LLM picks among frontiers) across area coverage, discovery quality, stuck rate, and cost

## Actionable Conclusions

### Architecture
1. **Use the Thread+Topic pattern (Pattern 3)** — async VLM inference on a dedicated thread, results published to an internal queue consumed by the SafetyExecutor. This extends `voice_mapper.py`'s existing threaded model naturally.
2. **Start embedded, extract later** — implement Phases A-C inside `voice_mapper.py`, then extract the VLM decision logic into a standalone `llm_navigator` ROS2 node using the Action interface (llama_ros pattern) once stable.
3. **Three-tier architecture is confirmed** — Deliberative (VLM, 0.2-0.5 Hz) → Sequencer (SafetyExecutor, 1-10 Hz) → Controller (Nav2 + Collision Monitor, 10-100 Hz). No existing framework provides this; it must be custom-built.

### LLM Integration
4. **Use native tool/function calling, not LangChain** — direct API calls with strict schema validation (Anthropic `strict: true` or OpenAI function calling). Zero framework overhead, proven in `voice_mapper.py`.
5. **Gemini 2.0 Flash as primary model** — lowest latency (250-300ms TTFT), cheapest ($0.11/hr at 0.2 Hz), with GPT-4o-mini as fallback. Claude for complex reasoning when needed.
6. **7 tools with discrete action spaces** — `navigate`, `turn`, `investigate`, `mark_area`, `observe_scene`, `report_finding`, `request_help`. Enum values, not continuous — eliminates VLM value errors.

### Sensor Data
7. **Annotated RGB + 12-sector LiDAR text** — a novel, cost-effective approach. One image (~85-410 tokens depending on provider) + ~200 tokens structured text per decision cycle.
8. **SayCan-style affordance pre-scoring** — include LiDAR-derived direction feasibility scores (0.1-1.0) in the prompt so the VLM is biased toward safe directions.

### Safety & Latency
9. **Reactive safety runs independently of LLM** — Collision Monitor at 10-20 Hz, emergency stop at 0.25m. LLM latency (1-3s) is never a safety risk.
10. **Graceful degradation to frontier exploration** — if cloud API fails for >10s, fall back to the proven frontier-based system. No new code needed for the fallback.

### Implementation Priority
11. **Minimum viable product at Phase C (week 3)** — Perception Loop + VLM Decision Loop + Safety Executor. This produces a working LLM-driven explorer.
12. **Phase F (local VLM) is the highest-impact optimization** — Moondream2 on Jetson provides 1-2 Hz scene awareness at zero API cost, triggering cloud VLM only for strategic decisions. Expected 50-70% cost reduction.

## Open Questions
- ~~What LLM provider to use?~~ → Phase 2: Gemini 2.0 Flash or GPT-4o-mini for routine, GPT-4o/Claude Sonnet for complex
- ~~Acceptable cost per hour?~~ → Phase 2: $0.11–$0.15/hr at 0.2 Hz is practical
- ~~Jetson Orin Nano VLM feasibility?~~ → Phase 2: Moondream2 (1.86B, ~1.5GB) is feasible as pre-filter
- ~~Prompt format?~~ → Phase 3: 7-section system prompt with tool/function calling, chain-of-thought, exploration memory
- ~~Action space design?~~ → Phase 3: Discrete direction/speed enums via 7 native tools, not continuous velocities
- ~~Single-step or multi-step output?~~ → Phase 3: Single immediate action + optional plan context (SayCan pattern)
- How to handle multi-floor or outdoor environments? (needs IMU integration)
- Privacy implications of streaming camera feeds to cloud APIs?
- Exact LiDAR-to-camera extrinsic calibration for projected distance arcs?
- ~~What happens when the VLM API is temporarily unreachable?~~ → Phase 4: 4-tier graceful degradation (continue→stop→local nav→return home), fixed timeouts not backoff
- ~~How to handle network outages gracefully?~~ → Phase 5: 4-tier degradation with proactive WiFi monitoring, fixed-interval retry (not exponential backoff)
- ~~What decision frequency is needed?~~ → Phase 5: 0.2-0.5 Hz adaptive, event-triggered at doorways/intersections
- ~~Can we use streaming for faster action execution?~~ → Phase 5: No — tool-call JSON can't be partially parsed; TTFT dominates
- ~~Token cost per hour at different frequencies?~~ → Phase 5: $0.12-0.30/hr (Gemini/GPT-4o-mini) with adaptive caching
- ~~How to A/B test LLM navigation vs frontier exploration performance?~~ → Phase 6: Three-mode protocol (frontier/LLM/hybrid), 5 runs each, 10-minute sessions, metrics defined
- Should `observe_scene` use the same VLM or a cheaper secondary model?
- How to detect room transitions (entering/exiting rooms) for exploration memory updates?
- ~~Which existing framework is closest to what we need?~~ → Phase 6: VLFM (frontier scoring concept) + ROS-LLM (action descriptions), but no single framework suffices — custom build required
- ~~What's the minimum viable version?~~ → Phase 6: Phase C (Perception + VLM Decision + Safety Executor) at week 3

## Standards Applied
- PCH Research methodology (phased, documented, one-phase-per-session)
- ROS2 Humble conventions
- Existing `voice_mapper.py` architecture as baseline

## Handoff
After research completion, findings should be handed to `/pch-planner` for
implementation planning. The planner should use this research to design concrete
code changes to `voice_mapper.py` and potentially new ROS2 nodes.
