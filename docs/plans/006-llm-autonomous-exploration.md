---
id: "006"
type: plan
title: "LLM-Driven Autonomous Exploration Navigation"
status: "✅ Complete"
created: "2026-02-25"
owner: pch-planner
version: 0.1
research: "docs/research/007-llm-autonomous-exploration-decisions.md"
---

## Introduction

This plan implements LLM-driven autonomous exploration for the ROSMASTER A1 robot,
replacing the current algorithmic frontier-based exploration with a VLM (Vision-Language
Model) that continuously perceives the environment and makes navigation decisions.
Based on the comprehensive research in document 007, the system uses a three-tier
architecture: a cloud VLM for strategic decisions (0.2-0.5 Hz), a SafetyExecutor
for command validation (1-10 Hz), and the existing Nav2/LiDAR reactive safety layer
(10-100 Hz).

The key innovation is a novel sensor fusion approach — annotated RGB camera images
(with depth overlay and heading indicator) combined with 12-sector LiDAR text summaries
— fed to the VLM via native tool/function calling with 7 discrete navigation tools.

## Planning Session Log

| # | Question | Decision | Rationale | Date |
|---|----------|----------|-----------|------|
| 1 | Implementation scope | **C — Full Cloud System (Phases A-E)** | Phases A-E form a self-contained unit: perceive, reason, act safely, remember, and call VLM efficiently. Phase F (local VLM) deferred to separate plan — different concern (on-device ML deployment). | 2026-02-25 |
| 2 | LLM provider strategy | **D — GPT-4o-mini via existing OpenAI SDK** | Reuses existing OpenAI dependency and API key. GPT-4o-mini for navigation decisions, GPT-4o for existing voice/observe calls. No provider abstraction — minimal code change. Provider switching deferred. | 2026-02-25 |
| 3 | Integration approach | **B — Separate module files, imported by voice_mapper.py** | Keeps concerns separated (testable, readable) without ROS2 topic-wiring overhead. Modules access sensor state via passed references. voice_mapper.py grows ~100-200 lines of wiring, not 1,000+. | 2026-02-25 |
| 4 | Local VLM inclusion | **Deferred** | Already resolved by Q1 — Phase F (local VLM) excluded from this plan's scope. | 2026-02-25 |
| 5 | Voice thread coexistence | **B — Voice overrides LLM navigation** | Preserves existing voice UX completely. LLM nav pauses on voice command, resumes after. No refactor of voice pipeline. Brief overlap of two OpenAI calls is harmless (thread-safe client). | 2026-02-25 |
| 6 | Exploration mode switching | **C — "Explore" always uses LLM, frontier is fallback** | Once LLM nav is stable, "explore" triggers LLM loop. Frontier retained only as Tier 3 graceful degradation fallback (API unavailable >10s). Simplest long-term UX. | 2026-02-25 |

### Review Session Log

| # | Question | Decision | Rationale | Date |
|---|----------|----------|-----------|------|
| R1 | Movement execution model (CRITICAL-1: Tasks 3.5 vs 4.3 conflict) | **A — Velocity-target model** | SafetyExecutor translates VLM decisions into velocity targets (Twist + duration). The 20 Hz local control loop is the sole cmd_vel publisher, applying targets with real-time speed scaling and obstacle checks. `navigate_to_goal` sets nav2-active state (control loop yields). `observe_scene` sets paused state. Single publisher eliminates all cmd_vel conflicts. Task 3.5 acceptance criteria updated: `_execute_move` returns `(Twist, duration)` tuple instead of calling `vm.move()`. | 2026-02-25 |
| R2 | Speed scaling extraction (MAJOR-3: duplication risk) | **A — Extract helper method** | Create `_compute_safe_velocity(self, linear, angular)` on VoiceMapper encapsulating safety logic from `move()` lines 2050-2078 (obstacle stop, proportional slowdown, Ackerman constraint). Both `move()` and `llm_control_loop()` call it. Single source of truth for speed scaling thresholds. Task 4.3 references this helper. | 2026-02-25 |
| R3 | Battery field & room detection (MINOR-1 + MINOR-3) | **A — Remove battery, VLM-driven rooms** | Drop `battery_pct` from `robot_state` (no battery monitoring in codebase). Rooms created when VLM calls `report_discovery(significance="room")` — VLM names rooms from visual context (e.g., "kitchen"). No automatic doorway-triggered heuristic. ExplorationMemory creates new room entry on report_discovery with significance="room". | 2026-02-25 |
| R4 | SSIM implementation approach (MINOR-4) | **A — Simplified MAD** | Use mean absolute difference on 160x120 grayscale frames, normalized to 0-1. ~3 lines of numpy. Rename function from `compute_ssim` to `compute_frame_similarity` to avoid implying full SSIM. Threshold recalibrated: similarity > 0.92 becomes difference < 0.08. Fast (<1ms), sufficient for coarse "has scene changed" gate. | 2026-02-25 |

## Holistic Review

### Decision Interaction Analysis

1. **Q2 (GPT-4o-mini) × Q3 (separate modules)**: Using the existing OpenAI SDK with no
   provider abstraction means `llm_navigator.py` directly imports `openai`. If a provider
   switch is needed later, only this one file changes. The module separation (Q3) actually
   *reduces* the cost of not having an abstraction layer (Q2) — the blast radius is contained.

2. **Q5 (voice overrides) × Q6 (LLM always for explore)**: These work well together. The
   user says "explore" → LLM navigation starts. They can interrupt with any voice command
   (which pauses LLM nav, runs the command, resumes). They say "stop" → everything halts.
   The frontier fallback (Q6) only activates on API failure, not user choice — so the user
   never needs to think about which mode they're in.

3. **Q1 (Phases A-E) × Q2 (GPT-4o-mini)**: Phase E (adaptive frequency) becomes more
   important with GPT-4o-mini than it would be with Gemini 2.0 Flash, because GPT-4o-mini
   is slightly more expensive ($0.15/hr vs $0.11/hr at 0.2 Hz). The SSIM frame skipping and
   event triggers in Phase E reduce this gap significantly.

### Architectural Assessment

**Strengths:**
- Fully backward-compatible — existing frontier exploration, voice commands, and SLAM all
  unchanged. New code only activates when explore_mode="llm".
- Thread-safe by design — VLM decisions pass through a Queue, voice override via Event.
  No new shared mutable state beyond the queue.
- Graceful degradation is *free* — the existing `exploration_loop()` is already proven and
  becomes the Tier 3 fallback with zero additional code.

**Risks to watch:**
- The 20 Hz local control loop (llm_control_loop) duplicates some logic from the existing
  `move()` method (proportional speed scaling). Phase 4 task 4.3 should reference the
  existing code rather than reimplementing — extract the scaling logic into a shared helper
  if needed.
- VoiceMapper is already ~2,900 lines. Even with separate modules, the wiring code in
  Phase 4 adds ~150 lines. The class is accumulating responsibilities. Future plans should
  consider extracting VoiceMapper into smaller components (this is out of scope here).

### Gap Analysis

- **No unit test plan**: The phases include "verification" sections but no formal test files.
  Tests can be added incrementally as modules are implemented. Each module (sensor_snapshot,
  llm_navigator, safety_executor, exploration_memory) is independently testable by design.
- **No deployment script update**: The new Python files need to be deployed to the robot
  alongside voice_mapper.py. The existing `run_voice_mapper.sh` should work unchanged since
  imports happen from the same directory, but verify that `robot_scripts/` rsync includes
  the new files.
- **No Nav2 config changes**: The research recommends expanding Collision Monitor with
  EmergencyStop and SlowdownZone — this is explicitly out of scope but noted as a
  recommended follow-up.
- **Room detection is VLM-driven** (resolved R3): ExplorationMemory tracks rooms but room
  creation is triggered by VLM calling `report_discovery(significance="room")`. The VLM
  names rooms from visual context (e.g., "kitchen"). No automatic doorway heuristic —
  the VLM's visual understanding is more reliable than gap-detection heuristics.

## Overview

Replace the algorithmic frontier-based exploration in `voice_mapper.py` with an
LLM-driven navigation loop where GPT-4o-mini continuously perceives the environment
(camera + LiDAR + depth + odometry) and decides where to explore next. The system
uses native OpenAI tool/function calling with 7 discrete navigation tools, a
SafetyExecutor that validates all commands against real-time sensor data before
execution, and structured exploration memory that tracks rooms, frontiers, and
dead ends across the session.

**Objectives:**
1. Give the LLM direct navigation authority during exploration (replacing frontier algorithm)
2. Fuse camera imagery with LiDAR/depth data for VLM consumption (annotated images)
3. Maintain all existing safety guarantees (10 Hz reactive layer unchanged)
4. Gracefully degrade to frontier exploration when API is unavailable
5. Keep existing voice command system fully functional (voice overrides LLM nav)
6. Enable adaptive VLM call frequency based on environmental triggers

**Key files created:**
- `scripts/sensor_snapshot.py` — Image annotation, 12-sector LiDAR summary, state assembly
- `scripts/llm_navigator.py` — VLM decision loop, prompt building, tool definitions
- `scripts/safety_executor.py` — Command validation, blocked-action memory, watchdog
- `scripts/exploration_memory.py` — Room/frontier/dead-end tracking, serialization

**Key files modified:**
- `scripts/voice_mapper.py` — Wire new modules, add LLM explore mode, voice coexistence

## Requirements

### Functional Requirements

- **FR-1**: When user says "explore", start LLM-driven exploration (VLM decision loop)
- **FR-2**: VLM receives annotated camera image (depth overlays + heading indicator) + 12-sector LiDAR text summary + robot state + exploration memory each decision cycle
- **FR-3**: VLM outputs navigation decisions via OpenAI tool/function calling with 7 tools: `move_toward`, `navigate_to_goal`, `rotate`, `stop_robot`, `observe_scene`, `check_path_clear`, `report_discovery`
- **FR-4**: SafetyExecutor validates all VLM commands against LiDAR data before execution; rejects or modifies unsafe commands and reports override to VLM
- **FR-5**: Exploration memory tracks discovered rooms, frontiers, dead ends, and recent action history; serialized as JSON in each VLM prompt (~200 tokens)
- **FR-6**: Voice commands pause LLM navigation, execute via existing `think()` → `execute()` pipeline, then resume LLM navigation
- **FR-7**: System prompt includes chain-of-thought reasoning instruction, exploration strategy, safety rules, and dynamic sensor context
- **FR-8**: Blocked-action memory prevents VLM from retrying the same blocked direction more than 2 times within 15 seconds
- **FR-9**: LLM watchdog implements tiered degradation: continue at reduced speed (0-3s), stop and wait (3-10s), fall back to frontier exploration (10-30s), return to start (>30s)
- **FR-10**: Adaptive VLM call frequency triggers on doorway detection, intersection detection, dead-end detection, and a 5-second base timer fallback
- **FR-11**: MAD-based frame similarity skipping avoids redundant VLM calls when scene hasn't changed (similarity > 0.92, i.e. MAD < 0.08)
- **FR-12**: Affordance scores (direction feasibility 0.1-1.0 from LiDAR) included in VLM context to bias toward safe directions

### Non-Functional Requirements

- **NFR-1**: VLM decision latency < 3s median (GPT-4o-mini with image)
- **NFR-2**: Local sensor processing (image annotation + LiDAR summary + state assembly) < 30ms
- **NFR-3**: API cost < $0.30/hour at 0.2 Hz base rate
- **NFR-4**: Existing safety guarantees unchanged — emergency stop at 0.3m, obstacle avoidance at 0.5m, all operating at 10-20 Hz independent of VLM
- **NFR-5**: No new Python package dependencies (uses existing `openai`, `numpy`, `cv2`)
- **NFR-6**: Robot continues moving between VLM decisions (dead reckoning + LiDAR safety)
- **NFR-7**: Each new module file < 500 lines; voice_mapper.py net addition < 200 lines

### Out of Scope

- Local VLM on Jetson (Moondream2/Florence-2) — deferred to Plan 007
- Multi-provider abstraction (Anthropic, Gemini) — deferred
- PIVOT-style candidate arrow annotations on camera image — deferred
- LiDAR polar plot image as second VLM input — deferred
- Geofencing / keepout zone management — deferred
- IR cliff sensor hardware integration — deferred
- Nav2 Collision Monitor config expansion (EmergencyStop/SlowdownZone) — separate config change
- A/B testing infrastructure — deferred
- IMU subscription and pitch/roll integration — deferred
- Multi-floor or outdoor navigation — deferred

## Technical Design

### Architecture

Three-tier architecture with 5 threads:

```
┌─────────────────────────────────────────────────────────┐
│ MAIN THREAD (10 Hz): rclpy.spin_once()                  │
│   ROS2 callbacks: camera, depth, LiDAR, odom, map       │
│   (unchanged from current)                               │
├─────────────────────────────────────────────────────────┤
│ VOICE THREAD: voice_loop() [line 2701, unchanged]        │
│   listen() → think() → execute()                         │
│   Sets self.llm_nav_paused = True during voice command   │
│   Resumes LLM nav after execute() returns                │
├─────────────────────────────────────────────────────────┤
│ LOCAL CONTROL LOOP (20 Hz): llm_control_loop() [NEW]     │
│   ├── Read latest sensor data from VoiceMapper attrs     │
│   ├── Continue dead reckoning (last VLM velocity cmd)    │
│   ├── Apply proportional speed scaling from existing     │
│   │   move() logic (lines 2050-2074)                     │
│   ├── Check for new VLM decision in queue                │
│   │   └── If available: pass to SafetyExecutor           │
│   ├── Confidence-based speed decay if VLM delayed        │
│   └── Publish cmd_vel                                    │
├─────────────────────────────────────────────────────────┤
│ VLM DECISION THREAD: vlm_decision_loop() [NEW]           │
│   ├── Wait for trigger or 5s base timer                  │
│   ├── Check if paused (voice override or stopped)        │
│   ├── SensorSnapshot.capture() — annotated image + state │
│   ├── Optional SSIM check — skip if scene unchanged      │
│   ├── LLMNavigator.decide() — OpenAI API call            │
│   ├── Post decision to thread-safe queue                 │
│   └── ExplorationMemory.update() with result             │
├─────────────────────────────────────────────────────────┤
│ SENSOR MONITOR THREAD (0.5 Hz): existing + network check │
│   ├── LiDAR/odom/camera health [existing, line 2722]     │
│   └── NetworkMonitor.update() [NEW — API health track]   │
└─────────────────────────────────────────────────────────┘
```

### Data Models

#### SensorSnapshot (dataclass)

```python
@dataclass
class SensorSnapshot:
    """Immutable snapshot of all sensor data for one VLM decision cycle."""
    timestamp: float                    # time.time()
    annotated_image_b64: str            # 640x480 JPEG with depth overlay + heading, base64
    lidar_summary: str                  # 12-sector text (000° front: 2.9m CLEAR ...)
    doorway_gaps: list[dict]            # [{direction_deg, width_m, distance_m}, ...]
    robot_state: dict                   # {x, y, heading_deg, heading_cardinal, speed_mps,
                                        #  distance_traveled_m, explore_time_s}
    affordance_scores: dict[str, float] # {forward: 0.9, left: 0.1, ...}
    previous_action_result: dict | None # {success, actual_distance_m, stopped_reason, ...}
```

#### ExplorationState (dataclass)

```python
@dataclass
class ExplorationState:
    """Persistent exploration memory serialized into VLM prompt."""
    rooms: list[dict]          # [{name, explored_pct, objects, entered_from}, ...]
    current_room: str          # "hallway", "kitchen", etc.
    dead_ends: list[dict]      # [{direction_deg, reason, position}, ...]
    frontiers: list[dict]      # [{id, direction_deg, distance_m, type, width_m}, ...]
    recent_actions: list[dict] # Last 5: [{action, params, result, position}, ...]
    total_discoveries: int
    exploration_duration_s: float
```

#### NavigationDecision (dataclass)

```python
@dataclass
class NavigationDecision:
    """Parsed result from a VLM tool call."""
    tool_name: str              # "move_toward", "rotate", etc.
    parameters: dict            # Tool-specific args
    reasoning: str              # VLM's chain-of-thought text (logged, not executed)
    timestamp: float
    raw_response: dict          # Full OpenAI response for debugging
```

### API Contracts (Tool Definitions)

7 tools via OpenAI function calling (`tools` parameter, `strict: true`):

#### Action Tools (4)

**1. `move_toward`** — Move in a direction at a speed
```json
{
  "direction": {"type": "string", "enum": ["forward", "forward_left", "left", "forward_right", "right", "backward"]},
  "speed": {"type": "string", "enum": ["slow", "medium", "fast"]},
  "duration_s": {"type": "number", "minimum": 1.0, "maximum": 8.0}
}
```
Speed mapping: slow=0.08, medium=0.12, fast=0.18 m/s.
Direction mapping: forward→(speed,0.0), forward_left→(speed,+0.25), left→(0.06,+0.45), etc.

**2. `navigate_to_goal`** — Delegate to Nav2 for longer distances
```json
{
  "goal_type": {"type": "string", "enum": ["frontier", "coordinates", "retrace"]},
  "frontier_id": {"type": "integer"},
  "x": {"type": "number"},
  "y": {"type": "number"},
  "reason": {"type": "string"}
}
```

**3. `rotate`** — Turn in place (Ackerman-adapted arc turn)
```json
{
  "degrees": {"type": "integer", "enum": [-180, -135, -90, -45, 45, 90, 135, 180]}
}
```

**4. `stop_robot`** — Intentional halt
```json
{
  "reason": {"type": "string", "enum": ["obstacle_detected", "exploration_complete", "need_to_observe", "user_requested", "uncertain"]}
}
```

#### Perception Tools (2)

**5. `observe_scene`** — Request detailed visual description
```json
{
  "focus": {"type": "string", "enum": ["general", "doorways", "obstacles", "objects", "floor_surface"]}
}
```
Triggers a secondary GPT-4o call with `detail: "high"` for enhanced analysis.

**6. `check_path_clear`** — Query traversability (local sensor only, no API call)
```json
{
  "direction": {"type": "string", "enum": ["forward", "forward_left", "left", "forward_right", "right", "backward"]},
  "distance_m": {"type": "number", "minimum": 0.5, "maximum": 5.0}
}
```
Returns: `{clear: bool, obstacle_distance_m: float, obstacle_type: string}`

#### Communication Tool (1)

**7. `report_discovery`** — Announce and log a finding
```json
{
  "label": {"type": "string"},
  "notes": {"type": "string"},
  "significance": {"type": "string", "enum": ["landmark", "room", "obstacle", "point_of_interest"]}
}
```

### Components

#### 1. `scripts/sensor_snapshot.py` (~200 lines)

| Class/Function | Purpose |
|----------------|---------|
| `SensorSnapshot` | Dataclass holding one cycle's sensor data |
| `SensorSnapshotBuilder` | Builds snapshots from VoiceMapper state refs |
| `annotate_image(image, depth_points, heading_deg)` | OpenCV: overlay 7 depth values + heading arrow on 640x480 RGB |
| `build_lidar_summary(ranges, angle_min, angle_inc)` | 12-sector text + qualitative labels (CLEAR/NEAR/OBSTACLE/WALL) |
| `compute_affordance_scores(obstacle_distances)` | Direction feasibility 0.1-1.0 from LiDAR distances |
| `compute_frame_similarity(frame_a, frame_b)` | MAD-based frame similarity for frame skipping (0-1, higher=more similar) |

Accesses VoiceMapper via passed references: `self.vm.latest_image`, `self.vm.latest_depth`,
`self.vm.obstacle_distances`, `self.vm.current_position`, `self.vm.latest_scan`, `self.vm.detected_gaps`.

#### 2. `scripts/llm_navigator.py` (~350 lines)

| Class/Function | Purpose |
|----------------|---------|
| `LLMNavigator` | Main class: builds prompts, calls OpenAI, parses tool calls |
| `_build_system_prompt()` | 7-section static prompt (~260 tokens): role, perception format, exploration strategy, safety rules, reasoning instruction |
| `_build_user_message(snapshot, memory)` | Dynamic context block + camera image (~400-600 tokens) |
| `_get_tool_definitions()` | Returns 7 tool schemas for OpenAI `tools` parameter |
| `decide(snapshot, memory)` | Full decision cycle: build messages → call API → parse → return NavigationDecision |
| `_parse_tool_call(response)` | Extract tool name + args from OpenAI response |
| `TOOL_DEFINITIONS` | Class constant: list of 7 tool dicts |
| `SYSTEM_PROMPT` | Class constant: static system prompt text |

Uses `openai.OpenAI` client (same as voice_mapper.py). Model: `gpt-4o-mini`.

#### 3. `scripts/safety_executor.py` (~250 lines)

| Class/Function | Purpose |
|----------------|---------|
| `SafetyExecutor` | Validates + executes VLM commands, manages watchdog |
| `execute_decision(decision, vm)` | Main entry: validate → execute → record result |
| `_pre_validate(decision, obstacle_distances)` | Check LiDAR clearance for commanded direction |
| `_execute_move(params, vm)` | Translate move_toward → vm.move() call |
| `_execute_navigate(params, vm)` | Translate navigate_to_goal → vm.navigate_to() |
| `_execute_rotate(params, vm)` | Translate rotate → arc-based turn |
| `_execute_stop(params, vm)` | Publish zero twist |
| `_execute_observe(params, vm)` | Call vm.observe() with focus hint |
| `_execute_check_path(params, vm)` | Query LiDAR arc in direction |
| `_execute_report(params, vm)` | Log to DiscoveryLog + TTS |
| `BlockedActionMemory` | Tracks blocked directions, 15s cooldown, max 2 retries |
| `LLMWatchdog` | Tiered timeout: continue(0-3s), stop(3-10s), local_nav(10-30s), return(>30s) |
| `NetworkMonitor` | Rolling API call success rate + latency tracking |

Accesses VoiceMapper methods: `vm.move()`, `vm.navigate_to()`, `vm.observe()`,
`vm.emergency_stop()`, `vm.speak()`, `vm.obstacle_distances`.

#### 4. `scripts/exploration_memory.py` (~150 lines)

| Class/Function | Purpose |
|----------------|---------|
| `ExplorationMemory` | Maintains and serializes exploration state |
| `ExplorationState` | Dataclass: rooms, frontiers, dead_ends, recent_actions |
| `update_after_action(decision, result, position)` | Update memory with action outcome |
| `record_discovery(label, notes, significance, position)` | Add to room's object list |
| `update_frontiers(frontier_list)` | Refresh from latest SLAM analysis |
| `record_dead_end(direction, position)` | Mark direction as dead end |
| `to_prompt_text()` | Serialize to ~200 token text block for VLM prompt |
| `save_to_disk(path)` / `load_from_disk(path)` | Session persistence (YAML) |

#### 5. Changes to `scripts/voice_mapper.py` (~150 lines added)

| Change | Location | Description |
|--------|----------|-------------|
| Import new modules | Top of file | `from sensor_snapshot import ...`, etc. |
| Init new components | `__init__` (~line 360) | Instantiate SensorSnapshotBuilder, LLMNavigator, SafetyExecutor, ExplorationMemory |
| `llm_control_loop()` | New method | 20 Hz local control: dead reckoning + safety + dequeue VLM decisions |
| `vlm_decision_loop()` | New method | Async VLM thread: trigger → snapshot → decide → enqueue |
| `start_llm_exploration()` | New method | Start both new threads, set self.exploring=True, self.explore_mode="llm" |
| `stop_llm_exploration()` | New method | Signal threads to stop, clean up |
| Modify `execute()` | Line 2192 | Add `explore_llm` action that calls `start_llm_exploration()` |
| Modify `exploration_loop()` | Line 2338 | This becomes the Tier 3 fallback (called by watchdog, not by "explore" command) |
| Modify `voice_loop()` | Line 2701 | Set `self.llm_nav_paused=True` before think(), `False` after execute() |
| Add `self.llm_nav_paused` | `__init__` | Threading event for voice override coordination |
| Add `self.vlm_decision_queue` | `__init__` | `queue.Queue(maxsize=1)` for VLM→control thread communication |

## Dependencies

### Existing (No New Packages)

- `openai` — Already used for GPT-4o in voice_mapper.py. GPT-4o-mini uses same SDK + API key
- `numpy` — LiDAR sector analysis, SSIM computation
- `cv2` (opencv-python) — Image annotation (putText, arrowedLine, circle)
- `queue` — Standard library, thread-safe Queue for decision passing
- `threading` — Standard library, Events for pause/resume coordination
- `dataclasses` — Standard library, for SensorSnapshot, ExplorationState, NavigationDecision
- `json` — Standard library, for tool definition serialization
- `yaml` — For exploration memory persistence (already used by DiscoveryLog)
- `math` — Heading conversion, TTC computation

### External Services

- **OpenAI API** — GPT-4o-mini for navigation decisions (~$0.15-0.30/hour)
- **WiFi connectivity** — Required for cloud VLM; graceful degradation when unavailable

### Prerequisites

- Plans 001-005 completed (OAK-D Pro camera, SLAM, Nav2, exploration mode all working)
- Robot connected to WiFi with internet access
- `OPENAI_API_KEY` set in environment (already required for existing voice_mapper.py)

## Risks

| # | Risk | Probability | Impact | Mitigation |
|---|------|------------|--------|------------|
| R1 | GPT-4o-mini navigation quality insufficient for indoor exploration | Medium | High | Tiered degradation to proven frontier exploration. Can swap model string to GPT-4o if needed (higher cost). |
| R2 | API latency spikes (>5s) cause jerky exploration behavior | Medium | Medium | Dead reckoning + confidence-based speed decay between decisions. Watchdog stops robot at 3s, falls back at 10s. |
| R3 | VLM repeatedly chooses blocked directions, wastes decision cycles | Medium | Low | BlockedActionMemory suppresses retries. Affordance scores bias toward clear directions. |
| R4 | WiFi signal loss in far rooms | Low | High | 4-tier degradation: frontier fallback at 10s, return-to-start at 30s. |
| R5 | voice_mapper.py integration breaks existing functionality | Low | High | New modules are imported but only activated when explore_mode="llm". Existing frontier/voice paths unchanged. |
| R6 | Thread safety issues between VLM thread and voice thread | Medium | Medium | VLM decision queue (thread-safe Queue). Voice pause via threading.Event. No shared mutable state beyond existing sensor attrs (already safe under GIL). |
| R7 | Token budget exceeds GPT-4o-mini context window | Very Low | Medium | Total ~1,700 tokens/cycle well within 128K context. Stateless calls (no growing history). |
| R8 | Annotated image quality insufficient for VLM spatial reasoning | Low | Medium | Depth overlay + heading indicator proven effective in research. Can fall back to raw image + text-only depth summary. |

## Execution Plan

### Phase 1: Sensor Snapshot Pipeline (Small — 5 tasks, 2 files)

**Goal**: Create `sensor_snapshot.py` with image annotation, 12-sector LiDAR summary,
affordance scoring, and SSIM computation. Wire into voice_mapper.py for testing.

**Prerequisites**: Plans 001-005 complete. Camera, LiDAR, depth, odom all publishing.

**Entry point**: Create new file `scripts/sensor_snapshot.py`.

| # | Task | File(s) | Status | Acceptance Criteria |
|---|------|---------|--------|-------------------|
| 1.1 | Create `SensorSnapshot` dataclass and `SensorSnapshotBuilder.__init__()` that accepts VoiceMapper reference | `scripts/sensor_snapshot.py` | ✅ Complete | Dataclass with fields: timestamp, annotated_image_b64, lidar_summary, doorway_gaps, robot_state, affordance_scores, previous_action_result. Builder stores ref to vm. |
| 1.2 | Implement `annotate_image(image_msg, depth_msg, heading_deg, heading_cardinal)` — sample 7 depth points from stereo depth, overlay as white text with black outline on 640x480 RGB, add heading arrow + cardinal text in top-right corner | `scripts/sensor_snapshot.py` | ✅ Complete | Uses existing `cv_bridge` pattern from `image_to_base64()` (line 1876). Depth sampling uses existing `get_dist(x, y)` pattern (line 1179) — 5x5 median at 7 points: center(320,240), left-center(160,240), right-center(480,240), floor-ahead(320,400), upper-left(160,120), upper-right(480,120), upper-center(320,100). Heading arrow drawn with `cv2.arrowedLine()`. Output: base64 JPEG at quality 80. Processing < 15ms. |
| 1.3 | Implement `build_lidar_summary(ranges, angle_min, angle_inc)` — divide 360° into 12 sectors (30° each), compute 10th percentile distance per sector, assign qualitative labels (CLEAR >2m, NEAR <2m, OBSTACLE <1m, WALL <0.5m), append doorway gap info | `scripts/sensor_snapshot.py` | ✅ Complete | Follows existing `scan_callback()` sector analysis pattern (line 998) but expands from 6→12 sectors. Text format matches research doc 007 Phase 2 section 2.3. Returns multi-line string ~100-120 tokens. |
| 1.4 | Implement `compute_affordance_scores(obstacle_distances)` and `compute_frame_similarity(frame_a, frame_b)` | `scripts/sensor_snapshot.py` | ✅ Complete | Affordance: maps 6 directions to 0.1-1.0 scores based on LiDAR clearance (blocked<0.5m→0.1, clear>2.0m→1.0, linear interpolation between). Frame similarity: downsample to 160x120 grayscale, compute mean absolute difference normalized to 0-1 (0.0=identical, 1.0=completely different). Returns similarity score (1.0 - MAD). No scipy dependency — ~3 lines of numpy. |
| 1.5 | Implement `SensorSnapshotBuilder.capture()` — assemble full snapshot from VoiceMapper state, including robot state dict (position, heading, speed, distance_traveled, explore_time) | `scripts/sensor_snapshot.py` | ✅ Complete | Reads: `vm.latest_image`, `vm.latest_depth`, `vm.obstacle_distances`, `vm.current_position`, `vm.latest_scan`, `vm.detected_gaps`, `vm.latest_odom`. Heading converted from radians→degrees + cardinal. Returns frozen `SensorSnapshot`. |

**Verification**: Import from voice_mapper.py, call `capture()` in a test method, log the
annotated image to disk, verify depth overlays and heading arrow are visible, verify LiDAR
summary text is well-formed.

---

### Phase 2: LLM Navigator Core (Medium — 6 tasks, 2 files)

**Goal**: Create `llm_navigator.py` with system prompt, tool definitions, OpenAI API
integration, and response parsing. Create `exploration_memory.py` with state tracking.

**Prerequisites**: Phase 1 complete (SensorSnapshot available).

**Entry point**: Create new files `scripts/llm_navigator.py` and `scripts/exploration_memory.py`.

| # | Task | File(s) | Status | Acceptance Criteria |
|---|------|---------|--------|-------------------|
| 2.1 | Create `ExplorationMemory` class with `ExplorationState` dataclass, `update_after_action()`, `record_discovery()`, `update_frontiers()`, `record_dead_end()` | `scripts/exploration_memory.py` | ✅ Complete | State tracks: rooms (name, explored_pct, objects, entered_from), current_room, dead_ends (direction, reason, position), frontiers (id, direction, distance, type, width), recent_actions (last 5 with outcomes). **Room creation**: new rooms are created when VLM calls `report_discovery(significance="room")` — the label becomes the room name (e.g., "kitchen"), position is recorded as entry point. `current_room` updates to the new room. No automatic doorway-triggered room detection. Initial state: one room named "start" at robot's origin. |
| 2.2 | Implement `ExplorationMemory.to_prompt_text()` — serialize state to ~200 token text block matching research doc 007 Phase 3 section 3.19 format | `scripts/exploration_memory.py` | ✅ Complete | Output format: `EXPLORATION MEMORY:\n  Rooms: hallway (80% explored)...\n  Current room: ...\n  Frontiers: ...\n  Dead ends: ...\n  Recent: [5] move_toward(forward) → success, +0.36m\n  ...`. Also `save_to_disk()` / `load_from_disk()` via YAML. |
| 2.3 | Create `LLMNavigator` class with `TOOL_DEFINITIONS` constant — 7 tools in OpenAI function calling format with `strict: true` | `scripts/llm_navigator.py` | ✅ Complete | All 7 tools defined per API Contracts section. Each tool has name, description, parameters with JSON Schema, required fields, `additionalProperties: false`, `strict: true`. |
| 2.4 | Implement `LLMNavigator._build_system_prompt()` — static 7-section prompt (~260 tokens) | `scripts/llm_navigator.py` | ✅ Complete | Sections: (1) Role & identity, (2) Perception format, (3) Exploration strategy, (4) Safety rules, (5) Reasoning instruction. Content from research doc 007 Phase 3 sections 3.18. Stored as class constant. |
| 2.5 | Implement `LLMNavigator._build_user_message(snapshot, memory)` — dynamic context block with image | `scripts/llm_navigator.py` | ✅ Complete | Assembles: robot state text + LiDAR summary + affordance scores + previous action result + exploration memory text. Camera image attached as OpenAI image_url content part (base64 data URI, `detail: "low"`). Total dynamic content ~400-600 tokens. |
| 2.6 | Implement `LLMNavigator.decide(snapshot, memory)` — full decision cycle with OpenAI API call and `_parse_tool_call()` | `scripts/llm_navigator.py` | ✅ Complete | Calls `openai.chat.completions.create()` with model=`gpt-4o-mini`, messages=[system, user], tools=TOOL_DEFINITIONS, tool_choice="required". Timeout 8s. Parses response into `NavigationDecision` dataclass. Handles API errors gracefully (returns None). Logs reasoning text + tool call for debugging. |

**Verification**: Call `decide()` with a test SensorSnapshot, verify it returns a valid
NavigationDecision with one of the 7 tool names and valid parameters.

---

### Phase 3: Safety Executor (Medium — 7 tasks, 1 file)

**Goal**: Create `safety_executor.py` with command pre-validation, blocked-action memory,
execution dispatch, watchdog, and network monitoring.

**Prerequisites**: Phase 2 complete (NavigationDecision available).

**Entry point**: Create new file `scripts/safety_executor.py`.

| # | Task | File(s) | Status | Acceptance Criteria |
|---|------|---------|--------|-------------------|
| 3.1 | Create `BlockedActionMemory` class — tracks blocked actions by hash(tool+direction), 15s cooldown, max 2 retries | `scripts/safety_executor.py` | ✅ Complete | `should_allow(decision)` returns (bool, warning_msg). `record_blocked(decision)` increments counter. Expired entries auto-cleaned on access. Hash key: `f"{tool_name}:{direction_or_degrees}"`. |
| 3.2 | Create `LLMWatchdog` class — evaluates elapsed time since last VLM response, returns tier string | `scripts/safety_executor.py` | ✅ Complete | `evaluate(elapsed_s)` returns: "CONTINUE" (0-3s), "STOP_WAIT" (3-10s), "LOCAL_NAV" (10-30s), "RETURN_HOME" (>30s). Configurable thresholds. |
| 3.3 | Create `NetworkMonitor` class — tracks rolling API call success rate and latency | `scripts/safety_executor.py` | ✅ Complete | `record_call(latency_ms, success)` appends to deque(maxlen=20). `current_tier` property: 0=normal, 1=elevated (avg>3s), 2=degraded (no success in 10s), 3=offline (no success in 30s). |
| 3.4 | Create `SafetyExecutor.__init__()` and `_pre_validate(decision, obstacle_distances)` — check LiDAR clearance for commanded direction | `scripts/safety_executor.py` | ✅ Complete | Maps tool direction to LiDAR sector(s). If clearance < 0.5m for move_toward, reject with reason. If clearance < 1.0m, downgrade speed to "slow". Returns (valid: bool, modified_decision, reason). |
| 3.5 | Implement action execution methods: `_execute_move()`, `_execute_navigate()`, `_execute_rotate()`, `_execute_stop()` | `scripts/safety_executor.py` | ✅ Complete | Each translates NavigationDecision params → velocity targets (non-blocking). `_execute_move`: maps direction enum→(linear, angular) pairs, speed enum→m/s, returns `VelocityTarget(twist, duration_s)` for the control loop to apply. `_execute_rotate`: returns `VelocityTarget` with angular component + minimum forward speed (Ackerman constraint). `_execute_navigate`: calls `vm.navigate_to(x, y)` and returns `NavGoalTarget(x, y)` — control loop yields cmd_vel to Nav2 while `self.navigating` is True. `_execute_stop`: returns `StopTarget(reason)`. All return via `ExecutionResult` dataclass with target type and pre-validation outcome. |
| 3.6 | Implement perception/communication execution: `_execute_observe()`, `_execute_check_path()`, `_execute_report()` | `scripts/safety_executor.py` | ✅ Complete | `_execute_observe`: returns `ObserveTarget(focus)` — control loop sets robot to stopped state, spawns background thread calling `vm.observe()` (existing GPT-4o vision, line 1890), result is stored for inclusion in next VLM decision cycle. `_execute_check_path`: queries LiDAR arc in specified direction ± 30° (local sensor only, instant), returns {clear, obstacle_distance_m, obstacle_type}. `_execute_report`: calls `vm.speak()` + logs to DiscoveryLog + calls `memory.record_discovery()` (non-blocking, speak is async). |
| 3.7 | Implement `SafetyExecutor.execute_decision(decision, vm, memory)` — main entry point that chains: blocked-action check → pre-validate → translate to target | `scripts/safety_executor.py` | ✅ Complete | Returns a target object (`VelocityTarget`, `NavGoalTarget`, `ObserveTarget`, or `StopTarget`) that the control loop applies. Does NOT directly publish cmd_vel or call blocking VoiceMapper methods (except `navigate_to` for Nav2 goals and `speak` which is async). On block/reject, records to BlockedActionMemory and returns `StopTarget` with reason. Updates NetworkMonitor on API health. |

**Verification**: Unit test with mock VoiceMapper — verify blocked actions are suppressed,
unsafe directions are rejected, speed is downgraded near obstacles, result dict format is correct.

---

### Phase 4: Integration & Wiring (Medium — 9 tasks, 1 file)

**Goal**: Wire all new modules into `voice_mapper.py`. Add VLM decision thread, local
control loop, voice override coordination, and explore mode switching.

**Prerequisites**: Phases 1-3 complete (all modules tested independently).

**Entry point**: Modify `scripts/voice_mapper.py`.

| # | Task | File(s) | Status | Acceptance Criteria |
|---|------|---------|--------|-------------------|
| 4.1 | Add imports and instantiate new components in `__init__()` (~line 360) | `scripts/voice_mapper.py` | ✅ Complete | Import SensorSnapshotBuilder, LLMNavigator, SafetyExecutor, ExplorationMemory. Instantiate with `self` as VoiceMapper reference. Add `self.llm_nav_paused = threading.Event()`, `self.vlm_decision_queue = queue.Queue(maxsize=1)`, `self.llm_exploring = False`, `self.last_vlm_velocity = Twist()`. |
| 4.2 | Extract `_compute_safe_velocity(self, linear, angular)` helper from `move()` (lines 2050-2078) | `scripts/voice_mapper.py` | ✅ Complete | New method encapsulates: emergency stop check, multi-sector front obstacle check (stop if < `min_obstacle_dist`), backward obstacle check, proportional slowdown (speed_factor from `min_obstacle_dist` to `slow_dist`), Ackerman minimum forward speed constraint. Returns `(actual_linear, actual_angular, should_stop, stop_reason)`. Refactor existing `move()` to call this helper. |
| 4.3 | Implement `vlm_decision_loop(self)` — async VLM decision thread | `scripts/voice_mapper.py` | ✅ Complete | Loop: wait for trigger (5s timer) → check paused → capture snapshot → frame similarity check (skip if >0.92) → call navigator.decide() → enqueue decision → update memory. Runs until `self.llm_exploring = False`. Handles API errors (log + continue). Updates `self.safety_executor.network_monitor` with call results. |
| 4.4 | Implement `llm_control_loop(self)` — 20 Hz local control thread, sole cmd_vel publisher during LLM exploration | `scripts/voice_mapper.py` | ✅ Complete | Loop at 50ms interval: (1) check queue for new VLM decision → pass to `safety_executor.execute_decision()` → (2) apply current target via `_compute_safe_velocity()` → (3) check watchdog tier → (4) check `self.llm_nav_paused` event. |
| 4.5 | Implement `start_llm_exploration(self)` and `stop_llm_exploration(self)` | `scripts/voice_mapper.py` | ✅ Complete | Start: set flags, reset memory/watchdog, start both threads. Stop: clear flags, publish zero Twist, join threads with timeout. |
| 4.6 | Modify `start_exploration()` and `execute()` explore handler to use LLM exploration | `scripts/voice_mapper.py` | ✅ Complete | `start_exploration()` calls `start_llm_exploration()`. `stop_exploration()` also stops LLM exploration. Existing `exploration_loop()` retained for watchdog fallback. |
| 4.7 | Modify `voice_loop()` for voice override coordination | `scripts/voice_mapper.py` | ✅ Complete | Before `think()`: if LLM exploring, set pause event + publish zero Twist. After `execute()`: clear pause event to resume. |
| 4.8 | Wire watchdog degradation — LOCAL_NAV falls back to frontier exploration | `scripts/voice_mapper.py` | ✅ Complete | In `llm_control_loop()`: STOP_WAIT publishes zero twist, LOCAL_NAV calls `_fallback_to_frontier()` which starts `exploration_loop()`, RETURN_HOME navigates to origin then falls back. |
| 4.9 | Wire `stop` voice command to also stop LLM exploration | `scripts/voice_mapper.py` | ✅ Complete | In `execute()` for `action == "stop"`: if `self.llm_exploring`, call `stop_llm_exploration()` before existing stop logic. |

**Verification**: Full integration test on robot:
1. Say "explore" → robot starts LLM-driven exploration
2. VLM makes navigation decisions visible in logs
3. Safety executor rejects unsafe commands
4. Say "stop" → robot stops
5. Say "look" during exploration → LLM nav pauses, observation runs, LLM nav resumes
6. Disconnect WiFi → robot falls back to frontier exploration within 10s

---

### Phase 5: Adaptive Frequency & Event Triggers (Small — 5 tasks, 2 files)

**Goal**: Replace fixed 5-second VLM call timer with event-triggered adaptive frequency.
Add doorway/intersection/dead-end triggers and SSIM-based frame skipping optimization.

**Prerequisites**: Phase 4 complete (integrated system working end-to-end).

**Entry point**: Modify `scripts/sensor_snapshot.py` and `scripts/voice_mapper.py`.

| # | Task | File(s) | Status | Acceptance Criteria |
|---|------|---------|--------|-------------------|
| 5.1 | Add `EventTrigger` class to sensor_snapshot.py — monitors LiDAR for doorway/intersection/dead-end events | `scripts/sensor_snapshot.py` | ✅ Complete | `check_triggers(obstacle_distances, detected_gaps, position, last_trigger_time)` returns list of trigger reasons. Triggers: doorway detected (gap ≥ 0.7m width), T-intersection (3+ open sectors adjacent), dead end (all forward sectors < 1.0m), position-based (moved >1.5m since last VLM call). Cooldown: no re-trigger within 2s. |
| 5.2 | Modify `vlm_decision_loop()` to use event-driven waiting instead of fixed 5s timer | `scripts/voice_mapper.py` | ✅ Complete | Replace `time.sleep(5.0)` with: check EventTrigger at 10 Hz → if triggered, immediately call VLM → else, if 5s elapsed since last call, call VLM (base rate fallback). Use `threading.Event` with timeout for efficient waiting. |
| 5.3 | Enhance frame similarity skipping — only skip when similarity > 0.92 AND no event trigger AND nearest obstacle > 2m | `scripts/voice_mapper.py` | ✅ Complete | In vlm_decision_loop: compute frame similarity (MAD-based) vs. last-sent frame. Skip VLM call if all three conditions met. Never skip if: event trigger fired, robot was recently stuck/overridden, or >10s since last VLM call (safety backstop). Log skip decisions for debugging. |
| 5.4 | Add text-only fallback for clear corridors — omit camera image when scene is static and path is clear | `scripts/llm_navigator.py` | ✅ Complete | In `_build_user_message()`: if caller passes `skip_image=True`, omit the image content part from the user message. Saves ~85 tokens per call. Called when frame similarity > 0.95 AND nearest obstacle > 3m. Note: `skip_image` support was already present in llm_navigator.py from Phase 2. |
| 5.5 | Add decision frequency metrics logging — track calls/minute, triggers/minute, skips/minute, avg latency | `scripts/voice_mapper.py` | ✅ Complete | Every 60 seconds, log: "VLM stats: X calls/min (Y triggered, Z skipped), avg latency Xms, tier N". Helps verify that adaptive frequency is working (target: ~800 calls/hr vs 1,800/hr fixed). |

**Verification**: Run 5-minute exploration session. Verify:
1. VLM calls cluster at doorways and intersections (from logs)
2. Corridor stretches show fewer calls (frame skipping active)
3. Total call count is 30-50% lower than Phase 4 fixed-rate
4. No degradation in navigation quality (robot still explores effectively)

## Complexity Assessment

| Factor | Score (1-5) | Notes |
|--------|-------------|-------|
| Files to modify | 2 | 4 new files + 1 modified (voice_mapper.py) |
| New patterns introduced | 4 | Velocity-target model, VLM decision loop, adaptive frequency triggers — new to this codebase |
| External dependencies | 3 | OpenAI API (existing) but new usage pattern (continuous VLM calls with images, tool calling) |
| Migration complexity | 1 | No data migration; existing behavior preserved, new code only activates in LLM explore mode |
| Test coverage required | 4 | Each module independently testable, but full integration requires robot hardware + API access |
| **Overall Complexity** | **14/25** | **Medium** — well-contained by modular design, but VLM decision loop + control loop threading is inherently complex |

## Review Summary

**Review conducted**: 2026-02-25
**Issues found**: 8 (1 Critical, 3 Major, 4 Minor) — all resolved
**Questions asked**: 4 — all answered
**Plan quality**: High after review

| Issue | Severity | Resolution |
|-------|----------|------------|
| Movement execution model conflict (Tasks 3.5 vs 4.3) | Critical | R1: Velocity-target model — SafetyExecutor returns targets, control loop is sole cmd_vel publisher |
| navigate_to_goal + control loop cmd_vel conflict | Major | Resolved by R1 — control loop yields to Nav2 when `self.navigating` is True |
| observe_scene blocking execution thread | Major | Resolved by R1 — ObserveTarget stops robot, runs observe in background thread |
| Speed scaling extraction unresolved | Major | R2: Extract `_compute_safe_velocity()` helper, both `move()` and control loop use it |
| Battery monitoring doesn't exist | Minor | R3: Removed `battery_pct` from SensorSnapshot |
| Task 4.5 references wrong method | Minor | R3: Task 4.6 now correctly targets `start_exploration()` at line 2682 |
| Room detection heuristic unspecified | Minor | R3: VLM-driven rooms via `report_discovery(significance="room")` |
| SSIM without scipy | Minor | R4: Simplified to MAD-based frame similarity, renamed function |

## Standards

- ROS2 Humble conventions
- Existing `voice_mapper.py` architecture patterns
- PCH planning methodology
- Research doc 007 findings

## Handoff

```
Plan Approved — Ready for Implementation

Plan at: docs/plans/006-llm-autonomous-exploration.md
Phases: 5
Tasks: 32 total (5 + 6 + 7 + 9 + 5)
Key decisions (planning):
  1. Scope: Phases A-E (full cloud system, local VLM deferred)
  2. Provider: GPT-4o-mini via existing OpenAI SDK (no abstraction layer)
  3. Structure: Separate module files imported by voice_mapper.py
  4. Voice: Voice overrides LLM navigation (pause/resume)
  5. Mode: "Explore" always uses LLM, frontier is fallback only
  6. Safety: Three-tier architecture with SafetyExecutor + existing reactive layer
Key decisions (review):
  R1. Velocity-target model — SafetyExecutor returns targets, control loop is sole cmd_vel publisher
  R2. Extract _compute_safe_velocity() helper from move() for shared speed scaling
  R3. Remove battery_pct, VLM-driven room creation via report_discovery
  R4. MAD-based frame similarity instead of full SSIM

New files: 4 (sensor_snapshot.py, llm_navigator.py, safety_executor.py, exploration_memory.py)
Modified files: 1 (voice_mapper.py — ~150 lines wiring)
New dependencies: None (all existing packages)

Review: 8 issues found (1 critical, 3 major, 4 minor) — all resolved
Plan Quality: High after review

Next Step: Use /pch-coder to begin implementing the plan.
```
