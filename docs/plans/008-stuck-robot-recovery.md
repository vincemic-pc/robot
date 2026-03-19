---
id: "008"
type: plan
title: "Stuck Robot Exploration Recovery"
status: ✅ Ready for Implementation
created: "2026-03-18"
owner: pch-planner
version: 1.0
research: "docs/research/009-stuck-robot-exploration-recovery.md"
---

## Introduction

This plan implements a robust stuck-robot recovery system for the LLM-driven autonomous exploration mode. Based on research document 009, the robot gets stuck indefinitely near obstacles because no layer in the system detects "robot is not making progress." The VLM sends decisions that get blocked by the safety executor, the auto-escape only goes backward and fails when cornered, the control loop has no reactive recovery, and the watchdog only monitors VLM health — not robot movement.

This plan addresses 10 findings across 4 files (`voice_mapper.py`, `safety_executor.py`, `velocity_arbiter.py`, `sensor_snapshot.py`) with a priority-ordered, phased implementation.

## Planning Session Log

| # | Question | Decision | Rationale | Date |
|---|----------|----------|-----------|------|
| 1 | Scope & phasing strategy | A — Full implementation (P0–P3), all 10 changes | Comprehensive fix addressing all identified failure modes in one plan | 2026-03-18 |
| 2 | Stuck detector position source | B — VSLAM odometry when available, fallback to wheel | More accurate absolute position; fallback ensures availability when VSLAM is down | 2026-03-18 |
| 3 | Stuck detector tier thresholds | C — Adaptive 8s/15s/30s with suspension during Nav2/observe | Fast response during VLM movement; avoids false positives during intentional slow ops | 2026-03-18 |
| 4 | Multi-strategy escape direction selection | C — Hybrid: affordance-ranked with pre-validation filter | Reuses existing affordance + pre-validation; guarantees chosen direction passes safety | 2026-03-18 |
| 5 | Arbiter escape threshold mechanism | D — Lower arbiter backward threshold globally to 0.3m | Arbiter becomes true emergency backstop; executor still validates at 0.5m for normal moves; zero message changes | 2026-03-18 |
| 6 | SLAM startup fix approach | D — Per-sensor camera-only grace period (10s after SLAM start) | Camera-specific suppression; LiDAR/odom monitoring stays active for safety | 2026-03-18 |
| 7 | Watchdog dual-feed architecture | A — Extend existing LLMWatchdog with `feed_success()` and `_last_success_time` | Smallest change; same class, 2 call sites to update; evaluate() returns worst tier across both feeds | 2026-03-18 |
| 8 | Stuck detector / VLM loop interaction | A — Shared `_stuck_state` flag + immediate VLM wake via `_vlm_trigger_event` | Both wakes VLM immediately and keeps it fast while stuck; uses existing event infrastructure | 2026-03-18 |
| 9 | Stale LiDAR affordance behavior | A — All zeros (0.0), every direction blocked | Safe failure mode; prevents VLM from requesting movement without spatial data; defense-in-depth | 2026-03-18 |
| 10 | Blocked-sector image visualization | B — Text-based legend bar with color-coded backgrounds | Simple OpenCV implementation; survives detail=low downscaling; doesn't occlude scene | 2026-03-18 |

## Review Session Log

| # | Question | Decision | Rationale | Date |
|---|----------|----------|-----------|------|
| R1 | Escape logic duplication — shared helper vs separate vs unified | A — Extract shared helper: `find_best_escape_direction()` + `validate_direction()` on SafetyExecutor | Eliminates direction-ranking duplication between `_maybe_escape()` and tier escapes; also resolves private `_pre_validate()` access issue; smallest change that addresses both concerns | 2026-03-18 |
| R2 | `_consecutive_blocked` reset after Tier 3 | B — Add public `reset_stuck_state()` to SafetyExecutor, called on stuck-clear transition | Clean public API; avoids reaching into private attributes; prevents surprising aggressive-escape after human repositions robot | 2026-03-18 |
| R3 | Task 4.4 wrong file — annotate_image caller is in sensor_snapshot.py, not voice_mapper.py | A — Fix file reference to sensor_snapshot.py:441, use `getattr(vm, 'obstacle_distances', None)` | Consistent with existing pattern; `capture_snapshot()` already reads `vm.obstacle_distances` elsewhere | 2026-03-18 |
| R4 | Batch minor corrections (4 items: 15Hz→10Hz, 3→4 files, docstring range, Task 2.3 line ref) | A — Apply all 4 corrections | Factual fixes; precise references prevent coder re-investigation | 2026-03-18 |

## Holistic Review

### Decision Interactions

1. **Q2 (VSLAM fallback) × Q3 (adaptive thresholds):** The stuck detector uses VSLAM position when available. If VSLAM goes stale mid-exploration, the detector falls back to wheel odometry. The position history must handle source switching without producing false displacement spikes — VSLAM and wheel odom may disagree on absolute position. **Mitigation:** When the source switches, clear the position history and restart the stuck timer. A source switch itself indicates sensor disruption, not genuine movement.

2. **Q4 (affordance-ranked escape) × Q5 (arbiter backward threshold):** The multi-strategy escape ranks directions by affordance then pre-validates. Lowering the arbiter's backward threshold to 0.3m means escape backward moves are no longer silently killed. But rotation escapes still creep forward at 0.05 m/s — if `front_wide < 0.3m`, the arbiter's emergency stop will zero that creep. This is acceptable because rotation's angular component is preserved during emergency stops (`result.angular.z = angular` in the arbiter). The robot still turns even if linear creep is zeroed.

3. **Q7 (watchdog dual-feed) × Q8 (VLM wake on stuck):** The watchdog's new `feed_success()` and the stuck detector both monitor "robot not making progress" but at different layers. The watchdog operates in the VLM loop context (decision-level), while the stuck detector operates in the control loop (position-level). They are complementary, not redundant: the watchdog triggers the existing fallback-to-frontier path, while the stuck detector triggers local reactive recovery. No conflict — but both should be documented clearly so future developers understand the two escalation paths.

4. **Q3 (Nav2 suspension) × Q8 (VLM wake):** The stuck detector suspends during Nav2 goals to avoid false positives. But if Nav2 itself gets stuck (e.g., costmap blocks the path), the stuck detector won't notice. **Mitigation:** Add a Nav2-specific timeout (e.g., 60s) — if `self.navigating` is True for >60s with no position change, resume stuck detection. This is already partially covered by the watchdog's `LOCAL_NAV` tier.

5. **Q9 (stale LiDAR → 0.0) × Q4 (affordance-ranked escape):** If LiDAR goes stale and affordances become all-0.0, the escape mechanism will find no passable direction and fall through to Tier 3 (help request). This is correct — without LiDAR, the robot should not attempt autonomous movement.

### Architectural Assessment

The changes span three files with clear responsibilities:
- **`safety_executor.py`**: Escape logic, watchdog, blocked counter (internal decision validation)
- **`voice_mapper.py`**: Stuck detector, VLM cadence, sensor monitor, action result enrichment (orchestration)
- **`velocity_arbiter.py`**: Backward threshold (final safety gate)
- **`sensor_snapshot.py`**: Affordance guard, image annotation (sensor processing)

No new files needed. No message definition changes. No new ROS2 topics or services. The changes are additive within existing class boundaries.

### Gap Analysis

1. **Testing:** No unit test infrastructure exists for these Python scripts (they're ROS2 nodes). Verification must be done via live testing on the robot with `journalctl` log analysis. Each phase should include specific log messages that confirm the new behavior is active.
2. **Logging:** All new behaviors (stuck detection, escape attempts, tier escalation) must log at WARNING or higher so they appear in `journalctl` without debug mode.
3. **Metrics:** The existing VLM metrics logging (`_log_vlm_metrics_if_due()`) should be extended to include stuck detection counts and escape success rates.
4. **Rollback:** If the stuck detector causes erratic behavior, the entire feature can be disabled by setting the Tier 1 threshold to a very high value (e.g., 9999s). No code rollback needed.

## Overview

### Feature Summary

Implement a three-tier escalating recovery system that detects stuck robots via odometry displacement, recovers using multi-strategy escape (rotation, lateral, backward), and requests human help when all automated recovery fails. Additionally fix the SLAM startup sensor monitor race, align arbiter/executor backward thresholds, and enhance the watchdog to detect stuck-with-healthy-VLM scenarios.

### Objectives

1. **P0 — Position-based stuck detector**: Detect stuck state in the control loop via odometry displacement, independent of VLM health
2. **P0 — Multi-strategy escape**: Replace backward-only escape with rotation-first, affordance-ranked escape
3. **P1 — Arbiter escape threshold alignment**: Prevent arbiter from killing escape moves
4. **P1 — Sensor monitor SLAM grace period**: Eliminate false camera-loss alarms during SLAM startup
5. **P1 — Watchdog dual-feed**: Detect stuck-with-healthy-VLM via execution success tracking
6. **P2 — Faster VLM cadence when stuck**: Reduce VLM interval from 15s to 3s during stuck state
7. **P2 — Populate actual_distance_m**: Give VLM feedback on how far the robot moved
8. **P2 — Fix cornered counter reset**: Don't reset `_consecutive_blocked` when escape fails
9. **P3 — Guard stale LiDAR affordances**: Prevent all-1.0 affordances on stale LiDAR data
10. **P3 — Blocked-sector image visualization**: Help VLM spatial reasoning with visual markers

## Requirements

### Functional Requirements

1. **FR-1:** The system shall detect when the robot has moved less than 0.05m over a rolling window while exploration is active, using VSLAM odometry when available and falling back to wheel odometry.
2. **FR-2:** Upon stuck detection, the system shall execute a three-tier escalating recovery: Tier 1 (rotate toward best direction at 8s), Tier 2 (try all passable directions at 15s), Tier 3 (stop and request human help at 30s).
3. **FR-3:** Stuck detection timers shall suspend while Nav2 navigation goals are active or while an observe action is in progress.
4. **FR-4:** The escape mechanism shall select directions by affordance score ranking, filtered through pre-validation to guarantee the chosen direction passes the safety executor.
5. **FR-5:** When cornered (all directions blocked), the escape mechanism shall NOT reset `_consecutive_blocked`, allowing the stuck detector to escalate to Tier 3 (help request).
6. **FR-6:** The arbiter shall use a 0.3m threshold (matching `emergency_distance`) for backward motion stops, allowing escape moves approved by the safety executor to execute.
7. **FR-7:** The sensor monitor shall suppress camera loss detection for 10 seconds after `start_slam()` is called, while continuing LiDAR and odometry monitoring.
8. **FR-8:** The watchdog shall track both VLM decision receipt and execution success, escalating when either feed times out.
9. **FR-9:** The VLM decision loop shall reduce its base interval from 15s to 3s while the stuck detector reports a non-NONE stuck state.
10. **FR-10:** The action result dict shall include `actual_distance_m` computed from odometry displacement during the movement execution.
11. **FR-11:** When `obstacle_distances` is empty (no LiDAR data), `compute_affordance_scores()` shall return 0.0 for all directions instead of 1.0.
12. **FR-12:** The annotated camera image shall include a text-based legend bar showing direction clearances with color-coded backgrounds (red=blocked, yellow=restricted, green=clear).

### Non-Functional Requirements

1. **NFR-1:** Stuck detector position sampling shall run at ≤1 Hz (not at the full 20 Hz control loop rate) to minimize CPU impact on the Jetson Orin Nano.
2. **NFR-2:** All new recovery behaviors shall log at WARNING level or higher for `journalctl` visibility.
3. **NFR-3:** The stuck detector, escape mechanism, and VLM cadence changes shall be independently disableable via constants at the top of each file.
4. **NFR-4:** No changes to the VelocityRequest.msg definition or any ROS2 interface.

### Out of Scope

1. Unit test infrastructure for ROS2 nodes (no existing framework to build on).
2. VLM model-level changes (prompt modifications, model switching).
3. Nav2 stuck detection or recovery (Nav2 has its own recovery behaviors).
4. Hardware-level fixes (LiDAR noise, IMU drift, motor calibration).
5. New ROS2 topics, services, or message definitions.

## Technical Design

### Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                     RECOVERY SYSTEM OVERVIEW                         │
│                                                                      │
│  VLM Decision Loop (10Hz poll / 15s base / 3s when stuck)           │
│    ├─ Reads self._stuck_state → reduces interval when stuck          │
│    └─ Woken by self._vlm_trigger_event on stuck detection            │
│                                                                      │
│  LLM Control Loop (20 Hz)                                            │
│    ├─ Dequeues VLM decisions → SafetyExecutor.execute_decision()     │
│    ├─ StuckDetector.update(position) — called every 1s               │
│    │   ├─ Tier 1 (8s): rotate toward best affordance direction       │
│    │   ├─ Tier 2 (15s): try all passable directions sequentially     │
│    │   └─ Tier 3 (30s): stop, speak, request human help              │
│    ├─ Feeds watchdog.feed_success() on execution success             │
│    └─ Suspended during Nav2 goals / observe actions                  │
│                                                                      │
│  SafetyExecutor                                                      │
│    ├─ _maybe_escape() → multi-strategy (affordance + pre-validate)   │
│    ├─ _consecutive_blocked: NOT reset when cornered                  │
│    └─ LLMWatchdog: dual-feed (decision + success)                    │
│                                                                      │
│  Velocity Arbiter                                                    │
│    └─ Backward threshold: 0.3m (was 0.5m)                           │
│                                                                      │
│  Sensor Monitor                                                      │
│    └─ Camera loss check suppressed for 10s after start_slam()        │
│                                                                      │
│  Sensor Snapshot                                                     │
│    ├─ compute_affordance_scores(): 0.0 on empty obstacle_distances   │
│    └─ annotate_image(): clearance legend bar at bottom               │
└──────────────────────────────────────────────────────────────────────┘
```

### Data Models

#### StuckDetector class (new, in `safety_executor.py`)

```python
class StuckDetector:
    """Position-based stuck detection with three-tier escalation.

    Samples odometry position at 1 Hz, maintains a rolling window,
    and reports stuck state when displacement < threshold.
    """

    # Tier thresholds (seconds of displacement < STUCK_DISPLACEMENT_M)
    TIER1_TIMEOUT_S = 8.0     # Rotate escape
    TIER2_TIMEOUT_S = 15.0    # Smart escape (try all directions)
    TIER3_TIMEOUT_S = 30.0    # Help request
    STUCK_DISPLACEMENT_M = 0.05  # 5cm — effectively stationary
    SAMPLE_INTERVAL_S = 1.0   # Position sample rate

    def __init__(self):
        self._positions: deque  # deque of (timestamp, x, y) tuples
        self._suspended: bool   # True during Nav2/observe
        self._last_sample_time: float
        self._stuck_since: Optional[float]  # timestamp when stuck started, or None

    def update(self, x: float, y: float, now: float = None) -> Optional[str]:
        """Sample position, return stuck tier or None.

        Returns: None, "TIER1", "TIER2", "TIER3"
        Called from control loop. Rate-limited internally to 1 Hz.
        """

    def suspend(self):
        """Pause stuck detection (Nav2 goal active, observe in progress)."""

    def resume(self):
        """Resume stuck detection."""

    def reset(self):
        """Clear position history and stuck state (e.g., after successful escape)."""

    @property
    def stuck_state(self) -> Optional[str]:
        """Current stuck tier or None."""
```

#### StuckState enum values (string constants)

| Value | Meaning | Triggered after |
|-------|---------|----------------|
| `None` | Not stuck | Displacement ≥ 0.05m in window |
| `"TIER1"` | Rotate escape | 8s stationary |
| `"TIER2"` | Smart escape | 15s stationary |
| `"TIER3"` | Help request | 30s stationary |

#### LLMWatchdog extensions (modified, in `safety_executor.py`)

```python
class LLMWatchdog:
    def __init__(self, continue_s=3.0, stop_s=10.0, local_nav_s=30.0,
                 success_timeout_s=30.0):  # NEW parameter
        self._last_response_time = time.time()
        self._last_success_time = time.time()  # NEW
        self._success_timeout_s = success_timeout_s  # NEW

    def feed(self):
        """Record VLM response received."""
        self._last_response_time = time.time()

    def feed_success(self):  # NEW
        """Record successful execution (movement actually happened)."""
        self._last_success_time = time.time()

    def evaluate(self, elapsed_s=None):
        """Return worst tier across both feeds."""
        # Existing logic for response feed
        # NEW: also check success feed timeout
```

#### Action result dict extensions (in `voice_mapper.py`)

```python
# Current (lines 2602-2606):
action_result = {
    'success': result.success,
    'safety_override': result.safety_override,
    'safety_message': result.safety_message,
}

# Extended:
action_result = {
    'success': result.success,
    'safety_override': result.safety_override,
    'safety_message': result.safety_message,
    'actual_distance_m': displacement,  # NEW — computed from odom
    'stopped_reason': result.stopped_reason,  # NEW — from ExecutionResult
}
```

### Component Design

#### 1. StuckDetector (new class in `safety_executor.py`)

**Location:** After `LLMWatchdog` class (~line 177), before `NetworkMonitor`.

**Position source:** Receives (x, y) from caller. The control loop in `voice_mapper.py` selects the source:
- If `self.vslam_tracking` and VSLAM odom fresh: use VSLAM position
- Else: use `self.current_position` (wheel odometry)

**Source-switch handling:** When the position source changes (VSLAM→wheel or wheel→VSLAM), call `reset()` to clear position history. This prevents false displacement from coordinate frame differences.

**Suspension logic:**
- `suspend()` called when `self.navigating` becomes True or `ObserveTarget` is assigned
- `resume()` called when `self.navigating` becomes False or observe completes
- While suspended, `update()` returns None and does not sample positions

**Tier evaluation:**
```
if suspended: return None
if len(positions) < 2: return None
displacement = distance(oldest_position, newest_position)
if displacement >= STUCK_DISPLACEMENT_M:
    reset stuck_since
    return None
if stuck_since is None:
    stuck_since = now
elapsed = now - stuck_since
if elapsed >= TIER3_TIMEOUT_S: return "TIER3"
if elapsed >= TIER2_TIMEOUT_S: return "TIER2"
if elapsed >= TIER1_TIMEOUT_S: return "TIER1"
return None
```

#### 2. Multi-Strategy Escape (replacing `_maybe_escape()` in `safety_executor.py`)

**Location:** `safety_executor.py:244–265`, full replacement.

**New signature:** `_maybe_escape(self, obstacle_distances) -> Optional[ExecutionResult]`

**Algorithm:**
```
1. If _consecutive_blocked < STUCK_ESCAPE_THRESHOLD: return None
2. Compute affordance scores for all 6 directions from obstacle_distances
3. Sort directions by affordance score (descending)
4. For each direction (best first):
   a. Build a synthetic NavigationDecision for that direction
   b. Call _pre_validate(decision, obstacle_distances)
   c. If valid:
      - If direction requires rotation (not current heading): return rotate target
      - Else: return move_toward target
      - Reset _consecutive_blocked = 0, clear blocked_memory
      - Log escape direction chosen
5. If NO direction passes pre-validation (cornered):
   - Do NOT reset _consecutive_blocked  ← KEY CHANGE (F2-6 fix)
   - Log "cornered — all directions blocked"
   - Return None (let stuck detector Tier 3 handle it)
```

**Direction-to-rotation mapping:**
| Best direction | Escape action |
|---------------|---------------|
| forward | move_toward forward (should be rare — would mean it's not actually blocked) |
| forward_left | rotate +45°, then move forward |
| forward_right | rotate -45°, then move forward |
| left | rotate +90° |
| right | rotate -90° |
| backward | move_toward backward at -0.08 m/s for 2s |

For rotation escapes, only the rotation is returned (not a subsequent forward move). The next VLM call or stuck detector cycle will handle forward movement after the rotation completes.

#### 3. Stuck Detector Integration in Control Loop (`voice_mapper.py`)

**Location:** `voice_mapper.py:2572–2700` (`llm_control_loop()`), after the watchdog check block (~line 2688).

**New instance variable:** `self._stuck_detector = StuckDetector()` (initialized alongside `safety_executor`)

**New instance variable:** `self._stuck_state = None` (shared with VLM loop)

**Per-cycle integration:**
```python
# After target application, before watchdog check:

# Select position source
if self.vslam_tracking and (time.time() - self.vslam_last_odom_time) < 2.0:
    pos_x, pos_y = self.vslam_position['x'], self.vslam_position['y']
else:
    pos_x, pos_y = self.current_position['x'], self.current_position['y']

# Suspend/resume based on state
if isinstance(self._current_target, NavGoalTarget) or self._observing:
    self._stuck_detector.suspend()
else:
    self._stuck_detector.resume()

# Update stuck detector
tier = self._stuck_detector.update(pos_x, pos_y)
self._stuck_state = tier  # shared with VLM loop

if tier == "TIER1":
    self._execute_tier1_escape()
elif tier == "TIER2":
    self._execute_tier2_escape()
elif tier == "TIER3":
    self._execute_tier3_help()
```

**Tier 1 escape (`_execute_tier1_escape`):**
- Compute affordance scores from `self.obstacle_distances`
- Find highest-affordance direction
- Build rotation toward that direction
- Publish via `_publish_velocity_request()` at PRIORITY_EXPLORATION
- Set `_vlm_trigger_event` to wake VLM loop
- Log at WARNING: "STUCK TIER1: rotating toward {direction} (affordance={score})"

**Tier 2 escape (`_execute_tier2_escape`):**
- Try all 6 directions in affordance order
- For each: build synthetic decision, call `safety_executor._pre_validate()`
- First direction that passes: execute 0.5s probe movement
- If position changes after probe: commit to 2s movement in that direction
- Set `_vlm_trigger_event` to wake VLM loop
- Log at WARNING: "STUCK TIER2: probing {direction}"

**Tier 3 help (`_execute_tier3_help`):**
- Publish zero velocity
- `self.speak("I'm stuck and can't find a way out. Please help me.")`
- Set `self._stuck_state = "TIER3"` (VLM loop reads this)
- Log at ERROR: "STUCK TIER3: requesting human help"
- Do NOT stop exploration — remain in the loop so manual repositioning + voice command can resume

#### 4. VLM Loop Cadence Acceleration (`voice_mapper.py`)

**Location:** `voice_mapper.py:2365` (base_interval), `voice_mapper.py:2390` (elapsed check).

**Change:** Read `self._stuck_state` at the top of each VLM loop iteration:
```python
effective_interval = 3.0 if self._stuck_state else base_interval
```

Replace `if elapsed >= base_interval` with `if elapsed >= effective_interval`.

#### 5. Camera Grace Period (`voice_mapper.py`)

**New instance variable:** `self._camera_monitor_suppress_until = 0.0`

**In `start_slam()` (~line 1420):** Before the 3s sleep:
```python
self._camera_monitor_suppress_until = time.time() + 10.0
```

**In `_sensor_monitor_loop()` (~line 3226):** Wrap the camera loss detection:
```python
if time.time() >= self._camera_monitor_suppress_until:
    # existing camera loss check
    if now - last_image_time > CAMERA_LOSS_TIMEOUT and not camera_lost:
        ...
```

#### 6. Arbiter Backward Threshold (`velocity_arbiter.py`)

**Location:** `velocity_arbiter.py:350–354`

**Change:** Replace `self.min_distance` with `self.emergency_distance` in the backward check:
```python
# Before:
if linear < 0 and back_dist < self.min_distance:
# After:
if linear < 0 and back_dist < self.emergency_distance:
```

This changes the backward stop from 0.5m to 0.3m. The safety executor still pre-validates backward at 0.5m for normal VLM-requested moves, so this only affects escape moves (which use a 0.3m threshold in the executor).

#### 7. Stale LiDAR Affordance Guard (`sensor_snapshot.py`)

**Location:** `sensor_snapshot.py:215–228` (`compute_affordance_scores()`)

**Change:** Add empty-dict guard at the top:
```python
def compute_affordance_scores(obstacle_distances):
    if not obstacle_distances:
        return {d: 0.0 for d in AFFORDANCE_DIRECTIONS}
    # ... existing logic
```

#### 8. Image Legend Bar (`sensor_snapshot.py`)

**Location:** `sensor_snapshot.py:108–159` (`annotate_image()`) — add after the heading arrow block, before JPEG encoding.

**Implementation:**
```python
# --- Clearance legend bar at bottom ---
if obstacle_distances:
    bar_y = 460  # 20px from bottom of 480px image
    bar_x = 10
    directions = ["F", "FL", "FR", "L", "R", "B"]
    keys = ["front", "front_left", "front_right", "left", "right", "back"]
    for i, (label, key) in enumerate(zip(directions, keys)):
        dist = obstacle_distances.get(key, 10.0)
        if dist < 0.5:
            color = (0, 0, 200)    # Red (BGR) — blocked
        elif dist < 1.0:
            color = (0, 200, 200)  # Yellow — restricted
        else:
            color = (0, 180, 0)    # Green — clear
        x = bar_x + i * 100
        cv2.rectangle(cv_image, (x, bar_y), (x + 90, bar_y + 18), color, -1)
        text = f"{label}:{dist:.1f}m"
        cv2.putText(cv_image, text, (x + 4, bar_y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
```

**Function signature change:** Add `obstacle_distances=None` parameter to `annotate_image()`. Caller passes it from the snapshot capture context.

## Dependencies

### Prerequisites

| Dependency | Type | Status | Notes |
|-----------|------|--------|-------|
| Research 009 complete | Research | ✅ | All findings documented with line references |
| `safety_executor.py` | Code | ✅ | Contains escape logic, watchdog, blocked memory |
| `voice_mapper.py` | Code | ✅ | Contains control loop, VLM loop, sensor monitor |
| `velocity_arbiter.py` | Code | ✅ | Contains arbiter safety filter |
| `sensor_snapshot.py` | Code | ✅ | Contains affordance scores, image annotation |
| OpenCV (cv2) | Library | ✅ | Already imported in `sensor_snapshot.py` |
| `self.vslam_position` / `self.vslam_tracking` | Runtime | ✅ | Already maintained in `voice_mapper.py` |
| `self._vlm_trigger_event` | Runtime | ✅ | Already exists for external VLM wake |

### External Dependencies

None. All changes are within existing Python scripts. No new packages, no message definition changes, no new ROS2 nodes.

## Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Stuck detector false positives during slow but intentional movement | Medium | Medium | Nav2/observe suspension; 0.05m threshold is generous (robot width ~0.25m); constants tunable |
| Multi-strategy escape causes erratic behavior near obstacles | Low | Medium | All escape moves go through pre-validation; arbiter safety filter is non-bypassable |
| VSLAM→wheel odometry source switch causes displacement spike | Low | Low | Position history cleared on source switch; stuck timer resets |
| Tier 2 probe movements (0.5s) confuse VLM's spatial model | Low | Low | VLM receives action result including escape metadata; stuck_state flag informs VLM |
| Lowering arbiter backward threshold allows collision | Low | High | 0.3m matches emergency_distance (already the minimum safe distance for rotation); LiDAR safety filter still active |
| Camera grace period masks real camera failure during SLAM startup | Low | Medium | 10s suppression is bounded; LiDAR monitoring remains active; camera recovery auto-resumes |
| Legend bar occludes important scene content at image bottom | Low | Low | Bar is 18px tall at very bottom; depth samples are at center/upper regions |

## Execution Plan

### Phase 1: Core Stuck Detection & Multi-Strategy Escape (P0)

**Goal:** Add the position-based stuck detector and replace the backward-only escape with affordance-ranked multi-strategy escape. This is the minimum viable fix for the stuck-robot problem.

**Files modified:** `safety_executor.py`, `voice_mapper.py`

**Prerequisites:** None (first phase)

**Verification:** Deploy to robot, drive near a wall, confirm:
1. `journalctl -u robot-voice-mapper` shows "STUCK TIER1" after ~8s of no movement
2. Robot rotates toward the clearest direction
3. After 15s, "STUCK TIER2" appears and robot probes alternative directions
4. After 30s, "STUCK TIER3" appears and robot speaks help request
5. Cornered robot (all sides blocked) does NOT reset `_consecutive_blocked` — reaches Tier 3

| # | Task | Status | File | Lines | Description | Acceptance Criteria |
|---|------|--------|------|-------|-------------|-------------------|
| 1.1 | Add `StuckDetector` class | ✅ Complete | `safety_executor.py` | After line 177 | New class with `__init__`, `update(x, y, now)`, `suspend()`, `resume()`, `reset()`, `stuck_state` property. Constants: `TIER1_TIMEOUT_S=8.0`, `TIER2_TIMEOUT_S=15.0`, `TIER3_TIMEOUT_S=30.0`, `STUCK_DISPLACEMENT_M=0.05`, `SAMPLE_INTERVAL_S=1.0`. Uses `deque(maxlen=30)` for position history (30 samples at 1 Hz = 30s window). `update()` rate-limits sampling to 1 Hz internally, computes displacement between oldest and newest position, returns tier string or None. Suspended state skips all logic and returns None. | Class instantiable; `update()` returns None when displacement > 0.05m; returns "TIER1" after 8s stationary; returns "TIER2" after 15s; returns "TIER3" after 30s; `suspend()`/`resume()` toggle sampling; `reset()` clears history and `_stuck_since` |
| 1.2 | Add shared escape direction helpers to `SafetyExecutor` | ✅ Complete | `safety_executor.py` | After `_maybe_escape()` (~line 265) | Add three public methods: (A) `find_best_escape_direction(self, obstacle_distances) -> Optional[str]`: computes affordance scores via `compute_affordance_scores(obstacle_distances)` (import from `sensor_snapshot`), sorts by score descending, for each direction calls `_pre_validate()` with synthetic `NavigationDecision`, returns first direction that passes or None if all blocked. (B) `validate_direction(self, direction, obstacle_distances) -> bool`: wraps `_pre_validate()` with a synthetic `NavigationDecision` for the given direction. (C) `reset_stuck_state(self)`: zeros `_consecutive_blocked` and calls `blocked_memory.clear()`. Called by voice_mapper when stuck detector transitions from stuck back to None (robot moving again after recovery). These are the public API used by both `_maybe_escape()` and the tier escapes in `voice_mapper.py`. Import `compute_affordance_scores` from `sensor_snapshot` at top of file. | `find_best_escape_direction()` returns best passable direction string or None; `validate_direction()` returns bool; `reset_stuck_state()` zeros counter and clears memory; all callable from external modules |
| 1.3 | Replace `_maybe_escape()` with multi-strategy version using shared helpers | ✅ Complete | `safety_executor.py` | 244–265 | Replace entire method. New logic: (1) check `_consecutive_blocked < STUCK_ESCAPE_THRESHOLD`, (2) call `self.find_best_escape_direction(obstacle_distances)`, (3) if direction found: call `self._escape_target_for_direction(direction)`, reset `_consecutive_blocked = 0`, clear blocked memory, return result, (4) if None (cornered): do NOT reset `_consecutive_blocked`, log warning "cornered — all directions blocked", return None. | Escape tries rotation/lateral before backward; cornered state does NOT reset counter; logs chosen escape direction at WARNING level |
| 1.3a | Add direction-to-target helper | ✅ Complete | `safety_executor.py` | After `_maybe_escape()` | New private method `_escape_target_for_direction(direction) -> ExecutionResult`. Maps direction to rotation degrees or backward move: forward→move forward, forward_left→rotate +45°, forward_right→rotate −45°, left→rotate +90°, right→rotate −90°, backward→move backward at −0.08 m/s for 2s. For rotation targets: use existing `_execute_rotate()` pattern (Ackerman-constrained twist). For move targets: use existing `_execute_move()` pattern. Returns `ExecutionResult(success=True, target=..., safety_override=True, safety_message="Auto-escape: {direction}")`. | Returns VelocityTarget with correct twist for each of 6 directions; rotation durations match existing `_execute_rotate()` formula |
| 1.4 | Initialize `StuckDetector` in `voice_mapper.py` | ✅ Complete | `voice_mapper.py` | Near line 413 (alongside `vlm_decision_queue`) | Add `self._stuck_detector = StuckDetector()` and `self._stuck_state = None`. Import `StuckDetector` from `safety_executor`. | Instance created at startup; `_stuck_state` initialized to None |
| 1.5 | Integrate stuck detector into `llm_control_loop()` | ✅ Complete | `voice_mapper.py` | After target application block (~line 2669), before watchdog check (~line 2672) | Add position source selection (VSLAM if `self.vslam_tracking` and odom fresh within 2s, else wheel odom). Add Nav2/observe suspension logic. Call `self._stuck_detector.update(pos_x, pos_y)`. Store result in `self._stuck_state`. On source switch (track previous source), call `self._stuck_detector.reset()`. | Stuck detector updated every control loop cycle; suspended during Nav2/observe; position source selected correctly |
| 1.6 | Add `_execute_tier1_escape()` method | ✅ Complete | `voice_mapper.py` | After `_fallback_to_frontier()` (~line 2712) | New method: call `self.safety_executor.find_best_escape_direction(self.obstacle_distances)` to get best direction. If found, build rotation target toward that direction, set `self._current_target` to VelocityTarget for rotation, set `self._target_start_time`, set `self._vlm_trigger_event` to wake VLM, log at WARNING. If None returned, skip (Tier 2 will handle). Uses public `find_best_escape_direction()` — no private method access. | Rotates toward highest-affordance direction; wakes VLM loop; logs tier and direction |
| 1.7 | Add `_execute_tier2_escape()` method | ✅ Complete | `voice_mapper.py` | After `_execute_tier1_escape()` | New method: call `self.safety_executor.find_best_escape_direction(self.obstacle_distances)` to get best passable direction. If found, set as current target (move_toward at slow speed, 2.0s duration). If not found, use `self.safety_executor.validate_direction(direction, self.obstacle_distances)` to iterate remaining directions individually with relaxed criteria. Set `self._vlm_trigger_event`. Log at WARNING. If no direction passes, log at ERROR "all directions blocked". Uses only public SafetyExecutor API — no private method access. | Tries all directions in affordance order; uses public validation API; wakes VLM; logs attempts |
| 1.8 | Add `_execute_tier3_help()` method | ✅ Complete | `voice_mapper.py` | After `_execute_tier2_escape()` | New method: publish zero velocity at PRIORITY_SAFETY, call `self.speak("I'm stuck and can't find a way out. Please help me.")`, set `self._stuck_state = "TIER3"`, log at ERROR. Do NOT call `stop_exploration()` — remain in control loop so voice commands can trigger recovery. | Robot stops, speaks, logs; exploration NOT terminated; voice commands still processed |
| 1.9 | Add tier dispatch in control loop | ✅ Complete | `voice_mapper.py` | In the stuck detector integration block (task 1.5) | After `tier = self._stuck_detector.update(...)`: dispatch to `_execute_tier1_escape()`, `_execute_tier2_escape()`, or `_execute_tier3_help()` based on tier value. Only dispatch once per tier level (track `_last_stuck_tier` to avoid repeating). When `tier` transitions from non-None back to None (robot moving again): reset `_last_stuck_tier`, call `self._stuck_detector.reset()`, and call `self.safety_executor.reset_stuck_state()` to zero `_consecutive_blocked` and clear blocked memory. This ensures a clean slate after recovery (whether from automated escape or human repositioning). After successful Tier 1 or 2 escape (robot moves), the same stuck-clear transition handles the reset. | Each tier triggers exactly once per stuck episode; detector and executor both reset on stuck-clear transition; no stale counter after Tier 3 recovery |

**Phase 1 Implementation Notes (2026-03-18):**
- `StuckDetector` class placed at line 183 in `safety_executor.py`, after `LLMWatchdog` and before `NetworkMonitor`
- `_escape_target_for_direction()` uses same Ackerman arc-turn pattern as `_execute_rotate()` (ACKERMAN_MIN_LINEAR forward + ACKERMAN_ANGULAR_SPEED rotation)
- `validate_direction()` creates synthetic `NavigationDecision` with required `timestamp` field
- Position source in control loop uses `self.vslam_pose.position.x/y` (ROS Pose message) — no `vslam_position` dict exists
- Stuck detector also reset in `start_llm_exploration()` alongside watchdog/blocked memory reset
- `_execute_tier1_escape()` uses `_escape_target_for_direction()` directly (private access within same codebase) since `find_best_escape_direction()` already validated the direction

---

### Phase 2: Safety System Alignment (P1)

**Goal:** Fix the supporting safety issues that would undermine Phase 1 effectiveness: arbiter backward threshold, SLAM startup race, and watchdog blind spot.

**Files modified:** `velocity_arbiter.py`, `voice_mapper.py`, `safety_executor.py`

**Prerequisites:** Phase 1 complete (stuck detector and escape mechanism exist)

**Verification:** Deploy to robot:
1. Escape backward moves are not killed by arbiter (test: back obstacle at 0.35m, escape should move)
2. SLAM startup does not trigger false camera-loss alarm (test: restart robot, watch `journalctl` for absence of "Camera data lost" during first 10s)
3. Robot stuck with healthy VLM connection → watchdog escalates after 30s of no successful execution

| # | Task | Status | File | Lines | Description | Acceptance Criteria |
|---|------|--------|------|-------|-------------|-------------------|
| 2.1 | Lower arbiter backward stop threshold to 0.3m | ✅ Complete | `velocity_arbiter.py` | 349–356 | Change the backward motion check from `self.min_distance` (0.5m) to `self.emergency_distance` (0.3m). Single line change: `if linear < 0 and back_dist < self.emergency_distance:`. Add a comment explaining why backward uses emergency_distance: "Backward uses emergency threshold (0.3m) — safety executor pre-validates at 0.5m for normal moves; this allows escape maneuvers approved at 0.3m to execute." | Backward motion allowed down to 0.3m obstacle distance; forward threshold unchanged at 0.5m |
| 2.2 | Add `_camera_monitor_suppress_until` instance variable | ✅ Complete | `voice_mapper.py` | 435 | Add `self._camera_monitor_suppress_until = 0.0` alongside other instance variables. | Variable initialized to 0.0 at startup |
| 2.3 | Set camera grace period in `start_slam()` | ✅ Complete | `voice_mapper.py` | 1428–1429 | Add `self._camera_monitor_suppress_until = time.time() + 10.0` and log at INFO: "Camera monitor suppressed for 10s during SLAM startup". | Timestamp set 10s in future when SLAM starts |
| 2.4 | Check grace period in `_sensor_monitor_loop()` | ✅ Complete | `voice_mapper.py` | 3334 | Added `time.time() >= self._camera_monitor_suppress_until` guard to the camera-loss detection `elif` condition. Camera recovery logic (when `camera_lost` is True and image arrives) is NOT suppressed — only the loss detection. | Camera loss not declared during first 10s after SLAM start; camera recovery still works normally; LiDAR monitoring unaffected |
| 2.5 | Add `feed_success()` to `LLMWatchdog` | ✅ Complete | `safety_executor.py` | 158–200 | Added `_last_success_time` and `_success_timeout_s` to `__init__`. Added `feed_success()` method. Modified `evaluate()` to return worst tier across both feeds: escalates to `STOP_WAIT` when no successful execution in 30s even if VLM is responding. | `feed_success()` updates timestamp; `evaluate()` returns `STOP_WAIT` when no successful execution in 30s even if VLM is responding; existing tier logic unchanged for response feed |
| 2.6 | Call `feed_success()` on execution success | ✅ Complete | `voice_mapper.py` | 2617 | After `set_previous_action_result()`, added: `if result.success: self.safety_executor.watchdog.feed_success()`. | Watchdog success feed updated only on successful execution; failed/blocked decisions do not feed it |

---

### Phase 3: VLM Feedback Enhancements (P2)

**Goal:** Improve the VLM's ability to respond to stuck situations: faster cadence when stuck, richer action feedback, and cornered counter fix.

**Files modified:** `voice_mapper.py`, `safety_executor.py`

**Prerequisites:** Phase 1 complete (stuck detector provides `_stuck_state`)

**Verification:**
1. VLM call interval drops to ~3s when stuck (check `journalctl` timestamps between VLM calls)
2. Action result includes `actual_distance_m` (check VLM prompt in debug logs)
3. Cornered robot's `_consecutive_blocked` counter does not reset (already verified in Phase 1, but confirm interaction with watchdog)

| # | Task | Status | File | Lines | Description | Acceptance Criteria |
|---|------|--------|------|-------|-------------|-------------------|
| 3.1 | Reduce VLM interval when stuck | ✅ Complete | `voice_mapper.py` | 2400–2401 | Compute `effective_interval = 3.0 if self._stuck_state else base_interval` and use it for the elapsed check in the event-driven wait loop. | VLM calls every ~3s when `_stuck_state` is non-None; reverts to 15s when stuck clears |
| 3.2 | Track position before/after movement execution | ✅ Complete | `voice_mapper.py` | 2607–2609, 2678–2684 | Record `_pre_move_pos` before `execute_decision()`. Compute `_last_move_distance` via `math.hypot()` when VelocityTarget duration expires. | Distance computed after each VelocityTarget completes |
| 3.3 | Include `actual_distance_m` in action result | ✅ Complete | `voice_mapper.py` | 2619–2624 | Added `actual_distance_m` and `stopped_reason` to action_result dict. Reset `_last_move_distance = None` after inclusion. | Action result dict contains distance and stopped_reason |
| 3.4 | Initialize `_last_move_distance` | ✅ Complete | `voice_mapper.py` | 436–437 | Added `self._last_move_distance = None` and `self._pre_move_pos = None`. | Variables initialized at startup |

---

### Phase 4: Sensor Safety & Visual Enhancements (P3)

**Goal:** Guard against stale LiDAR producing dangerous affordance scores, and add visual clearance indicators to the camera image for better VLM spatial reasoning.

**Files modified:** `sensor_snapshot.py`, `voice_mapper.py`

**Prerequisites:** None (independent of Phases 1–3, but deployed after for safety)

**Verification:**
1. With LiDAR cable disconnected: affordance scores show all 0.0 (not 1.0)
2. Annotated image shows colored legend bar at bottom with direction labels and distances

| # | Task | Status | File | Lines | Description | Acceptance Criteria |
|---|------|--------|------|-------|-------------|-------------------|
| 4.1 | Guard empty `obstacle_distances` in `compute_affordance_scores()` | ✅ Complete | `sensor_snapshot.py` | 215–219 | Added empty-dict guard returning all 0.0 scores. Updated docstring to reflect 0.0–1.0 range. | Empty dict → all 0.0 scores; non-empty dict → existing behavior unchanged; docstring reflects new range |
| 4.2 | Add `obstacle_distances` parameter to `annotate_image()` | ✅ Complete | `sensor_snapshot.py` | 108–109 | Added `obstacle_distances=None` parameter. Default None preserves backward compatibility. | Function accepts optional `obstacle_distances`; existing callers unaffected |
| 4.3 | Add clearance legend bar to `annotate_image()` | ✅ Complete | `sensor_snapshot.py` | 153–171 | Added 6 colored rectangles at y=460 with distance labels. Red (<0.5m), yellow (0.5–1.0m), green (≥1.0m). | 6 colored rectangles visible at bottom of image; colors match thresholds; text readable; no rendering when `obstacle_distances` is None |
| 4.4 | Pass `obstacle_distances` to `annotate_image()` from caller | ✅ Complete | `sensor_snapshot.py` | 464–469 | Added `obstacle_distances=getattr(vm, 'obstacle_distances', None)` to the `annotate_image()` call in `capture_snapshot()`. | `obstacle_distances` flows from vm through to image annotation; None-safe via getattr default |

## Complexity Assessment

| Factor | Score (1-5) | Notes |
|--------|-------------|-------|
| Files to modify | 3 | 4 files, but changes are well-scoped within each |
| New patterns introduced | 3 | StuckDetector class, shared escape helpers, public reset API |
| External dependencies | 1 | No new packages, no new ROS2 interfaces |
| Migration complexity | 1 | Purely additive; no data migration, no breaking changes |
| Test coverage required | 3 | No unit test infra; verification via live robot + journalctl |
| **Overall Complexity** | **11/25** | **Medium** — Multiple interacting components across 4 phases, but each task is well-defined and independently verifiable |

## Review Summary

**Review Date:** 2026-03-18
**Reviewer:** pch-plan-reviewer
**Issues Found:** 8 (1 Critical, 3 Major, 4 Minor — all resolved)
**Plan Quality:** High after review

### Changes Made During Review

1. **Task 4.4 file corrected** (Critical): `voice_mapper.py` → `sensor_snapshot.py:441`. The `annotate_image()` caller was in the wrong file.
2. **Shared escape helpers extracted** (Major, R1): Added `find_best_escape_direction()`, `validate_direction()`, and `reset_stuck_state()` as public SafetyExecutor methods. Tasks 1.2, 1.3, 1.6, 1.7 updated to use public API instead of duplicating logic or accessing private methods.
3. **Stuck-clear reset added** (Major, R2): `reset_stuck_state()` called when stuck detector transitions back to None, preventing stale `_consecutive_blocked` after Tier 3 recovery. Task 1.9 updated.
4. **Private method access eliminated** (Major, R1/R2): Tasks 1.6 and 1.7 no longer call `_pre_validate()` directly; they use public `find_best_escape_direction()` and `validate_direction()`.
5. **Architecture diagram fixed**: 15Hz → 10Hz poll rate.
6. **File count fixed**: 3 files → 4 files in introduction.
7. **Docstring range noted**: Task 4.1 includes docstring update for 0.0–1.0 range.
8. **Task 2.3 line reference sharpened**: Precise insertion point between lines 1420–1421.
9. **Task count updated**: Phase 1 now has 10 tasks (1.3a added for direction-to-target helper).

## Standards

- All velocity commands go through VelocityRequest messages to the arbiter
- The arbiter is the single writer to /cmd_vel with a non-bypassable LiDAR safety filter
- Safety executor pre-validates all movement decisions against LiDAR
- Existing code patterns and naming conventions must be followed

## Handoff

```
Plan Approved — Ready for Implementation

Plan at: docs/plans/008-stuck-robot-recovery.md
Research: docs/research/009-stuck-robot-exploration-recovery.md
Issues Found: 8 (all resolved)
Plan Quality: High after review

Phase summary:
- Phase 1 (P0): StuckDetector class + shared escape helpers + multi-strategy escape + control loop integration (10 tasks)
- Phase 2 (P1): Arbiter backward threshold + SLAM grace period + watchdog dual-feed (6 tasks)
- Phase 3 (P2): VLM cadence acceleration + actual_distance_m (4 tasks)
- Phase 4 (P3): Stale LiDAR guard + image legend bar (4 tasks)

Next Step: Use /pch-coder to begin implementing the plan.
```
