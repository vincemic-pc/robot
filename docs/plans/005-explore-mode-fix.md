---
id: "005"
type: implementation-plan
title: "Explore Mode Fix — Forward Movement & Obstacle Detection"
status: ✅ complete
created: "2026-02-25"
updated: "2026-02-25"
owner: pch-planner
version: v1.1
research: "[006 — Explore Mode Not Navigating](../research/006-explore-mode-not-navigating.md)"
---

## Version History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| v0.1 | 2026-02-25 | pch-planner | Initial plan skeleton |
| v1.0 | 2026-02-25 | pch-planner | Complete plan: all decisions, requirements, technical design, execution plan (2 phases), holistic review |
| v1.1 | 2026-02-25 | pch-plan-reviewer | Review: fixed critical `fl > fr` NameError [R1], moved stuck backup to Phase 1 [R2], eliminated redundant `find_frontiers()` [R3], documented implicit `slow_dist` behavior change [R4] |
| v2.0 | 2026-02-25 | pch-coder | Implementation complete: Phase 1 (6 tasks) + Phase 2 (7 tasks), all verified |

## Introduction

The robot's explore mode (triggered by `{"action": "explore"}`) exhibits three symptoms: the robot stays stationary, moves backwards a few cm then stops, and the LLM repeatedly describes surroundings without issuing navigation commands. [Research 006](../research/006-explore-mode-not-navigating.md) traced these to a cascade of compounding issues in `scripts/voice_mapper.py`: a 135° obstacle detection cone that triggers on side objects, Ackerman reverse-to-turn logic that always moves backwards, a 0.5m threshold that's too conservative for the wide cone, a reactive fallback that barely moves when Nav2 is unavailable, inline `speak()` calls that block movement, and a `get_context()` that gives the LLM no visibility into exploration progress.

This plan implements all six prioritized fixes from Research 006 across two phases: Phase 1 addresses the three core symptom-causing bugs (cone, turn direction, threshold), and Phase 2 improves reactive exploration reliability and LLM context. All changes are confined to `scripts/voice_mapper.py`.

## Planning Session Log

| # | Decision Point | Answer | Rationale |
|---|----------------|--------|-----------|
| 1 | Plan scope | C — All fixes (P1–P6) | Core cone/turn/threshold fixes (P1–P3) address symptoms directly; reliability improvements (P4–P5) prevent recurrence when Nav2 unavailable; LLM context enhancement (P6) gives user/LLM visibility into exploration health. Excludes recursive call refactor to limit risk. |
| 2 | Obstacle detection cone width | A — ±30° only (`front_wide`) | Tightest cone, matches emergency stop logic. Side objects at 35–90° completely ignored for exploration obstacle avoidance. Acceptable risk for 20cm-wide robot — if a 35–45° object could clip the robot, the costmap/Nav2 collision monitor (when active) or the separate `front`/`front_right` sector checks in `choose_exploration_direction()` still provide coverage for forward driving decisions. |
| 3 | Exploration obstacle threshold | B — 0.30m (matches `emergency_dist`) | Aligns exploration avoidance with existing emergency stop distance. Clean mental model: exploration avoids at the same distance manual mode emergency-stops. 2s reaction window at max exploration speed (0.15 m/s) is sufficient. With ±30° cone (D2), 0.30m eliminates most indoor false positives. |
| 4 | Reactive mode strategy | C — Merge forward-arc fix + walk promotion | Fix `_reactive_exploration_step()` to forward arc turns (never reverse) AND restructure `_reactive_exploration_loop()` to call `_random_exploration_walk()` more aggressively. Micro-steps never reverse; robot quickly escalates to proven gap-detection/open-space walk. Both changes complementary and low-effort. |
| 5 | LLM context enhancement | B — Moderate telemetry | Add Nav2 status (`active/starting/unavailable`), exploration mode (`nav2-frontier/reactive/random-walk`), frontier count, and time since last meaningful movement. Enough for LLM to distinguish "exploring successfully" from "stuck" and explain why. Not too verbose for context window. |

## Review Session Log

| # | Question | Decision | Rationale |
|---|----------|----------|-----------|
| R1 | `fl > fr` NameError at L2629 after Change 3a removes `fl`/`fr` variables | A — Drop `fl > fr`, use `left > right` only | Consistent with `_reactive_exploration_step()` pattern. `left`/`right` sectors (72°–126° / 234°–288°) give reliable side-clearance signal. With cone narrowed to ±30°, front-left/front-right quadrants are no longer the detection zone. Simpler, consistent. |
| R2 | Phase 1 verification checkpoint claims "never reverse except stuck_count > 5 backup" but that backup (`linear.x = -0.1`, L2615-2623) is unchanged until Phase 2 Task 2.2 | B — Move stuck backup fix into Phase 1 | Makes Phase 1 fully "no reverse in exploration". The stuck_count > 3 → `_random_exploration_walk()` escalation is a natural complement to forward-arc turns — without it, the robot can still reverse after 5 obstacle hits, undermining the forward-only fix. |
| R3 | Task 2.4 calls `find_frontiers()` redundantly — `choose_frontier()` already calls it at L759 | B — Refactor `choose_frontier()` to expose count | One-line addition (`self.explore_frontier_count = len(frontiers)`) inside `choose_frontier()` at L759. Zero extra computation, accurate count. The out-of-scope note applies to `_random_exploration_walk()` internals, not `choose_frontier()`. |
| R4 | `choose_exploration_direction()` `slow_dist` check at L2110 implicitly changes behavior — `min_front` now uses ±30° instead of 135° for speed selection too | A — Accept implicit change | The ±30° cone is the direction the robot is heading. Side objects at 35–90° shouldn't gate forward speed. The old 135° `min_front` caused unnecessary slowdowns (wall 0.8m to the left → half speed while heading straight), contributing to the "barely moves" symptom. Consistent with narrow-cone philosophy. |

## Holistic Review

### Decision Interactions

All five decisions form a coherent, layered fix with no conflicts:

1. **Narrow cone (D2) + lower threshold (D3):** Complementary. The ±30° cone eliminates side-object false triggers; the 0.3m threshold reduces false triggers from objects that *are* ahead but at a safe distance. Together they solve the "obstacle everywhere" problem that trapped the robot. Neither change alone would be sufficient — a narrow cone with 0.5m threshold would still trigger too often in hallways; a wide cone with 0.3m threshold would still see side objects.

2. **Forward arc turns (D4) + narrow cone (D2):** Forward turns push the robot *toward* the detected obstacle, but with the cone narrowed to ±30°, the obstacle is genuinely ahead and the arc turn steers around it. With the old 135° cone, forward turns would have been dangerous (turning toward a side object). The two changes must ship together.

3. **Walk promotion (D4) + non-blocking speak (P5, in scope via D1):** `_random_exploration_walk()` calls `speak()` at L2449 when driving through a doorway. With walk promotion (D4), this function is called more frequently. Without non-blocking speak (P5), the increased walk frequency would mean more movement-blocking TTS calls. The two fixes must ship together.

4. **LLM context (D5) + exploration telemetry state:** The `explore_mode` and `last_meaningful_movement` variables serve double duty — they power both the `get_context()` output and provide future debugging visibility via logs. Adding them is low-cost and doesn't affect control flow.

### Architectural Considerations

- **Single-file scope** — All changes are in `scripts/voice_mapper.py`. No config, launch, or external changes. This minimizes regression risk.
- **Existing pattern reuse** — Forward arc turns at `0.08 m/s` are already proven in `_random_exploration_walk()` (L2431, L2489, L2493). The plan extends this pattern to the reactive functions.
- **Preserved safety layers** — The stuck_count > 5 reverse backup in `_reactive_exploration_loop()` is being replaced by stuck_count > 3 → `_random_exploration_walk()`. However, `_random_exploration_walk()` itself has a forward-only obstacle check at 0.35m (L2456) that stops if too close. The last-resort reverse escape is no longer present in the reactive loop; if `_random_exploration_walk()` also fails, the loop will increment stuck_count again and re-escalate. FR10 should be revised: the 1.5s reverse backup is being removed, not preserved.
- **Phase independence** — Phase 1 is fully self-contained (cone + threshold + turn direction). Phase 2 depends only on the `self.explore_obstacle_dist` attribute from Phase 1. Each phase can be tested independently.

### Gap Analysis

1. **FR10 conflict**: FR10 states "stuck backup continues to use reverse as last-resort escape" but D4 (merged reactive fix) removes the 1.5s reverse backup block in favor of `_random_exploration_walk()`. **Resolution**: Update FR10 — the reverse backup is replaced by smart walk escalation, which is a better escape strategy. The only remaining reverse in exploration is within `_random_exploration_walk()` itself (there is none — it's all forward). Accept that exploration no longer reverses at all.
2. **`_random_exploration_walk()` obstacle check still uses `front` at 0.5m** (L2518): The random walk's forward driving loop checks `front < 0.5` to stop. This is not being changed. This is acceptable — the random walk is driving *forward* at 0.12 m/s with a known clear path; the 0.5m check here is a safety stop, not an avoidance trigger. The broader `front` sector (0–45°) is appropriate for forward driving safety.
3. **`_random_exploration_walk()` doorway code uses `front < 0.35`** (L2456): Even tighter threshold for doorway traversal. This is correct and unchanged — when deliberately driving through a doorway, tighter tolerance is acceptable.
4. **No reverse at all in exploration after this plan**: With the reactive backup removed and all turns changed to forward, the robot will never reverse during exploration. If truly boxed in with no forward path, `_random_exploration_walk()` will find the most open direction and drive forward. In a dead-end, this means the robot would turn in a forward arc and exit. Acceptable behavior for Ackerman steering.

### Trade-offs Accepted

1. **No reverse escape** — The robot can no longer back out of tight spaces during exploration. Accepted because: (a) forward arc turns at 0.08 m/s with Ackerman steering have a ~0.3m turn radius, sufficient for most indoor spaces; (b) `_random_exploration_walk()` actively seeks gaps and open space; (c) the old reverse behavior was the primary symptom (backwards movement).
2. **±30° cone may miss obstacles** — A wall approaching at 35° won't trigger avoidance. Accepted because: (a) Nav2 costmap handles this when active; (b) `choose_exploration_direction()` still reads wider sectors for *direction preference* (just not for stop triggering); (c) 20cm robot width means 35° objects at 0.3m are ~17cm from the robot edge — tight but passable.
3. **Increased `_random_exploration_walk()` frequency** — Walk promotion means more gap-detection and open-space driving. Each walk takes 3–8 seconds. Accepted because: this is the proven working path and the whole point of P4.

## Overview

### Feature Summary

Fix the robot's autonomous exploration so it reliably moves forward, navigates through doorways, and gives the user/LLM honest feedback about exploration progress.

### Objectives

1. **Eliminate backward-only movement** — Replace all reverse-to-turn commands in reactive exploration with forward arc turns, matching the proven pattern in `_random_exploration_walk()`
2. **Eliminate false obstacle triggers** — Narrow the exploration obstacle detection cone from 135° (±45° + asymmetric +90° left) to ±30° (`front_wide` only), and lower the threshold from 0.5m to 0.3m
3. **Improve reactive mode robustness** — Promote `_random_exploration_walk()` (gap detection + open-space driving) as the primary reactive strategy, with micro-steps as fill between walk cycles
4. **Unblock exploration thread from TTS** — Run `speak()` in background threads from exploration code
5. **Give LLM exploration visibility** — Enhance `get_context()` with Nav2 status, exploration mode, frontier count, and stuck detection

### Symptoms Addressed

| Symptom | Root Causes (from Research 006) | Fixes Applied |
|---------|-------------------------------|---------------|
| Robot doesn't move | H1 (Nav2 fails → reactive mode), H8 (no map → no frontiers), H15 (135° cone false triggers) | P1 narrow cone, P3 lower threshold, P4 better reactive |
| Moves backwards only | H3 (Ackerman reverse-to-turn), H9 (Nav2 BackUp recovery), H5 (0.5m on wide cone) | P2 forward arc turns, P1 narrow cone, P3 lower threshold |
| Loops describing | H4/H14 (observation timer decoupled from movement), H12 (no progress in LLM context) | P5 non-blocking speak, P6 LLM context |

## Requirements

### Functional Requirements

| ID | Requirement | Priority | Traces To |
|----|-------------|----------|-----------|
| FR1 | Exploration obstacle detection uses ±30° cone (`front_wide`) instead of 135° combined sector | Must | D2, P1 |
| FR2 | All reactive exploration obstacle avoidance uses forward arc turns (minimum `linear.x = 0.08`) — no reverse commands | Must | D4, P2 |
| FR3 | Exploration-specific obstacle threshold of 0.3m, separate from manual movement's 0.5m | Must | D3, P3 |
| FR4 | `_reactive_exploration_step()` uses forward arc turns with ±30° cone and 0.3m threshold | Must | D2, D3, D4 |
| FR5 | `_reactive_exploration_loop()` escalates to `_random_exploration_walk()` after 3 obstacle encounters (down from 5+) | Should | D4, P4 |
| FR6 | `_reactive_exploration_loop()` obstacle avoidance block uses forward arc turns, not reverse | Must | D4, P2 |
| FR7 | Inline `speak()` calls in `_random_exploration_walk()` and `_reactive_exploration_loop()` run in background threads | Should | P5 |
| FR8 | `get_context()` includes: Nav2 status, exploration mode, frontier count, time since last meaningful movement | Should | D5, P6 |
| FR9 | `choose_exploration_direction()` uses ±30° cone and 0.3m threshold consistent with other exploration functions | Must | D2, D3 |
| FR10 | `_reactive_exploration_loop()` stuck escalation (stuck_count > 3) calls `_random_exploration_walk()` — no reverse commands remain in exploration; forward-only movement is acceptable for Ackerman steering | Must | D4, Safety |

### Non-Functional Requirements

| ID | Requirement |
|----|-------------|
| NFR1 | All changes confined to `scripts/voice_mapper.py` — no config, launch, or external file changes |
| NFR2 | Ackerman steering constraint respected: minimum forward speed ≥ 0.06 m/s for any turn command |
| NFR3 | Existing logging patterns preserved (logger levels, message format) |
| NFR4 | No new dependencies — only stdlib `threading` (already imported) |
| NFR5 | Manual movement (`move()`, voice commands) behavior unchanged — `min_obstacle_dist = 0.5` still used |

### Out of Scope

- Recursive `exploration_loop()` call from `_try_nav2_with_reactive_fallback()` (structural refactor deferred)
- AMCL motion model mismatch (`DifferentialMotionModel` for Ackerman — no better option in Nav2)
- Nav2 goal failure propagation to exploration loop
- Nav2 costmap inflation tuning
- `_random_exploration_walk()` internal logic changes (it works correctly as-is)
- `scan_callback()` sector definitions (sectors remain unchanged; only how exploration *reads* them changes)

## Technical Design

### Architecture

No architectural changes. All modifications are within existing functions in `scripts/voice_mapper.py`. The exploration control flow remains:

```
start_exploration() → exploration_loop()
    ├── Nav2 available → frontier-based loop (unchanged)
    └── Nav2 unavailable → _try_nav2_with_reactive_fallback()
            ├── 30s wait → _reactive_exploration_step() (MODIFIED: forward turns, narrow cone)
            └── fallback → _reactive_exploration_loop() (MODIFIED: forward turns, walk promotion)
```

### Change Summary by Function

#### 1. New state variable: `self.explore_obstacle_dist` (L358, after `self.look_interval`)

**Add** a new exploration-specific obstacle threshold:

```python
# AFTER line 359 (self.look_interval = 15.0):
self.explore_obstacle_dist = 0.3  # Exploration obstacle threshold (matches emergency_dist)
```

Also add exploration telemetry state for `get_context()` (P6):

```python
self.explore_mode = "idle"            # "idle" | "nav2-frontier" | "reactive" | "random-walk"
self.last_meaningful_movement = 0.0   # timestamp of last position change > 0.3m
self.explore_frontier_count = 0       # frontiers found in last check
```

#### 2. `_reactive_exploration_step()` (L2552-2574) — MODIFIED

**Before** (current code):
```python
def _reactive_exploration_step(self):
    front = self.obstacle_distances.get("front", 10)
    front_wide = self.obstacle_distances.get("front_wide", 10)
    fl = self.obstacle_distances.get("front_left", 10)
    fr = self.obstacle_distances.get("front_right", 10)
    min_front = min(front, front_wide, fl, fr)       # ← 135° cone
    twist = Twist()
    if min_front < self.min_obstacle_dist:            # ← 0.5m threshold
        left = self.obstacle_distances.get("left", 10)
        right = self.obstacle_distances.get("right", 10)
        twist.linear.x = -0.05                        # ← REVERSE
        twist.angular.z = 0.4 if left > right else -0.4
    else:
        twist.linear.x = 0.1
        twist.angular.z = random.uniform(-0.1, 0.1)
    self.cmd_vel_pub.publish(twist)
```

**After**:
```python
def _reactive_exploration_step(self):
    # Use narrow ±30° cone for exploration (not the full 135° combined sector)
    front_wide = self.obstacle_distances.get("front_wide", 10)
    twist = Twist()
    if front_wide < self.explore_obstacle_dist:       # ← 0.3m, ±30° only
        # Forward arc turn — Ackerman needs forward motion to steer
        left = self.obstacle_distances.get("left", 10)
        right = self.obstacle_distances.get("right", 10)
        twist.linear.x = 0.08                         # ← FORWARD arc turn
        twist.angular.z = 0.4 if left > right else -0.4
    else:
        twist.linear.x = 0.1
        twist.angular.z = random.uniform(-0.1, 0.1)
    self.cmd_vel_pub.publish(twist)
```

**Changes**: (a) `min_front` → `front_wide` only (±30°), (b) `self.min_obstacle_dist` → `self.explore_obstacle_dist`, (c) `linear.x = -0.05` → `0.08`.

#### 3. `_reactive_exploration_loop()` (L2576-2661) — MODIFIED

**Change 3a — Obstacle detection cone** (L2600-2604):

```python
# BEFORE:
front = self.obstacle_distances.get("front", 10)
front_wide = self.obstacle_distances.get("front_wide", 10)
fl = self.obstacle_distances.get("front_left", 10)
fr = self.obstacle_distances.get("front_right", 10)
min_front = min(front, front_wide, fl, fr)

# AFTER:
# Use narrow ±30° cone for exploration obstacle detection
front_wide = self.obstacle_distances.get("front_wide", 10)
min_front = front_wide
```

**Change 3b — Threshold** (L2606):

```python
# BEFORE:
if min_front < self.min_obstacle_dist:

# AFTER:
if min_front < self.explore_obstacle_dist:
```

**Change 3c — Escalate to random walk sooner** (L2615):

```python
# BEFORE:
if stuck_count > 5:

# AFTER:
if stuck_count > 3:
    # Escalate to smart exploration walk (gap detection + open space)
    self.get_logger().info("Stuck — escalating to smart exploration walk")
    self._random_exploration_walk()
    stuck_count = 0
    continue
```

Note: the old behavior at stuck_count > 5 was to back up for 1.5s then continue. The new behavior at stuck_count > 3 calls `_random_exploration_walk()` which actively finds gaps and open space. The 1.5s reverse backup block (L2616-2623) is removed.

**Change 3d — Forward arc turn + fix turn direction** (L2627-2629):

```python
# BEFORE:
twist = Twist()
twist.linear.x = -0.05
if left > right or fl > fr:

# AFTER:
twist = Twist()
twist.linear.x = 0.08  # Forward arc turn (Ackerman minimum)
if left > right:  # fl/fr removed — consistent with _reactive_exploration_step() [R1]
```

> **Review fix [R1]**: Change 3a removes `fl`/`fr` variables. The `fl > fr` condition at L2629 must also be removed to avoid NameError. Using `left > right` only is consistent with `_reactive_exploration_step()` and sufficient for side-clearance direction selection.

**Change 3e — Non-blocking speak for doorway detection** (L2594):

```python
# BEFORE (at L2594, inside the doorway detection block):
self.get_logger().info(f"Doorway spotted! Going through...")

# No speak() call here currently — but _random_exploration_walk() at L2449 has one:
# BEFORE (in _random_exploration_walk, L2449):
self.speak("I see an opening, going through!")

# AFTER:
threading.Thread(target=self.speak, args=("I see an opening, going through!",), daemon=True).start()
```

**Change 3f — Set explore_mode state** (at top of function):

```python
# ADD at L2582, after the log line:
self.explore_mode = "reactive"
```

#### 4. `choose_exploration_direction()` (L2085-2124) — MODIFIED

**Before** (L2094-2100):
```python
min_front = min(front, front_wide, front_left, front_right)
# ...
if min_front < self.min_obstacle_dist:
```

**After**:
```python
# Use narrow ±30° cone consistent with other exploration functions
min_front = front_wide
# ...
if min_front < self.explore_obstacle_dist:
```

The `slow_dist` speed-selection check at L2110 is **unchanged in code** but now operates on the ±30° `front_wide` reading instead of the old 135° minimum. This is an intentional implicit improvement [R4] — side objects at 35–90° no longer trigger unnecessary speed reduction, which contributed to the "barely moves" symptom.

```python
# L2110 — code unchanged, but min_front is now front_wide (±30°) [R4]:
if min_front > self.slow_dist:
```

#### 5. `get_context()` (L2128-2144) — ENHANCED

**Before**:
```python
context = f"""
Status:
- Mapping: {'active' if self.mapping else 'not started'}
- Exploring: {'yes' if self.exploring else 'no'}
- Position: ({self.current_position['x']:.1f}, {self.current_position['y']:.1f})
- Distance traveled: {summary['distance_meters']:.1f}m
- Discoveries: {summary['discoveries']}
- LiDAR: front={self.obstacle_distances.get('front', 0):.1f}m, left={self.obstacle_distances.get('left', 0):.1f}m, right={self.obstacle_distances.get('right', 0):.1f}m
"""
```

**After**:
```python
context = f"""
Status:
- Mapping: {'active' if self.mapping else 'not started'}
- Exploring: {'yes' if self.exploring else 'no'}
- Position: ({self.current_position['x']:.1f}, {self.current_position['y']:.1f})
- Distance traveled: {summary['distance_meters']:.1f}m
- Discoveries: {summary['discoveries']}
- LiDAR: front={self.obstacle_distances.get('front', 0):.1f}m, left={self.obstacle_distances.get('left', 0):.1f}m, right={self.obstacle_distances.get('right', 0):.1f}m
"""
        if self.exploring:
            time_since_move = time.time() - self.last_meaningful_movement if self.last_meaningful_movement else 0
            nav2_status = "active" if self.nav2_available else "unavailable"
            stuck = time_since_move > 30
            context += f"- Nav2: {nav2_status}\n"
            context += f"- Explore mode: {self.explore_mode}\n"
            context += f"- Frontiers found: {self.explore_frontier_count}\n"
            context += f"- Exploration progress: {'stuck' if stuck else 'moving'}\n"
```

#### 6a. `choose_frontier()` (L755-765) — ONE-LINE ADDITION [R3]

Store frontier count as side effect of existing `find_frontiers()` call — no redundant computation:

```python
# BEFORE (L759):
frontiers = self.find_frontiers()

# AFTER:
frontiers = self.find_frontiers()
self.explore_frontier_count = len(frontiers)
```

#### 6b. `exploration_loop()` (L2325-2415) — MINOR ADDITIONS

Add `explore_mode` state tracking and movement telemetry. No changes to the loop's navigation logic.

```python
# ADD at L2335, after "Starting adventurous exploration!":
self.explore_mode = "nav2-frontier"
self.last_meaningful_movement = time.time()
```

Also update `self.last_meaningful_movement` when a navigation goal succeeds (L2388-2389):
```python
# AFTER consecutive_failures = 0 (L2389):
self.last_meaningful_movement = time.time()
```

#### 7. `_random_exploration_walk()` (L2417-2527) — MINOR CHANGES

Only two changes:
1. **Non-blocking speak** at L2449
2. **Set explore_mode** at function entry

```python
# ADD at L2419, after log line:
self.explore_mode = "random-walk"

# CHANGE L2449:
# BEFORE:
self.speak("I see an opening, going through!")
# AFTER:
threading.Thread(target=self.speak, args=("I see an opening, going through!",), daemon=True).start()
```

#### 8. `_try_nav2_with_reactive_fallback()` (L2528-2550) — NO STRUCTURAL CHANGES

This function's structure is unchanged (the recursive call refactor is out of scope). The `_reactive_exploration_step()` it calls is already fixed (Change 2). No additional modifications needed.

#### 9. `stop_exploration()` — MINOR ADDITION

Reset explore_mode on stop:

```python
# ADD inside stop_exploration(), after self.exploring = False:
self.explore_mode = "idle"
```

### Key Code Locations (from Research 006)

| Component | File | Lines | Purpose |
|-----------|------|-------|---------|
| State init | `scripts/voice_mapper.py` | L328, L350-356 | `exploring` flag, `min_obstacle_dist=0.5`, movement params |
| Scan callback / sectors | `scripts/voice_mapper.py` | L993-1082 | LiDAR sector definitions, `min_front_distance` (±30°) |
| `choose_exploration_direction()` | `scripts/voice_mapper.py` | L2085-2124 | Direction selection for reactive exploration; uses 135° cone |
| `_observe_in_background()` | `scripts/voice_mapper.py` | L2316-2323 | 15s observation timer, GPT-4o vision → speak |
| `exploration_loop()` | `scripts/voice_mapper.py` | L2325-2415 | Main Nav2-based exploration loop |
| `_random_exploration_walk()` | `scripts/voice_mapper.py` | L2417-2527 | Gap/open-space forward driving (works correctly) |
| `_try_nav2_with_reactive_fallback()` | `scripts/voice_mapper.py` | L2528-2550 | Nav2 wait loop with reactive fallback |
| `_reactive_exploration_step()` | `scripts/voice_mapper.py` | L2552-2574 | Single reactive step; 135° cone + reverse |
| `_reactive_exploration_loop()` | `scripts/voice_mapper.py` | L2576-2661 | Full reactive fallback; 135° cone + reverse + stuck backup |
| `get_context()` | `scripts/voice_mapper.py` | L2128-2144 | LLM context; only `Exploring: yes/no` |

### Data Models

*N/A — no data model changes.*

### Components

All changes are within the `VoiceMapper` class in `scripts/voice_mapper.py`. No new files, classes, or ROS2 nodes.

## Dependencies

### Prerequisites

| Dependency | Status | Notes |
|-----------|--------|-------|
| Research 006 complete | ✅ | All 5 phases complete, root causes ranked |
| `scripts/voice_mapper.py` accessible | ✅ | Single file, ~2893 lines |
| No concurrent PRs modifying exploration functions | ⚠️ Verify | Check before starting Phase 1 |

### External Dependencies

None. All changes use existing Python stdlib (`threading`, `time`, `math`, `random`) and ROS2 types (`Twist`) already imported.

## Risks

| ID | Risk | Likelihood | Impact | Mitigation |
|----|------|-----------|--------|------------|
| R1 | **±30° cone too narrow** — robot clips obstacles at 35–45° during forward motion | Low | Medium | Nav2 costmap + collision monitor provide secondary protection when active; `choose_exploration_direction()` still reads all sectors for *direction selection* (just not for stop/reverse triggering); can widen to ±45° if field testing shows clips |
| R2 | **Forward arc turns push robot into obstacle** — old reverse logic avoided obstacles, new forward turns approach them | Low | Medium | 0.3m threshold + 0.08 m/s speed gives 3.75s to clear; `_random_exploration_walk()` doorway code already uses forward turns at 0.08 m/s successfully; stuck_count escalation to walk provides escape |
| R3 | **Exploration thread speaks while voice loop processes** — background `speak()` may conflict with voice loop TTS | Low | Low | `self.speaking` flag already gates voice loop (L2694); background speak sets this flag; worst case: voice input delayed by ~3s |
| R4 | **LLM context tokens increase** — ~50 extra tokens per call during exploration | Very Low | Very Low | Only added when `self.exploring` is True; GPT-4o context window is 128K; 50 tokens is negligible |
| R5 | **Line numbers drift** — Phase 1 edits shift line numbers for Phase 2 targets | Medium | Low | Phase 2 tasks reference function names, not line numbers; coder should locate by function signature |

## Execution Plan

### Phase 1: Core Fixes — Cone, Turn Direction, Threshold, Stuck Escalation (P1–P3, partial P4)

**Goal**: Eliminate all causes of "no movement" and "backwards movement" — zero reverse commands in exploration after this phase.
**Size**: Small (6 tasks, 1 file)
**Prerequisites**: None
**Entry point**: `scripts/voice_mapper.py`

| Task | Description | File | Location | Status | Acceptance Criteria |
|------|-------------|------|----------|--------|-------------------|
| 1.1 | Add `self.explore_obstacle_dist = 0.3` state variable | `scripts/voice_mapper.py` | After L359 (`self.look_interval = 15.0`) | ✅ | New attribute exists; value is `0.3`; `self.min_obstacle_dist` unchanged at `0.5` |
| 1.2 | Fix `_reactive_exploration_step()`: narrow cone to `front_wide` only, change threshold to `self.explore_obstacle_dist`, replace `linear.x = -0.05` with `0.08` | `scripts/voice_mapper.py` | L2552-2574 | ✅ | Function uses only `front_wide` (±30°); threshold is `self.explore_obstacle_dist` (0.3m); obstacle avoidance uses `linear.x = 0.08` (forward); no reverse commands |
| 1.3 | Fix `_reactive_exploration_loop()` obstacle detection: narrow cone to `front_wide` only (remove `front`, `fl`, `fr` variables at L2600-2604), change `min_front` to `front_wide`, change threshold at L2606 to `self.explore_obstacle_dist` | `scripts/voice_mapper.py` | L2599-2606 | ✅ | Obstacle check uses only `front_wide`; threshold is `self.explore_obstacle_dist` |
| 1.4 | Fix `_reactive_exploration_loop()` turn direction: change `twist.linear.x = -0.05` at L2628 to `0.08`; remove `fl > fr` from turn condition at L2629 (variables removed in Task 1.3) [R1] | `scripts/voice_mapper.py` | L2627-2629 | ✅ | Obstacle avoidance turn uses `linear.x = 0.08` (forward arc); turn condition is `left > right` only (no `fl > fr`); no reverse commands |
| 1.5 | Fix `choose_exploration_direction()`: change `min_front` to use `front_wide` only, change threshold to `self.explore_obstacle_dist` [R4] | `scripts/voice_mapper.py` | L2094-2100 | ✅ | `min_front = front_wide`; threshold is `self.explore_obstacle_dist`; `slow_dist` check at L2110 code unchanged but now uses ±30° reading (intentional — side objects no longer gate speed); direction preference logic (L2110-2124) otherwise unchanged |
| 1.6 | Replace `_reactive_exploration_loop()` stuck backup: change `stuck_count > 5` reverse backup block (L2615-2623) to `stuck_count > 3` → `_random_exploration_walk()` call + `stuck_count = 0` + `continue` [R2] | `scripts/voice_mapper.py` | `_reactive_exploration_loop()`, L2615-2623 | ✅ | At stuck_count > 3: calls `_random_exploration_walk()`, resets stuck_count, continues loop; old 1.5s reverse backup block (`linear.x = -0.1`) removed; **no reverse commands remain in any exploration function** |

**Verification checkpoint**: After Phase 1, the robot should:
- Move forward when exploring — **no reverse commands remain in any exploration code path**
- Only trigger obstacle avoidance when something is within 0.3m of the ±30° forward cone
- Escalate to `_random_exploration_walk()` after 3 stuck iterations (not 5)
- `choose_exploration_direction()` uses the same narrow cone and threshold

### Phase 2: Reliability & Context — Non-blocking Speak, LLM Telemetry (P5–P6)

**Goal**: Unblock exploration thread from TTS and give the LLM/user visibility into exploration state.
**Size**: Small (6 tasks, 1 file)
**Prerequisites**: Phase 1 complete (new `self.explore_obstacle_dist` attribute must exist)
**Entry point**: `scripts/voice_mapper.py`

| Task | Description | File | Location | Status | Acceptance Criteria |
|------|-------------|------|----------|--------|-------------------|
| 2.1 | Add exploration telemetry state variables: `self.explore_mode`, `self.last_meaningful_movement`, `self.explore_frontier_count` | `scripts/voice_mapper.py` | After `self.explore_obstacle_dist` (added in Phase 1) | ✅ | Three new attributes initialized: `explore_mode = "idle"`, `last_meaningful_movement = 0.0`, `explore_frontier_count = 0` |
| 2.2 | Non-blocking `speak()` in `_random_exploration_walk()`: wrap `self.speak("I see an opening...")` at L2449 in `threading.Thread(..., daemon=True).start()` | `scripts/voice_mapper.py` | `_random_exploration_walk()`, L2449 | ✅ | `speak()` runs in background thread; exploration movement not paused by TTS |
| 2.3 | Set `self.explore_mode` at entry points: `"nav2-frontier"` in `exploration_loop()`, `"reactive"` in `_reactive_exploration_loop()`, `"random-walk"` in `_random_exploration_walk()`, `"idle"` in `stop_exploration()` | `scripts/voice_mapper.py` | Top of each function | ✅ | `explore_mode` reflects current exploration strategy at all times |
| 2.4a | Add `self.explore_frontier_count = len(frontiers)` inside `choose_frontier()` right after L759 (`frontiers = self.find_frontiers()`) [R3] | `scripts/voice_mapper.py` | `choose_frontier()`, L759 | ✅ | Frontier count stored as side effect of existing `find_frontiers()` call; no redundant computation |
| 2.4b | Update `exploration_loop()` to track movement telemetry: set `self.last_meaningful_movement = time.time()` at start and when nav goal succeeds | `scripts/voice_mapper.py` | `exploration_loop()`, ~L2335 and L2389 | ✅ | `last_meaningful_movement` updated on successful navigation |
| 2.5 | Enhance `get_context()`: when `self.exploring` is True, append Nav2 status, explore_mode, frontier count, and stuck/moving indicator | `scripts/voice_mapper.py` | `get_context()`, after existing context string (~L2140) | ✅ | Context includes 4 new lines during exploration; no change when not exploring |
| 2.6 | Reset telemetry in `stop_exploration()`: set `explore_mode = "idle"`, `explore_frontier_count = 0` | `scripts/voice_mapper.py` | `stop_exploration()`, after `self.exploring = False` | ✅ | Telemetry reset to defaults when exploration ends |

**Verification checkpoint**: After Phase 2, the robot should:
- Never block exploration movement for TTS
- Report accurate exploration state in LLM context (mode, Nav2 status, frontier count, progress)

## Standards

- All changes confined to `scripts/voice_mapper.py`
- Ackerman steering constraint: robot cannot turn in place; minimum forward speed required for steering
- Follow existing code patterns (LiDAR sector naming, Twist publishing, logging conventions)

## Complexity Assessment

| Factor | Score (1-5) | Notes |
|--------|-------------|-------|
| Files to modify | 1 | Single file: `scripts/voice_mapper.py` |
| New patterns introduced | 1 | Only reuses existing patterns (forward arc turns, `threading.Thread`) |
| External dependencies | 1 | None — stdlib only, all already imported |
| Migration complexity | 1 | No data migration, no config changes |
| Test coverage required | 3 | Behavioral changes need field testing on robot; no unit test framework in place |
| **Overall Complexity** | **7/25** | **Low** — Focused parameter + control flow changes in a single file |

## Review Summary

**Reviewer**: pch-plan-reviewer
**Date**: 2026-02-25
**Plan Quality**: High after review

### Issues Found and Resolved

| # | Severity | Issue | Resolution |
|---|----------|-------|------------|
| 1 | Critical | `fl > fr` NameError at L2629 after Change 3a removes variables | [R1] Drop `fl > fr`, use `left > right` only |
| 2 | Major | Phase 1 checkpoint falsely claims "no reverse" while stuck backup still reverses | [R2] Moved stuck backup fix (Task 2.2) into Phase 1 as Task 1.6 |
| 3 | Minor | Redundant `find_frontiers()` call in telemetry tracking | [R3] One-line addition in `choose_frontier()` instead |
| 4 | Minor | Implicit speed-selection behavior change undocumented | [R4] Documented as intentional improvement |

### Quality Gate Checklist

- [x] All Critical and Major issues resolved
- [x] User has answered all clarifying questions (4/4)
- [x] Every task has specific file paths and acceptance criteria
- [x] All dependencies verified (imports, attributes, function signatures)
- [x] Security implications acknowledged (N/A — no auth/network changes)
- [x] Test coverage requirements defined (field testing; no unit test framework)
- [x] Phase independence validated (Phase 2 depends only on `self.explore_obstacle_dist` from Phase 1)
- [x] No `fl`/`fr` references remain after cone narrowing
- [x] No reverse commands remain in exploration after Phase 1

## Handoff

```
Plan Approved — Ready for Implementation

Plan at: docs/plans/005-explore-mode-fix.md
Issues Found: 4 (all resolved)
Plan Quality: High after review

Next Step: Use /pch-coder to begin implementing the plan.
```
