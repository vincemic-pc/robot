---
id: "005"
type: research
title: "USB 3.0 Configuration Documentation Audit"
status: 🔄 In Progress
created: "2026-02-25"
current_phase: "1 of 3"
---

## Introduction

The OAK-D Pro camera has been successfully migrated to USB 3.0 SuperSpeed (5 Gbps) on the ROSMASTER A1 robot. The fix involved physical port migration (USB-C → Type-A), udev rules for LPM disable, kernel boot parameters, and DepthAI config changes (`i_usb_speed: "SUPER"`, 30fps stereo, 15fps RGB). All project documentation and copilot instructions must be audited and updated to reflect this new working configuration so it can be reproduced from scratch.

## Objectives

- Identify all documentation files containing outdated USB 2.0 references or stale camera configuration details
- Document the exact before/after state for every file that needs updating
- Ensure the copilot instructions accurately reflect current hardware topology, FPS rates, USB speed, and VSLAM pipeline (no resize nodes)
- Capture the full reproducible setup procedure (physical port, udev, kernel params, YAML, launch file)
- Update PROGRESS.md with completed USB 3.0 milestone

## Research Phases

| Phase | Name | Status | Scope | Session |
|-------|------|--------|-------|---------|
| 1 | Documentation Inventory & Gap Analysis | ✅ Complete | Scan all docs and scripts for outdated USB/camera references; catalog every stale value (FPS, speed, bandwidth, topology) with file paths and line numbers | 2026-02-25 |
| 2 | Copilot Instructions Deep Audit | ⏳ Not Started | Full review of `.github/copilot-instructions.md` for stale Architecture diagram, Hardware table, OAK-D Pro pattern, topics table, dependencies section, VSLAM sections; produce exact before/after diffs | — |
| 3 | PROGRESS.md & Plan 004 Closure | ⏳ Not Started | Audit PROGRESS.md for USB 3.0 milestone recording; audit plan 004 for status update to "complete"; document the canonical reproducible setup procedure (physical + software) | — |

## Phase 1: Documentation Inventory & Gap Analysis

**Status:** ✅ Complete  
**Session:** 2026-02-25

### Stale Documentation Inventory

Complete scan of all documentation files, scripts, and config files for outdated USB 2.0/camera references. Every stale value catalogued with file path, line numbers, current text, and correct replacement value.

---

### 1. PROGRESS.md — Major Staleness (6 items)

| Line(s) | Stale Content | Correct Value | Notes |
|---------|---------------|---------------|-------|
| 70 | `OAK-D-PRO (Luxonis) on USB 2.0 (5-10 fps, 1280x720)` | `OAK-D-PRO (Luxonis) on USB 3.0 SuperSpeed — stereo 720P@30fps, RGB 1080P@15fps` | In "Recently Completed (Plan 002)" section |
| 114 | `[ ] Move OAK-D Pro to USB 3.0 port for better FPS (currently 5-10 fps on USB 2.0)` | Should be marked `[x]` with updated description reflecting completion | In "4. USB 3.0 Port Migration" under In Progress |
| 115 | `[ ] Verify 640x480@30fps after port migration` | Should be marked `[x]` — actual verified config is stereo 720P@30fps + RGB 1080P@15fps | Wrong expected resolution was pre-migration guess |
| 128 | `OAK-D-PRO (Luxonis), DepthAI v2.12.2, USB 2.0 (5-10 fps)` | `OAK-D-PRO (Luxonis), DepthAI v2.12.2, USB 3.0 SuperSpeed (stereo 30fps, RGB 15fps)` | In Hardware Status table |
| 140 | `[ ] Move OAK-D Pro to USB 3.0 port (currently USB 2.0, limiting to 5-10 fps)` | Should be marked `[x]` completed | In "Next Steps → Immediate" |
| 7 | `Latest Milestone: OAK-D Pro camera fully deployed and verified on robot (Plan 002 complete)` | Should mention USB 3.0 SuperSpeed milestone (Plan 004 complete) | Header milestone is stale |

---

### 2. .github/copilot-instructions.md — Moderate Staleness (1 item)

| Line | Stale Content | Correct Value | Notes |
|------|---------------|---------------|-------|
| 135 | `/oak/rgb/image_raw` listed as `640x480@30fps` | Should be `1080P@15fps` (1920x1080 via MJPEG at 15fps per oakd_params.yaml) | Key ROS2 Topics table — resolution AND fps are both wrong |

**Note:** The rest of copilot-instructions.md correctly says "USB 3.0". However, the detailed Phase 2 audit should verify the Architecture diagram has no additional stale details and that topics like `/oak/left/image_rect` and `/oak/right/image_rect` are documented with their FPS/resolution.

---

### 3. README.md — Extensive Staleness (30+ items, mostly HP60C legacy)

The README is the most stale file. The entire document still describes the HP60C as the primary camera.

#### HP60C References (should be replaced or removed)

| Line(s) | Stale Content | Action Needed |
|---------|---------------|---------------|
| 5 | `🎥 **3D Depth Camera** setup and streaming (Angstrong HP60C)` | Replace with OAK-D Pro |
| 30 | `RGB Camera` row: `HP60C RGB works but gain=4` | Replace with OAK-D Pro status |
| 31 | `Depth Camera` row: `HP60C USB 2.0 - use Yahboom's get_dist(x,y)` | Replace with OAK-D Pro USB 3.0 |
| 51 | Components table: `Angstrong HP60C` / `USB 2.0` | Replace with `OAK-D Pro (Luxonis)` / `USB 3.0 Type-A` |
| 100-106 | "RGB Camera - Dark Images" section about HP60C | Remove or archive (HP60C no longer installed) |
| 108-132 | "Depth Camera - USB 2.0 Bandwidth Limitation" section | Remove entirely (not applicable to OAK-D Pro) |
| 136-137 | Depth topics `/ascamera_hp60c/...` | Remove (HP60C topics no longer exist) |
| 144 | Prerequisites: `Angstrong HP60C Depth Camera` | Replace with `OAK-D Pro (Luxonis)` |
| 166 | ROS2 Packages: `ascamera - Nuwa camera driver` | Replace with `depthai-ros - OAK-D Pro driver` |
| 192 | Topics table: `/ascamera_hp60c/.../rgb0/image` at `640x480@20fps` | Replace with `/oak/rgb/image_raw` at `1080P@15fps` |
| 193 | Topics table: `/ascamera_hp60c/.../depth0/image` | Replace with `/oak/stereo/image_raw` |
| 205 | Active Nodes: `/ascamera_hp60c/camera_publisher` | Replace with OAK-D Pro nodes |
| 254-258 | Quick Start Step 3 mentions "Nuwa HP60C camera" | Replace with OAK-D Pro and depthai-ros |
| 263-271 | "Option A: Yahboom-Style Explorer" promoted as recommended | Demote — voice_mapper is the primary |
| 310 | "Uses Yahboom's proven depth camera pattern - better for USB 2.0 bandwidth constraints" | Update: deprecated, HP60C-only |
| 340 | Architecture diagram: `Camera HP60C` | Replace with `OAK-D Pro` |
| 375-400 | Yahboom Explorer Architecture section (HP60C-centric) | Mark as deprecated/legacy |
| 540 | Architecture diagram: `HP60C/OAK-D • (auto-detect)` and `HP60C (current)` | Replace with OAK-D Pro only |

#### Camera Environment Variable

| Line | Stale Content | Correct Value |
|------|---------------|---------------|
| 688 | `CAMERA_TYPE: nuwa, astra, or realsense (default: nuwa)` | Default should be `oakd`; nuwa is no longer primary |

#### VSLAM Camera Table

| Line | Stale Content | Action |
|------|---------------|--------|
| 699 | `Angstrong HP60C` row: `❌ Not Suitable / USB 2.0 only, no stereo` | Remove row — HP60C is no longer installed |

#### Troubleshooting Section

| Line(s) | Stale Content | Action |
|---------|---------------|--------|
| 1013-1050 | "Dark/black images from HP60C camera" troubleshooting | Remove or archive — HP60C no longer installed |
| 1051-1078 | HP60C camera config/launch/topic verification | Replace with OAK-D Pro troubleshooting |

---

### 4. scripts/rosmaster_control.sh — Minor Staleness (3 items)

| Line | Stale Content | Correct Value |
|------|---------------|---------------|
| 283 | `# Run yahboom_explorer.py (alternative for USB 2.0 cameras)` | `# Run yahboom_explorer.py (deprecated — HP60C only)` |
| 416 | `Run Yahboom Explorer (USB 2.0 camera)` | `Run Yahboom Explorer (deprecated)` |
| 482 | `yahboom         Run yahboom_explorer.py (USB 2.0 camera)` | `yahboom         Run yahboom_explorer.py (deprecated)` |

---

### 5. scripts/yahboom_explorer.py — Already Deprecated (no changes needed)

Already correctly marked as deprecated with `⚠️ DEPRECATED` header. Legacy USB 2.0 references within the deprecated file are acceptable.

---

### 6. docs/plans/004-usb3-lpm-fix-oakd-pro.md — Status Staleness (1 item)

| Line | Stale Content | Correct Value |
|------|---------------|---------------|
| 5 | `status: in-review` | `status: complete` — the USB 3.0 fix has been successfully implemented and verified |

---

### 7. Files Confirmed Clean (no updates needed)

| File | Status |
|------|--------|
| `scripts/oakd_params.yaml` | ✅ Already reflects USB 3.0 config |
| `scripts/oakd_vslam.launch.py` | ✅ Already reflects USB 3.0 (no resize, 720P@30fps) |
| `scripts/start_robot.sh` | ✅ Already updated for OAK-D Pro |
| `scripts/voice_mapper.py` | ✅ CameraConfig abstraction is clean |
| `scripts/voice_mapper.service` | ✅ No stale camera/USB references |
| `scripts/nav2_params.yaml` | ✅ Already uses `/oak/` topics |
| `scripts/llm_robot_brain.py` | ✅ No camera-specific references |
| `docs/openai-intelligence-architecture.md` | ✅ No specific camera model/USB references |
| `docs/research/001-004` | ✅ Historical records — not changed |
| `docs/plans/001-003` | ✅ Historical records — not changed |

---

### Canonical Correct Values Reference

| Parameter | Correct Value | Source |
|-----------|---------------|--------|
| USB speed | SuperSpeed / USB 3.0 / 5 Gbps | Verified via `lsusb -t` (5000M) |
| Physical port | USB Type-A (was USB-C) | Physical port migration |
| Hub chip | VIA Labs VL822 (2109:0822 SS / 2109:2822 HS) | `lsusb` output |
| DepthAI i_usb_speed | `"SUPER"` | oakd_params.yaml |
| RGB resolution | 1920x1080 (IMX378 minimum) | oakd_params.yaml |
| RGB FPS | 15 fps | oakd_params.yaml |
| RGB encoding | MJPEG (low_bandwidth: true) | oakd_params.yaml |
| Stereo (L/R) resolution | 1280x720 (OV9282 only) | oakd_params.yaml |
| Stereo (L/R) FPS | 30 fps | oakd_params.yaml |
| Stereo encoding | RAW (low_bandwidth: false) | oakd_params.yaml |
| RGB topic | `/oak/rgb/image_raw` (RELIABLE QoS) | voice_mapper.py |
| Depth topic | `/oak/stereo/image_raw` (RELIABLE QoS) | voice_mapper.py |
| Left rect topic | `/oak/left/image_rect` | voice_mapper.py |
| Right rect topic | `/oak/right/image_rect` | voice_mapper.py |
| VSLAM pipeline | mono8→rgb8 converters + Isaac VSLAM (no resize) | oakd_vslam.launch.py |
| Bandwidth budget | ~60 MB/s (~15% of VL822 capacity) | oakd_params.yaml comments |

**Key Discoveries:**
- **README.md is the most stale file** with 30+ outdated references — still describes HP60C as primary camera with USB 2.0 constraints throughout
- **PROGRESS.md has 6 stale items** — still claims USB 2.0 at 5-10fps with unchecked USB 3.0 migration tasks
- **copilot-instructions.md has 1 stale entry** — Key ROS2 Topics table claims `/oak/rgb/image_raw` is `640x480@30fps` when it's actually `1080P@15fps`
- **Plan 004 status is stale** — still `in-review` but the fix is fully implemented and verified
- **rosmaster_control.sh has 3 minor stale references** — labels yahboom_explorer as "USB 2.0 camera" instead of "deprecated"
- **All script files (voice_mapper.py, start_robot.sh, oakd_params.yaml, oakd_vslam.launch.py) are already correct** — no code changes needed

| File | Relevance |
|------|-----------|  
| `PROGRESS.md` | 6 stale USB 2.0 / 5-10fps references |
| `README.md` | 30+ stale HP60C / USB 2.0 references (most impacted file) |
| `.github/copilot-instructions.md` | 1 stale FPS/resolution value in topics table |
| `docs/plans/004-usb3-lpm-fix-oakd-pro.md` | status field still `in-review` |
| `scripts/rosmaster_control.sh` | 3 "USB 2.0 camera" labels for yahboom |

**Gaps:** None  
**Assumptions:** Plan 004 implementation is fully complete based on terminal history showing USB 3.0 SuperSpeed confirmed. RGB resolution is 1080P based on `oakd_params.yaml` `i_resolution: "1080"`.

## Phase 2: Copilot Instructions Deep Audit

**Status:** ⏳ Not Started  
**Session:** —

## Phase 3: PROGRESS.md & Plan 004 Closure

**Status:** ⏳ Not Started  
**Session:** —

## Overview

_To be written after all phases complete._

## Key Findings

_To be populated after all phases complete._

## Actionable Conclusions

_To be populated after all phases complete._

## Open Questions

_To be populated after all phases complete._

## Standards Applied

No organizational standards applicable to this research.

## Handoff

| Field | Value |
|-------|-------|
| Created By | pch-researcher |
| Created Date | 2026-02-25 |
| Status | 🔄 In Progress |
| Current Phase | 0 |
| Path | /docs/research/005-usb3-documentation-update.md |
