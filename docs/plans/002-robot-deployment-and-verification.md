---
id: "002"
type: plan
title: "Robot-Side Deployment, Driver Installation & Hardware Verification"
status: ✅ Ready for Implementation
created: "2026-02-24"
updated: "2026-02-24"
owner: pch-planner
version: v2.1
---

## Version History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| v1.0 | 2026-02-24 | pch-planner | Initial plan skeleton |
| v1.1 | 2026-02-24 | pch-planner | Decision: step-by-step SSH commands |
| v1.2 | 2026-02-24 | pch-planner | Decision: Isaac VSLAM best-effort |
| v1.3 | 2026-02-24 | pch-planner | Decision: batch-fix deviations after discovery |
| v2.0 | 2026-02-24 | pch-planner | All phases detailed; holistic review complete |
| v2.1 | 2026-02-24 | pch-plan-reviewer | Added .rosmaster_llm_env generation sub-step to Phase 5 Step 5.1 |
## Introduction

Plan 001 completed all local code changes for the OAK-D camera transition, but deferred 6 SSH-based tasks that require on-robot execution. This plan covers the full robot-side deployment: script transfer, disk cleanup, DepthAI driver installation, OAK-D hardware discovery and verification, assumption validation, Isaac VSLAM installation, service activation, and end-to-end integration testing. All work is executed via SSH to `jetson@192.168.7.250`.

## Planning Session Log

| # | Decision Point | Answer | Rationale |
|---|----------------|--------|-----------|
| 1 | Execution strategy for SSH tasks | A — Step-by-step SSH commands | Maximum visibility and control; operator runs commands one at a time from dev machine, checking output between steps; appropriate for hardware-dependent operations with gated dependencies |
| 2 | Isaac VSLAM priority | B — Best-effort, don't block | Existing stereo-topic guard handles missing VSLAM gracefully; attempt install costs ~5 min with zero risk; blocking is counterproductive given uncertain JetPack R36 apt availability |
| 3 | Assumption deviation handling | C — Document deviations, batch-fix after discovery | Complete full discovery pass, collect all deviations into a table, then batch-fix in dev repo in one commit and redeploy once; minimizes round-trips, keeps repo authoritative, clean git history |

## Review Session Log

**Questions Pending:** 0  
**Questions Resolved:** 1  
**Last Updated:** 2026-02-24

| # | Issue | Category | Decision | Plan Update |
|---|-------|----------|----------|-------------|
| 1 | Service EnvironmentFile requires `.rosmaster_llm_env` but no step generates it | Correctness | Option B: Add explicit env file generation sub-step to 5.1 | Step 5.1 updated with `sed` command |

## Overview

### Parent Plan

This plan is a continuation of [001-oakd-camera-transition.md](001-oakd-camera-transition.md), specifically covering the deferred SSH-based tasks from Phases 1, 2, 4.8, 5, and 6.6.

### Deferred Tasks from Plan 001

| Plan 001 Ref | Description | Category |
|--------------|-------------|----------|
| Phase 1 (all) | Disk cleanup & DepthAI driver installation | Driver Install |
| Phase 2 (all) | OAK-D Pro discovery, topic/frame/orientation verification | Hardware Discovery |
| Phase 4.8 | Deploy scripts to robot & test service | Deployment |
| Phase 5 (all) | Isaac VSLAM package install & verification | VSLAM |
| Phase 6.6 | Remove old ascamera workspace references | Cleanup |

### Objectives

1. **Deploy** updated scripts from dev machine to robot via `scp`
2. **Clean disk** — remove RealSense packages, old logs, Docker artifacts, apt cache
3. **Install DepthAI ROS2 driver** and OAK-D udev rules
4. **Discover & verify** OAK-D hardware: model, USB speed, topics, TF frames, orientation
5. **Validate assumptions** made in Plan 001 code — update scripts if deviations found
6. **Install Isaac VSLAM** package (if available for JetPack R36)
7. **Activate & test** `voice_mapper.service` end-to-end
8. **Clean up** old HP60C/ascamera workspace artifacts on robot

## Requirements

### Functional

1. All updated scripts deployed to `/home/jetson/robot_scripts/` on robot
2. DepthAI ROS2 driver installed and OAK-D Pro publishes to `/oak/*` topics
3. OAK-D model, USB speed, TF frames, and orientation documented
4. Any assumption deviations from Plan 001 corrected in deployed scripts
5. `voice_mapper.service` auto-starts and reaches "Mapper ready" state
6. Isaac VSLAM installed (if apt package available) and verified with voice command
7. Old ascamera workspace files identified and optionally removed

### Non-Functional

1. Disk usage < 85% after all installations
2. Service starts within 60 seconds from boot
3. OAK-D Pro RGB at ≥15 fps @ 640×480
4. All operations non-destructive to existing Yahboom workspaces

### Out of Scope

1. Local code changes (completed in Plan 001)
2. Hybrid SLAM, RTABmap, multi-camera
3. OAK-D IR illumination configuration

## Technical Design

### Robot Connection

```
Host: jetson@192.168.7.250
Password: yahboom
ROS_DOMAIN_ID: 62
Script directory: /home/jetson/robot_scripts/
```

### Deployment Manifest

Files to deploy from `scripts/` on dev machine to `/home/jetson/robot_scripts/` on robot:

| File | Action | Notes |
|------|--------|-------|
| `start_robot.sh` | NEW — deploy | Wrapper script for systemd service |
| `voice_mapper.py` | UPDATE | HP60C removed, OAK-D Pro default |
| `voice_mapper.service` | UPDATE | ExecStart now uses start_robot.sh |
| `nav2_params.yaml` | UPDATE | Depth topics → /oak/points |
| `rosmaster_control.sh` | UPDATE | Fixed bringup, workspace sources |
| `02_setup_depth_camera.sh` | UPDATE | oakd camera type support |
| `llm_robot_brain.py` | CHECK | Verify current on robot (no changes in Plan 001) |

### Assumption Validation Matrix

These assumptions were made in Plan 001 code. Phase 2 (discovery) validates them.

| Assumption | Used In | Validation Method | If Wrong |
|------------|---------|-------------------|----------|
| Camera RGB topic: `/oak/rgb/image_raw` | voice_mapper.py, start_robot.sh | `ros2 topic list` | Update CAMERA_CONFIGS, start_robot.sh readiness check |
| Depth topic: `/oak/stereo/image_raw` | voice_mapper.py | `ros2 topic list` | Update CAMERA_CONFIGS |
| Point cloud topic: `/oak/points` | nav2_params.yaml | `ros2 topic list` | Update all 3 refs in nav2_params.yaml |
| Stereo topics: `/oak/left/image_rect`, `/oak/right/image_rect` | voice_mapper.py VSLAM | `ros2 topic list` | Update CAMERA_CONFIGS |
| TF base frame: `oak-d-base-frame` | start_robot.sh static TF | `ros2 run tf2_tools view_frames` | Update static transform publisher |
| Camera mounted upside-down | start_robot.sh (roll=π) | Visual inspection of saved frame | May remove or adjust rotation |
| Launch file: `camera.launch.py` | start_robot.sh, 02_setup_depth_camera.sh | `ros2 launch ... --show-args` | Update launch commands |
| Workspace paths: `~/yahboomcar_ros2_ws/...` | start_robot.sh, rosmaster_control.sh | `ls ~/yahboomcar_ros2_ws/...` | Update source paths |
| `camera_model:=OAK-D-PRO` param | start_robot.sh | `--show-args` check | Update or remove param |

### Data Contracts

No data entities in scope — data contracts not applicable.

### Codebase Patterns

```yaml
codebase_patterns:
  - pattern: "SSH command execution"
    location: "scripts/01_ssh_connect.sh"
    usage: "Reference for SSH patterns to robot"
  - pattern: "apt package install on Jetson"
    location: "Plan 001 Phase 1 specification"
    usage: "Follow same apt install + verify pattern"
  - pattern: "systemd service management"
    location: "scripts/rosmaster_control.sh service_install()"
    usage: "Use systemctl daemon-reload, restart, journalctl"
```

## Dependencies

| Dependency | Type | Required By | Status |
|------------|------|-------------|--------|
| SSH access to robot (jetson@192.168.7.250) | Infrastructure | All phases | ⚠️ Verify (robot must be powered on, on network) |
| Plan 001 code changes committed locally | Software | Phase 1 | ✅ Complete |
| OAK-D physically connected to USB | Hardware | Phase 2+ | ✅ Detected previously (bootloader mode) |
| Internet on robot (for apt install) | Infrastructure | Phase 2, Phase 4 | ✅ WiFi connected |
| Jetson GPU (CUDA) | Hardware | Phase 4 (VSLAM) | ✅ Available (Orin Nano) |

## Risks

| # | Risk | Likelihood | Impact | Mitigation |
|---|------|------------|--------|------------|
| R1 | Robot not reachable at 192.168.7.250 | Medium | High | Verify SSH connectivity first; DHCP may have changed IP |
| R2 | Disk space insufficient after cleanup + installs | Low | High | Verify df -h after each install step; cleanup first |
| R3 | Assumed topic/frame names differ from actual | Medium | Medium | Phase 2 validates all; correction scripts ready |
| R4 | Isaac VSLAM not available in apt | Medium | Medium | Try install; if fails, skip — Phases 1-3 still deliver full camera value |
| R5 | OAK-D on USB 2.0 (insufficient bandwidth) | Medium | High | Check lsusb -t post-firmware; may need physical re-seating in USB 3.0 port |
| R6 | Workspace paths differ from assumed | Low | Low | `ls` check; update start_robot.sh source lines |
| R7 | `--remap` syntax bug in VSLAM launch | Known | Medium | Fix to `--ros-args -r` during Phase 4 verification |

## Execution Plan

### Phase 1: Connectivity & Script Deployment

**Status:** ⏳ Not Started  
**Size:** Small (5 tasks, 2 files modified on robot)  
**Prerequisites:** Robot powered on, connected to network, Plan 001 changes committed in dev repo  
**Entry Point:** PowerShell terminal on dev machine  
**Verification:** All 7 deployment manifest files present on robot with correct sizes

| Step | Task | Acceptance Criteria |
|------|------|---------------------|
| 1.1 | Verify SSH connectivity | SSH connects and returns hostname `jetson` |
| 1.2 | Check disk space before deployment | `df -h /` shows ≥ 3 GB free |
| 1.3 | Backup existing scripts on robot | `~/robot_scripts_backup_$(date)` directory created |
| 1.4 | Deploy all updated scripts via scp | All 7 files from deployment manifest transferred |
| 1.5 | Verify deployed files and set permissions | All files present, `start_robot.sh` executable |

#### Step 1.1 — Verify SSH Connectivity

```bash
# From dev machine PowerShell
ssh jetson@192.168.7.250 "hostname; uname -r; cat /etc/nv_tegra_release 2>/dev/null | head -1"
```

**Expected output:** Hostname `jetson`, Linux kernel version, JetPack release info.  
**If fails:** Robot may be off or IP changed. Try `ping 192.168.7.250`. If no response, check robot power and network.

#### Step 1.2 — Check Disk Space

```bash
ssh jetson@192.168.7.250 "df -h / ; echo '---'; du -sh ~/robot_scripts/ 2>/dev/null; echo '---'; du -sh /opt/ros/ 2>/dev/null"
```

**Expected:** Root filesystem ≥ 3 GB free. Note current `robot_scripts/` size for comparison.  
**If < 3 GB:** Phase 2 disk cleanup becomes critical — proceed but note urgency.

#### Step 1.3 — Backup Existing Scripts

```bash
ssh jetson@192.168.7.250 "cp -r ~/robot_scripts ~/robot_scripts_backup_$(date +%Y%m%d_%H%M%S) && echo 'Backup created' && ls ~/robot_scripts_backup_*"
```

**Expected:** Backup directory created with all current scripts preserved.

#### Step 1.4 — Deploy Scripts via scp

```bash
# From dev machine, in c:\repos\robot directory
scp scripts/start_robot.sh jetson@192.168.7.250:~/robot_scripts/start_robot.sh
scp scripts/voice_mapper.py jetson@192.168.7.250:~/robot_scripts/voice_mapper.py
scp scripts/voice_mapper.service jetson@192.168.7.250:~/robot_scripts/voice_mapper.service
scp scripts/nav2_params.yaml jetson@192.168.7.250:~/robot_scripts/nav2_params.yaml
scp scripts/rosmaster_control.sh jetson@192.168.7.250:~/robot_scripts/rosmaster_control.sh
scp scripts/02_setup_depth_camera.sh jetson@192.168.7.250:~/robot_scripts/02_setup_depth_camera.sh
scp scripts/llm_robot_brain.py jetson@192.168.7.250:~/robot_scripts/llm_robot_brain.py
```

**Expected:** Each scp completes with 100% transfer. 7 files total.

#### Step 1.5 — Verify Deployed Files & Set Permissions

```bash
ssh jetson@192.168.7.250 "ls -la ~/robot_scripts/{start_robot.sh,voice_mapper.py,voice_mapper.service,nav2_params.yaml,rosmaster_control.sh,02_setup_depth_camera.sh,llm_robot_brain.py}"
```

```bash
ssh jetson@192.168.7.250 "chmod +x ~/robot_scripts/start_robot.sh ~/robot_scripts/rosmaster_control.sh ~/robot_scripts/02_setup_depth_camera.sh"
```

**Expected:** All 7 files listed with recent timestamps. Shell scripts have execute permission.

### Phase 2: Disk Cleanup & DepthAI Installation

**Status:** ⏳ Not Started  
**Size:** Small (7 tasks, 0 code files modified)  
**Prerequisites:** Phase 1 complete (scripts deployed)  
**Entry Point:** SSH into robot  
**Verification:** `ros2 pkg list | grep depthai` returns package name; `df -h /` shows ≥ 2 GB free after install

| Step | Task | Acceptance Criteria |
|------|------|---------------------|
| 2.1 | Remove old RealSense packages | `dpkg -l \| grep realsense` returns empty |
| 2.2 | Remove old ascamera/HP60C packages | No ascamera-related packages installed |
| 2.3 | Clean apt cache and old logs | ≥ 500 MB reclaimed |
| 2.4 | Clean Docker artifacts (if present) | Docker images pruned or Docker not present |
| 2.5 | Verify disk space post-cleanup | `df -h /` shows sufficient space for DepthAI install |
| 2.6 | Install DepthAI ROS2 driver + udev rules | `ros2 pkg list \| grep depthai` succeeds |
| 2.7 | Verify OAK-D USB detection | `lsusb` shows Movidius device (not in bootloader mode) |

#### Step 2.1 — Remove Old RealSense Packages

```bash
ssh jetson@192.168.7.250 "dpkg -l | grep -i realsense"
```

If packages found:
```bash
ssh jetson@192.168.7.250 "sudo apt purge -y 'librealsense2*' 'ros-humble-realsense*' && sudo apt autoremove -y"
```

**Expected:** No RealSense packages remain. If none were installed, this is a no-op.

#### Step 2.2 — Remove Old ascamera/HP60C Packages

```bash
ssh jetson@192.168.7.250 "dpkg -l | grep -i ascamera; ls ~/yahboomcar_ros2_ws/yahboomcar_ws/src/ 2>/dev/null | grep -i ascamera"
```

If ascamera packages or workspace directories found:
```bash
ssh jetson@192.168.7.250 "sudo apt purge -y 'ros-humble-ascamera*' 2>/dev/null; echo 'Done'"
```

**Note:** Do NOT delete ascamera workspace source at this point — it may have build artifacts other packages depend on. Phase 6 handles workspace cleanup after integration testing confirms everything works.

#### Step 2.3 — Clean Apt Cache and Old Logs

```bash
ssh jetson@192.168.7.250 "sudo apt clean && sudo apt autoremove -y && sudo journalctl --vacuum-time=7d && echo '---' && df -h /"
```

```bash
ssh jetson@192.168.7.250 "du -sh /var/log/*.log /var/log/*.gz 2>/dev/null; sudo find /var/log -name '*.gz' -delete; sudo find /tmp -maxdepth 1 -mtime +7 -delete 2>/dev/null; echo 'Log cleanup done'"
```

**Expected:** Combined savings ≥ 500 MB.

#### Step 2.4 — Clean Docker Artifacts (If Present)

```bash
ssh jetson@192.168.7.250 "which docker && docker system df 2>/dev/null || echo 'Docker not installed'"
```

If Docker is present and consuming significant space:
```bash
ssh jetson@192.168.7.250 "docker system prune -af && echo 'Docker pruned' && df -h /"
```

**Expected:** Docker images/containers pruned, or Docker not installed (both fine).

#### Step 2.5 — Verify Disk Space Post-Cleanup

```bash
ssh jetson@192.168.7.250 "df -h / ; echo '---'; free -h"
```

**Expected:** ≥ 2 GB free on root filesystem. DepthAI ROS2 driver typically requires ~500 MB–1 GB.  
**If < 2 GB:** Investigate further with `du -sh /home/jetson/* | sort -rh | head -10` to find large directories.

#### Step 2.6 — Install DepthAI ROS2 Driver + Udev Rules

```bash
ssh jetson@192.168.7.250 "sudo apt update && sudo apt install -y ros-humble-depthai-ros-driver ros-humble-depthai-bridge ros-humble-depthai-descriptions"
```

If apt packages not available (custom Luxonis PPA may be needed):
```bash
ssh jetson@192.168.7.250 "sudo wget -qO /etc/apt/trusted.gpg.d/luxonis.gpg https://artifacts.luxonis.com/artifactory/luxonis-deb-release/gpg && echo 'deb https://artifacts.luxonis.com/artifactory/luxonis-deb-release focal main' | sudo tee /etc/apt/sources.list.d/luxonis.list && sudo apt update && sudo apt install -y ros-humble-depthai-ros-driver"
```

Install udev rules for USB device access:
```bash
ssh jetson@192.168.7.250 "echo 'SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03e7\", MODE=\"0666\"' | sudo tee /etc/udev/rules.d/80-movidius.rules && sudo udevadm control --reload-rules && sudo udevadm trigger"
```

Verify installation:
```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && ros2 pkg list | grep depthai"
```

**Expected:** `depthai_ros_driver` (and possibly `depthai_bridge`, `depthai_descriptions`) in package list.

#### Step 2.7 — Verify OAK-D USB Detection

```bash
ssh jetson@192.168.7.250 "lsusb | grep -i 'movidius\|luxonis\|03e7'; echo '---'; lsusb -t 2>/dev/null | head -20"
```

**Expected:** Movidius device appears as `03e7:xxxx` (NOT `03e7:2485` which is bootloader mode — that indicates the device needs firmware flash or power cycle).  
**Check USB speed:** Look for `5000M` (USB 3.0) in `lsusb -t` output. If `480M` (USB 2.0), try reseating in a different USB port — 2.0 bandwidth is insufficient for simultaneous RGB+depth+stereo.

### Phase 3: OAK-D Discovery & Assumption Validation

**Status:** ⏳ Not Started  
**Size:** Medium (10 tasks, 0 code files modified — discovery only)  
**Prerequisites:** Phase 2 complete (DepthAI driver installed, OAK-D detected on USB)  
**Entry Point:** SSH into robot, source ROS2 workspace  
**Verification:** All 9 assumption validation matrix rows filled with actual values; deviation table complete

This phase validates every assumption made in Plan 001 code. **Per Decision 3, all deviations are collected first, then batch-fixed in a single commit after this phase completes.** No scripts are modified during this phase.

| Step | Task | Acceptance Criteria |
|------|------|---------------------|
| 3.1 | Launch DepthAI driver and confirm startup | `camera.launch.py` starts without errors |
| 3.2 | Capture full topic list | All `/oak/*` topics recorded |
| 3.3 | Validate RGB topic name | Actual topic compared to assumed `/oak/rgb/image_raw` |
| 3.4 | Validate depth topic name | Actual topic compared to assumed `/oak/stereo/image_raw` |
| 3.5 | Validate point cloud topic | Actual topic compared to assumed `/oak/points` |
| 3.6 | Validate stereo pair topics | Actual topics compared to assumed `/oak/left/image_rect`, `/oak/right/image_rect` |
| 3.7 | Validate TF frames | Actual base frame compared to assumed `oak-d-base-frame` |
| 3.8 | Check camera orientation (roll=π assumption) | Visual inspection of saved RGB frame |
| 3.9 | Validate workspace paths | Confirm `~/yahboomcar_ros2_ws/...` paths exist |
| 3.10 | Compile deviation report | Deviation table completed; ready for batch-fix if needed |

#### Step 3.1 — Launch DepthAI Driver

```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-PRO 2>&1 | head -50"
```

**Expected:** Driver starts, logs show camera initialized, no fatal errors.  
**If `camera_model` param not recognized:** Try without it:
```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 launch depthai_ros_driver camera.launch.py 2>&1 | head -50"
```

**Record:** Whether `camera_model:=OAK-D-PRO` is needed/valid → updates Assumption #9 in matrix.

**Leave the driver running** in this SSH session for subsequent steps. Open a second SSH session for the remaining commands.

#### Step 3.2 — Capture Full Topic List

In a second SSH session:
```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 topic list | grep oak"
```

```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 topic list"
```

**Record:** Complete list of all `/oak/*` topics. This is the reference for steps 3.3–3.6.

#### Step 3.3 — Validate RGB Topic

```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 topic info /oak/rgb/image_raw 2>&1"
```

**Expected:** Type `sensor_msgs/msg/Image`, at least 1 publisher.  
**If topic not found:** Check step 3.2 output for the actual RGB topic name (common alternatives: `/oak/rgb/image`, `/oak/rgb/preview/image_raw`, `/oak/color/image_raw`).  
**Record:** Actual topic name → Assumption #1.

If topic exists, check rate and resolution:
```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 topic hz /oak/rgb/image_raw --window 10 2>&1 | tail -3"
```

**Expected:** ≥ 15 Hz for 640×480 over USB 3.0.

#### Step 3.4 — Validate Depth Topic

```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 topic info /oak/stereo/image_raw 2>&1"
```

**If not found:** Check for alternatives: `/oak/stereo/depth`, `/oak/depth/image_raw`, `/oak/stereo/image_rect`.  
**Record:** Actual topic name → Assumption #2.

#### Step 3.5 — Validate Point Cloud Topic

```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 topic info /oak/points 2>&1"
```

**If not found:** Check for `/oak/stereo/points`, `/oak/points/cloud`, `/oak/depth/points`.  
**Record:** Actual topic name → Assumption #3. This affects 3 locations in `nav2_params.yaml`.

#### Step 3.6 — Validate Stereo Pair Topics (VSLAM)

```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 topic info /oak/left/image_rect 2>&1; ros2 topic info /oak/right/image_rect 2>&1"
```

**If not found:** Check for `/oak/left/image_raw`, `/oak/left/image_rect_raw`, `/oak/left/image_mono`.  
**Record:** Actual stereo topic names → Assumption #4. These are used in `voice_mapper.py` `CAMERA_CONFIGS['oakd']['stereo_topics']` and `start_isaac_vslam()`.

#### Step 3.7 — Validate TF Frames

```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 run tf2_tools view_frames 2>&1; ls ~/frames*.pdf 2>/dev/null || ls ~/frames*.gv 2>/dev/null"
```

If `view_frames` not available:
```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 topic echo /tf_static --once 2>&1 | head -30"
```

**Record:** Actual OAK-D base frame name (e.g., `oak-d-base-frame`, `oak_frame`, `oak-d_frame`) → Assumption #5. This affects the static TF publisher in `start_robot.sh`.

#### Step 3.8 — Check Camera Orientation

Save a test image and transfer to dev machine for visual inspection:
```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 run image_transport republish raw --ros-args --remap in:=/oak/rgb/image_raw --remap out:=/test_image &"
```

Alternative — save directly with a one-shot Python script:
```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && python3 -c \"
import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init()
n = Node('snap')
br = CvBridge()
def cb(msg):
    cv2.imwrite('/tmp/oak_test.jpg', br.imgmsg_to_cv2(msg, 'bgr8'))
    print('Saved /tmp/oak_test.jpg')
    rclpy.shutdown()
n.create_subscription(Image, '/oak/rgb/image_raw', cb, 1)
rclpy.spin(n)
\""
```

Then transfer:
```bash
scp jetson@192.168.7.250:/tmp/oak_test.jpg ./oak_test.jpg
```

**Inspect:** Is the image upside-down? If YES → roll=π static TF is correct. If NO → remove the roll rotation from `start_robot.sh`.  
**Record:** Orientation result → Assumption #6.

#### Step 3.9 — Validate Workspace Paths

```bash
ssh jetson@192.168.7.250 "ls -d ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash 2>&1"
```

```bash
ssh jetson@192.168.7.250 "ros2 pkg list | grep yahboomcar | head -10"
```

**Record:** Which paths exist, which are missing → Assumption #8.

#### Step 3.10 — Compile Deviation Report

After completing steps 3.1–3.9, fill in this deviation table:

```markdown
| # | Assumption | Expected Value | Actual Value | Deviation? | Files to Fix |
|---|-----------|----------------|--------------|------------|--------------|
| 1 | RGB topic | /oak/rgb/image_raw | [ACTUAL] | Yes/No | voice_mapper.py, start_robot.sh |
| 2 | Depth topic | /oak/stereo/image_raw | [ACTUAL] | Yes/No | voice_mapper.py |
| 3 | Point cloud | /oak/points | [ACTUAL] | Yes/No | nav2_params.yaml (×3) |
| 4 | Stereo topics | /oak/left/image_rect, /oak/right/image_rect | [ACTUAL] | Yes/No | voice_mapper.py |
| 5 | TF base frame | oak-d-base-frame | [ACTUAL] | Yes/No | start_robot.sh |
| 6 | Upside-down mount | Yes (roll=π) | [ACTUAL] | Yes/No | start_robot.sh |
| 7 | Launch file | camera.launch.py camera_model:=OAK-D-PRO | [ACTUAL] | Yes/No | start_robot.sh, 02_setup_depth_camera.sh |
| 8 | Workspace paths | ~/yahboomcar_ros2_ws/... | [ACTUAL] | Yes/No | start_robot.sh, rosmaster_control.sh |
| 9 | camera_model param | OAK-D-PRO | [ACTUAL] | Yes/No | start_robot.sh |
```

**If any deviations found:** Do NOT fix on robot. Record all deviations, then proceed to a batch-fix step (Phase 3a — injected between Phase 3 and Phase 4) that:

1. Updates affected source files in the dev repo (`c:\repos\robot\scripts\`)
2. Commits with message `fix: correct OAK-D topic/frame/param deviations from discovery`
3. Re-deploys corrected files via `scp` (repeat Phase 1 Step 1.4 for affected files only)

**If zero deviations:** Proceed directly to Phase 4. Stop the DepthAI driver launched in step 3.1.

### Phase 4: Isaac VSLAM Installation & Verification (Best-Effort)

**Status:** ⏳ Not Started  
**Size:** Small (5 tasks, 1 code file may be modified)  
**Prerequisites:** Phase 3 complete (camera verified, deviations fixed if any)  
**Entry Point:** SSH into robot  
**Verification:** Isaac VSLAM package installed OR documented as unavailable; `--remap` bug fixed  
**Non-blocking:** If installation fails, document the failure reason and proceed to Phase 5. This phase cannot block deployment.

| Step | Task | Acceptance Criteria |
|------|------|---------------------|
| 4.1 | Check Isaac VSLAM apt availability | Package search completed, availability documented |
| 4.2 | Install Isaac VSLAM (if available) | Package installed or failure documented |
| 4.3 | Fix `--remap` syntax bug in voice_mapper.py | `--ros-args -r` used instead of `--remap` |
| 4.4 | Test VSLAM launch (if installed) | VSLAM node starts with OAK-D stereo input |
| 4.5 | Document VSLAM status | Status recorded in plan |

#### Step 4.1 — Check Isaac VSLAM Apt Availability

```bash
ssh jetson@192.168.7.250 "apt-cache search isaac | grep -i vslam; echo '---'; apt-cache policy ros-humble-isaac-ros-visual-slam 2>&1"
```

**Expected outcomes:**
- **Available:** `ros-humble-isaac-ros-visual-slam` listed with version → proceed to 4.2
- **Not found:** Package not in any configured repo → check if Isaac ROS apt repo is configured:

```bash
ssh jetson@192.168.7.250 "cat /etc/apt/sources.list.d/isaac* 2>/dev/null || echo 'No Isaac apt repo configured'"
```

If repo not configured, try adding it:
```bash
ssh jetson@192.168.7.250 "sudo apt-key adv --fetch-keys https://isaac.download.nvidia.com/isaac-ros/repos.key 2>/dev/null; echo 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' | sudo tee /etc/apt/sources.list.d/isaac-ros.list 2>/dev/null; sudo apt update 2>&1 | tail -5"
```

#### Step 4.2 — Install Isaac VSLAM (If Available)

```bash
ssh jetson@192.168.7.250 "sudo apt install -y ros-humble-isaac-ros-visual-slam 2>&1 | tail -20"
```

**If fails with dependency errors:** Record the error message and skip to step 4.3.  
**If succeeds:** Verify:
```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && ros2 pkg list | grep isaac"
```

#### Step 4.3 — Fix `--remap` Syntax Bug

**Known issue from Plan 001 code review:** `voice_mapper.py` `start_isaac_vslam()` (~line 1440) uses `--remap` which is invalid for `ros2 launch`. Must be `--ros-args -r`.

This fix must be applied in the dev repo and redeployed:

1. In dev repo, edit `scripts/voice_mapper.py` — find the VSLAM launch command in `start_isaac_vslam()` method
2. Replace `--remap` with `--ros-args -r` in the subprocess command construction
3. Commit: `fix: correct --remap to --ros-args -r in VSLAM launch`
4. Redeploy:
```bash
scp scripts/voice_mapper.py jetson@192.168.7.250:~/robot_scripts/voice_mapper.py
```

**Note:** This fix is valuable regardless of whether VSLAM is installed — it corrects a syntax error that would cause a confusing runtime failure when the user voice-triggers "Start visual SLAM".

#### Step 4.4 — Test VSLAM Launch (If Installed)

Only if step 4.2 succeeded. First ensure the DepthAI driver is running (from Phase 3 or relaunch):

```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 launch depthai_ros_driver camera.launch.py &"
```

Then test VSLAM:
```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py --ros-args -r /stereo_camera/left/image:=/oak/left/image_rect -r /stereo_camera/right/image:=/oak/right/image_rect 2>&1 | head -30"
```

**Expected:** VSLAM node starts, subscribes to stereo topics, begins tracking.  
**If topic names differ:** Adjust remaps based on Phase 3 discovery results.

#### Step 4.5 — Document VSLAM Status

Record one of:
- ✅ **VSLAM installed and verified** — package version, test result
- ⚠️ **VSLAM installed but untested** — installed ok, test skipped/failed (reason)
- ❌ **VSLAM not available** — apt package not found for JetPack R36 (Isaac ROS may require container-based install)

**Regardless of outcome, proceed to Phase 5.**

### Phase 5: Service Activation & Integration Testing

**Status:** ⏳ Not Started  
**Size:** Medium (8 tasks, 1 file modified on robot)  
**Prerequisites:** Phase 3 complete (camera verified, deviations fixed); Phase 4 complete or skipped  
**Entry Point:** SSH into robot  
**Verification:** `voice_mapper.service` starts from boot, reaches "Mapper ready", responds to voice command, all key topics publishing

| Step | Task | Acceptance Criteria |
|------|------|---------------------|
| 5.1 | Install systemd service file | `systemctl status voice_mapper` shows loaded |
| 5.2 | Test start_robot.sh standalone | All hardware launches, readiness checks pass |
| 5.3 | Start service via systemd | Service reaches Active (running) state |
| 5.4 | Verify key topics are publishing | /scan, /odom, /oak/rgb/image_raw all active |
| 5.5 | Verify voice_mapper.py startup | "Mapper ready" appears in journal logs |
| 5.6 | Test voice command (speak test) | Robot responds to "Hello" or similar voice input |
| 5.7 | Test camera vision | Image capture from /oak/rgb/image_raw succeeds |
| 5.8 | Reboot integration test | Service auto-starts after `sudo reboot` |

#### Step 5.1 — Install Systemd Service File

First, generate the `.rosmaster_llm_env` file required by the service's `EnvironmentFile=` directive. This strips `export` prefixes from `.rosmaster_llm_config` so systemd can parse the key-value pairs:

```bash
ssh jetson@192.168.7.250 "sed 's/export //' ~/.rosmaster_llm_config > ~/.rosmaster_llm_env && chown jetson:jetson ~/.rosmaster_llm_env && echo 'Env file created' && cat ~/.rosmaster_llm_env | head -3"
```

**Expected:** File created with `KEY=value` lines (no `export` prefix). Verify API keys are present (e.g., `OPENAI_API_KEY=sk-...`).
**If `.rosmaster_llm_config` missing:** Create it first: `echo 'export OPENAI_API_KEY=sk-...' > ~/.rosmaster_llm_config` (see `06_setup_llm_brain.sh` for full setup).

Then install the service unit file:

```bash
ssh jetson@192.168.7.250 "sudo cp ~/robot_scripts/voice_mapper.service /etc/systemd/system/voice_mapper.service && sudo systemctl daemon-reload && systemctl status voice_mapper 2>&1 | head -5"
```

**Expected:** Service shows as `loaded` (inactive). Unit file path points to `/etc/systemd/system/voice_mapper.service`.

Enable for auto-start:
```bash
ssh jetson@192.168.7.250 "sudo systemctl enable voice_mapper && echo 'Service enabled'"
```

#### Step 5.2 — Test start_robot.sh Standalone (Manual Dry Run)

Before using systemd, test the startup wrapper directly to catch any issues:

```bash
ssh jetson@192.168.7.250 "cd ~/robot_scripts && source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && bash start_robot.sh 2>&1 | head -80"
```

**Expected output sequence:**
1. `[start_robot.sh] Starting robot bringup (A1)...`
2. `[start_robot.sh] Starting LiDAR...`
3. `[start_robot.sh] Starting OAK-D Pro camera...`
4. `[start_robot.sh] Publishing static TF...`
5. `[start_robot.sh] Waiting for key topics...`
6. `[start_robot.sh] All key topics detected! System ready.`
7. `[start_robot.sh] Starting voice_mapper.py...`

**If readiness check times out:** Check which topic is missing from the timeout message. Common issues:
- `/scan` missing → LiDAR USB connection issue or wrong launch file
- `/oak/rgb/image_raw` missing → DepthAI driver failed (check if topic name deviates from assumption)
- `/odom` missing → Bringup launch file issue

**Ctrl+C to stop** after confirming "System ready" message. The cleanup trap should fire and kill background processes.

#### Step 5.3 — Start Service via Systemd

```bash
ssh jetson@192.168.7.250 "sudo systemctl start voice_mapper && sleep 5 && systemctl status voice_mapper 2>&1"
```

**Expected:** Active (running) state, PID shown, no errors in recent log lines.  
**If fails:** Check full journal:
```bash
ssh jetson@192.168.7.250 "journalctl -u voice_mapper --no-pager -n 50"
```

#### Step 5.4 — Verify Key Topics Publishing

```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && echo '=== Topics ===' && ros2 topic list | grep -E 'scan|odom|oak|cmd_vel|imu' && echo '=== Hz check ===' && timeout 5 ros2 topic hz /scan --window 5 2>&1 | tail -1 && timeout 5 ros2 topic hz /oak/rgb/image_raw --window 5 2>&1 | tail -1"
```

**Expected:**
- `/scan` at ~10 Hz
- `/odom` publishing
- `/oak/rgb/image_raw` at ≥15 Hz
- `/cmd_vel` topic exists (may not be publishing until movement commanded)
- `/imu/data` publishing

#### Step 5.5 — Verify voice_mapper.py Startup

```bash
ssh jetson@192.168.7.250 "journalctl -u voice_mapper --no-pager -n 100 | grep -i 'ready\|error\|warning\|camera\|audio\|mapper'"
```

**Expected:** Logs show:
- Camera detection: "Detected camera: OAK_D_PRO" (or similar)
- Audio initialization: PulseAudio device found
- LLM provider loaded
- Final ready message

**If audio fails:** Known issue — PulseAudio index 32. Check:
```bash
ssh jetson@192.168.7.250 "pactl list sources short 2>/dev/null"
```

#### Step 5.6 — Test Voice Command

This requires physical presence near the robot's microphone, or testing via ROS2 topic:

Option A — **Physical voice test:** Speak "Hello" or "What do you see?" near the robot's USB microphone. Check logs for response:
```bash
ssh jetson@192.168.7.250 "journalctl -u voice_mapper -f --no-pager 2>&1 | head -30"
```

Option B — **Programmatic test via cmd_vel:** Send a movement command to verify the stack is responding:
```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'"
```

**Expected:** Command accepted without error (zero velocity is safe — robot stays still).

#### Step 5.7 — Test Camera Vision

Verify the GPT-4o vision pipeline can capture and process an image:

```bash
ssh jetson@192.168.7.250 "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && python3 -c \"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
rclpy.init()
n = Node('test')
received = [False]
def cb(msg):
    if not received[0]:
        print(f'Image received: {msg.width}x{msg.height}, encoding={msg.encoding}')
        received[0] = True
        rclpy.shutdown()
n.create_subscription(Image, '/oak/rgb/image_raw', cb, 1)
rclpy.spin(n)
\""
```

**Expected:** `Image received: 640x480, encoding=bgr8` (or similar resolution/encoding).

#### Step 5.8 — Reboot Integration Test

The ultimate test — full cold-boot to working system:

```bash
ssh jetson@192.168.7.250 "sudo reboot"
```

Wait ~90 seconds for Jetson to boot, then verify:
```bash
ssh jetson@192.168.7.250 "systemctl status voice_mapper 2>&1 | head -10; echo '---'; source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 topic list | grep -c 'scan\|odom\|oak'"
```

**Expected:** Service active (running), all 3 key topic categories present.  
**If service failed at boot:** Check journal for startup errors:
```bash
ssh jetson@192.168.7.250 "journalctl -u voice_mapper -b --no-pager | head -60"
```

### Phase 6: Cleanup & Documentation

**Status:** ⏳ Not Started  
**Size:** Small (5 tasks, documentation only)  
**Prerequisites:** Phase 5 complete (service verified working)  
**Entry Point:** SSH into robot  
**Verification:** No HP60C/ascamera references in active code on robot; PROGRESS.md updated; Plan 001 fully closed

| Step | Task | Acceptance Criteria |
|------|------|---------------------|
| 6.1 | Remove old ascamera workspace references on robot | No ascamera source/build artifacts in active paths |
| 6.2 | Verify no HP60C references remain on robot | `grep -r` finds zero active HP60C references |
| 6.3 | Final disk space check | `df -h /` shows healthy disk usage |
| 6.4 | Update PROGRESS.md in dev repo | Deployment status documented |
| 6.5 | Close Plan 001 deferred tasks | All Plan 001 SSH tasks marked complete or documented as N/A |

#### Step 6.1 — Remove Old ascamera Workspace References

First, check what exists:
```bash
ssh jetson@192.168.7.250 "find ~/yahboomcar_ros2_ws -name '*ascamera*' -o -name '*hp60c*' 2>/dev/null"
```

If ascamera build artifacts are found in an otherwise-active workspace:
```bash
ssh jetson@192.168.7.250 "ls ~/yahboomcar_ros2_ws/yahboomcar_ws/src/ | grep -i ascamera"
```

**Safe to remove** (built packages only, not the workspace itself):
```bash
ssh jetson@192.168.7.250 "cd ~/yahboomcar_ros2_ws/yahboomcar_ws && rm -rf build/ascamera* install/ascamera* log/ascamera* 2>/dev/null && echo 'Cleaned ascamera build artifacts'"
```

**Do NOT remove** the entire `yahboomcar_ws` — other packages live there.

Also check for any standalone ascamera workspace:
```bash
ssh jetson@192.168.7.250 "ls -d ~/ascamera_ws ~/ascamera* 2>/dev/null || echo 'No standalone ascamera directories'"
```

If standalone ascamera directories exist and are NOT referenced by any other package:
```bash
ssh jetson@192.168.7.250 "rm -rf ~/ascamera_ws && echo 'Removed standalone ascamera workspace'"
```

#### Step 6.2 — Verify No HP60C References on Robot

```bash
ssh jetson@192.168.7.250 "grep -rl 'hp60c\|HP60C\|ascamera' ~/robot_scripts/ 2>/dev/null || echo 'No HP60C references in robot_scripts'"
```

```bash
ssh jetson@192.168.7.250 "grep -rl 'hp60c\|HP60C' ~/yahboomcar_ros2_ws/yahboomcar_ws/src/ 2>/dev/null || echo 'No HP60C references in workspace source'"
```

**Expected:** No references in `robot_scripts/`. Workspace source may contain Yahboom-provided code with HP60C references — those are from the vendor and should not be modified.

#### Step 6.3 — Final Disk Space Check

```bash
ssh jetson@192.168.7.250 "df -h / ; echo '---'; du -sh /opt/ros/ ~/yahboomcar_ros2_ws/ ~/robot_scripts/ 2>/dev/null"
```

**Expected:** Root filesystem usage < 85%. Document the final state.

#### Step 6.4 — Update PROGRESS.md in Dev Repo

Update `PROGRESS.md` in the local dev repo with:
- Deployment date and status
- Phase 3 discovery results (actual topic names, any deviations found)
- VSLAM installation status
- Service verification result
- Any remaining follow-up items

This is a dev-machine file edit (not SSH).

#### Step 6.5 — Close Plan 001 Deferred Tasks

Update [001-oakd-camera-transition.md](001-oakd-camera-transition.md) to mark all deferred SSH tasks as either:
- ✅ Complete (done via Plan 002)
- ⚠️ Partial (with notes on what was and wasn't done)
- ❌ Blocked (with reason, e.g., "Isaac VSLAM not available for JetPack R36")

Final commit in dev repo:
```bash
git add -A && git commit -m "docs: close Plan 001 deferred tasks, complete Plan 002 deployment"
```

## Holistic Review

### Decision Interactions

The three decisions form a coherent strategy:

1. **Step-by-step SSH (D1) + Batch deviation handling (D3):** The operator runs all discovery commands sequentially in Phase 3, recording results as they go. This naturally feeds into the batch-fix approach — by the time discovery is complete, all deviations are known and can be fixed in one commit. If we had chosen automated scripts (D1 option B/C), deviation discovery and correction would need more complex branching logic.

2. **Best-effort VSLAM (D2) + Step-by-step execution (D1):** Since the operator manually runs each command, they naturally observe whether VSLAM installation succeeds and can make a judgment call on proceeding vs. investigating. A fully automated script would need explicit error-handling branches for VSLAM failure.

3. **Batch deviation fixes (D3) + Best-effort VSLAM (D2):** If VSLAM installation reveals additional topic name issues (e.g., actual stereo topic names differ), these corrections merge naturally into the same batch-fix commit as other deviations from Phase 3.

### Architectural Considerations

- **Single deploy, potential redeploy:** Phase 1 deploys scripts once. Phase 3 may trigger a redeploy (batch-fix). This means the robot runs "assumed" code briefly between Phases 1 and 3. This is safe because no service is activated until Phase 5 — the deployed scripts sit idle.
- **Phase ordering is critical:** Disk cleanup (P2) before driver install (P2) before discovery (P3) before service activation (P5). This ordering prevents disk space issues and ensures the driver is available for topic discovery.
- **The `--remap` bug fix (R7)** is placed in Phase 4 regardless of VSLAM availability, because the fix corrects a syntax error in shipped code that would cause confusing runtime failures later.

### Trade-offs Accepted

1. **Manual execution overhead:** Step-by-step SSH means more operator time vs. scripted deployment. Accepted because hardware-dependent tasks benefit from human judgment on intermediate results.
2. **Two-pass deployment:** If deviations are found, scripts are deployed twice (initial + corrected). Accepted because the clean git history and single-commit fix outweigh the extra scp calls.
3. **VSLAM may not be available:** Accepted because the camera transition delivers value without VSLAM, and the voice_mapper.py code already handles the missing-VSLAM case gracefully.

### Risks Acknowledged

- **R1 (Robot IP):** DHCP-assigned IP may change. Phase 1 Step 1.1 is the first gate. If it fails, the entire plan blocks until connectivity is restored.
- **R5 (USB 2.0 bandwidth):** Previous detection showed OAK-D in bootloader mode. After driver install, if the device enumerates on USB 2.0, physical re-seating is required — this needs hands-on access.
- **R7 (`--remap` bug):** This is a known pre-existing bug — it will be fixed in Phase 4 Step 4.3 regardless of VSLAM installation status.

### Gap Analysis

No significant gaps identified. The plan covers:
- All 5 deferred task groups from Plan 001
- Known bug fix (--remap syntax)
- Full assumption validation matrix (9 items)
- Non-blocking VSLAM strategy
- Reboot-level integration testing
- Rollback path (backup in Phase 1 Step 1.3)



No organizational standards applicable to this plan.

### Minor Issues for Implementer Awareness

These are informational — no plan changes required, but the operator should be aware:

1. **Phase 3.1 launch syntax may need adjustment** — `camera_model:=OAK-D-PRO` is an assumption. The plan already handles this ("Try without it" fallback + Assumption #9 recording). No change needed, just calling it out as the most likely deviation.

2. **Phase 3.8 image capture depends on `cv_bridge`** — The one-shot Python script uses `from cv_bridge import CvBridge` which requires `ros-humble-cv-bridge`. This is typically installed as a dependency of `depthai-ros`, but if the capture fails with an import error, install it: `sudo apt install -y ros-humble-cv-bridge`.

3. **Phase 2.6 Luxonis PPA URL uses `focal`** — The fallback PPA command uses `focal main` in the sources list, which is for Ubuntu 20.04. Jetson Orin Nano with JetPack 6 may run Ubuntu 22.04 (jammy). If the PPA fails, the operator should check `lsb_release -cs` and adjust the codename accordingly.

4. **Phase 4 `--remap` fix is in Phase 4.3 but Phase 3a redeploy may overwrite** — If Phase 3 finds deviations triggering a batch-fix redeploy (Phase 3a), and Phase 4.3 also modifies `voice_mapper.py`, ensure the `--remap` fix is included in the Phase 3a commit if both happen. Otherwise the Phase 3a redeploy could overwrite the Phase 4.3 fix. Recommendation: if both Phase 3a and 4.3 affect `voice_mapper.py`, combine them into one commit.

5. **Phase 5.6 voice test requires physical presence** — Option A (physical voice test) needs someone near the robot. The plan offers Option B (cmd_vel publish) which only tests the ROS2 stack, not the audio/LLM pipeline. This is an inherent limitation of SSH-based testing.

6. **Phase 1.4 uses individual `scp` commands** — Could use `scp scripts/{file1,file2,...}` glob or `rsync` for fewer commands. Minor ergonomic improvement; current approach is fine for 7 files and aligns with step-by-step visibility.

### Implementation Complexity

| Factor | Score (1-5) | Notes |
|--------|-------------|-------|
| Files to modify | 2 | 7 files deployed; 0-2 files fixed post-discovery |
| New patterns introduced | 1 | All SSH commands follow established patterns |
| External dependencies | 3 | DepthAI apt packages, optional Isaac VSLAM, USB hardware |
| Migration complexity | 1 | No data migration; file copy + service install |
| Test coverage required | 3 | Hardware integration tests, reboot test, voice test |
| **Overall Complexity** | **10/25** | **Low** — straightforward deployment with hardware verification |

## Review Summary

**Review Date:** 2026-02-24  
**Reviewer:** pch-plan-reviewer  
**Original Plan Version:** v2.0  
**Reviewed Plan Version:** v2.1

### Review Metrics
- Issues Found: 7 (Critical: 0, Major: 1, Minor: 6)
- Clarifying Questions Asked: 1
- Sections Updated: Phase 5 Step 5.1, Version History, Review Session Log

### Key Improvements Made
1. Added `.rosmaster_llm_env` generation sub-step to Phase 5 Step 5.1 — prevents service startup failure due to missing EnvironmentFile

### Remaining Considerations
- 6 minor issues documented above for implementer awareness (no plan changes needed)
- If Phase 3 discovery reveals topic deviations AND Phase 4.3 `--remap` fix both touch `voice_mapper.py`, combine into one commit
- Luxonis PPA URL may need Ubuntu codename adjustment depending on actual JetPack version

### Sign-off
This plan has been reviewed and is **Ready for Implementation**

## Handoff

| Field | Value |
|-------|-------|
| Created By | pch-planner |
| Created Date | 2026-02-24 |
| Reviewed By | pch-plan-reviewer |
| Review Date | 2026-02-24 |
| Status | ✅ Ready for Implementation |
| Next Agent | pch-coder |
| Plan Location | /docs/plans/002-robot-deployment-and-verification.md |
