---
id: "004"
type: implementation-plan
title: "USB 3.0 LPM Fix — OAK-D Pro SuperSpeed Restoration"
status: in-review
created: "2026-02-24"
updated: "2026-02-24"
owner: pch-planner
version: v2.0
---

## Version History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| v1.0 | 2026-02-24 | pch-planner | Initial plan skeleton |
| v1.1 | 2026-02-24 | pch-planner | Added camera config decision (conservative FPS) |
| v1.2 | 2026-02-24 | pch-planner | Added persistence strategy (belt-and-suspenders) |
| v1.3 | 2026-02-24 | pch-planner | Added USB speed (AUTO) and VSLAM resize (remove) decisions |
| v1.4 | 2026-02-24 | pch-planner | Added deployment strategy (two-phase) |
| v2.0 | 2026-02-24 | pch-planner | Complete plan: requirements, technical design, execution plan, holistic review |
| v2.1 | 2026-02-24 | pch-plan-reviewer | Review corrections: fixed VSLAM converter removal bug, corrected VL817→VL822 hub topology, updated udev/kernel quirks, fixed line references |

## Introduction

This plan implements the USB 3.0 fix identified in [Research 004](../research/004-orin-nano-super-usb3-cameras.md). The OAK-D Pro camera on the ROSMASTER A1 robot falls back from USB 3.0 (5 Gbps) to USB 2.0 (480 Mbps) due to U1/U2 Link Power Management handshake failures between the Intel Movidius MyriadX VPU, VIA Labs VL822 hub chip (2109:0822 SuperSpeed / 2109:2822 Hi-Speed), and the NVIDIA tegra-xusb host controller. The camera is also currently on the USB-C port, which cannot provide USB 3.0 at all. The fix involves physical port migration, udev rules for persistent LPM disable, kernel boot parameters, and camera config updates for USB 3.0 bandwidth.

## Planning Session Log

| # | Decision Point | Answer | Rationale |
|---|----------------|--------|-----------|
| 1 | Camera config after USB 3.0 | A — Conservative: increase stereo FPS only | Stereo L/R → 30fps raw, RGB → 15fps MJPEG. Modest bandwidth increase (~60 MB/s) validates USB 3.0 stability before pushing limits. VSLAM gets smooth tracking; LLM vision gets more frequent frames. |
| 2 | Persistence strategy for LPM fix | C — Both udev rules AND kernel boot params | Kernel `usbcore.quirks` prevents LPM advertising at enumeration (earliest fix); udev rules disable hardware LPM on hub side (complementary layer). `usbfs_memory_mb=1000` only available via kernel param. Both low-risk on dedicated robot. |
| 3 | USB speed param in oakd_params.yaml | B — AUTO (auto-detect) | `AUTO` uses USB 3.0 when available, gracefully downgrades to USB 2.0 without SIGABRT crash. Resilient fallback if LPM fix ever fails transiently. Enables bandwidth benefits from decision #1 without forcing `SUPER` crash risk. |
| 4 | VSLAM resize workaround for USB 3.0 | B — Remove resize nodes, test native 720P | With USB 3.0's smoother 30fps delivery, test whether CUDA pitch error was caused by jitter. Higher resolution = better VSLAM feature matching. Keep resize version as documented rollback if CUDA error persists. |
| 5 | Deployment & verification strategy | B — Two-phase: USB fix first, then camera/VSLAM config | Phase 1: cable move + udev + kernel params + reboot → verify SuperSpeed. Phase 2: updated oakd_params.yaml + oakd_vslam.launch.py → verify FPS + VSLAM. Clear transport-vs-application isolation. |

## Review Session Log

**Reviewer:** pch-plan-reviewer
**Review Started:** 2026-02-24
**Issues Found:** 7 (Critical: 1, Major: 4, Minor: 2)
**Issues Resolved:** 7 (all corrections applied — no user decisions required)
**Last Updated:** 2026-02-24

| # | Issue | Category | Resolution | Plan Update |
|---|-------|----------|------------|-------------|
| 1 | File 4 removes ImageFormatConverterNodes — VSLAM will fail without mono8→rgb8 conversion | Critical/Correctness | Fixed: replacement code now keeps convert_left/convert_right, updates VSLAM remappings to converter outputs | File 4 code, description, Task 2.3, Risk R2 updated |
| 2 | VL817 (2109:0817) does not exist on this board — live `lsusb` shows only VL822 (0822 SuperSpeed / 2822 Hi-Speed) | Major/Correctness | Fixed: removed all VL817 references, updated udev rules and kernel quirks | Introduction, Architecture, File 1, File 2, Tasks 1.2/1.3/1.5/1.7, File 5 |
| 3 | `cudaErrorInvalidPitchValue` attribution wrong — caused by mono8→rgb8 format mismatch, not resolution/jitter | Major/Correctness | Fixed: updated Risk R2, Trade-off 1, File 4 rollback text | Risk R2, Holistic Review trade-offs |
| 4 | File 4 before description omits convert_left/convert_right (2 of 5 composable nodes) | Major/Completeness | Fixed: enumerated all 5 composable nodes with KEEP/REMOVE annotations | File 4 before/after description |
| 5 | Task 2.3 acceptance criteria missing converter preservation requirement | Major/Specificity | Fixed: added "ImageFormatConverterNodes preserved" to criteria | Task 2.3 |
| 6 | VL822 USB 2.0 companion (2109:2822) not in udev rules | Minor/Correctness | No action needed: USB 2.0 device has no LPM attributes; autosuspend handled by global kernel param | Documented only |
| 7 | start_robot.sh line references off (63-67 → 65-67) | Minor/Specificity | Fixed: corrected line numbers | File 5 before reference |

## Holistic Review

### Decision Interactions

All five decisions form a coherent, layered strategy with no conflicts:

1. **Conservative FPS (D1) + AUTO speed (D3):** Complementary. `AUTO` adapts to whatever bus speed is available, and conservative FPS (~60 MB/s) is well within both USB 3.0 (400 MB/s) and USB 2.0 (30 MB/s with MJPEG compression). If LPM fix fails and bus stays at USB 2.0, the camera won't crash — it'll just deliver lower actual FPS.

2. **Belt-and-suspenders persistence (D2) + Two-phase deployment (D5):** Both are about risk management. Kernel params + udev rules provide defense-in-depth at different USB stack layers. Two-phase deployment ensures the transport layer is verified before application config is changed. Phase 1 validates both persistence mechanisms together (single reboot).

3. **Remove resize nodes (D4) + Conservative FPS (D1):** Slight tension — removing resize increases VSLAM processing load (720P vs 480P), but 30fps delivery over USB 3.0 is smoother and may help CUDA memory operations that failed under USB 2.0 jitter. The explicit rollback path (step 2.10) mitigates the risk.

4. **AUTO speed (D3) + Remove resize (D4):** If `AUTO` falls back to USB 2.0, stereo will deliver ~10fps at 720P. VSLAM may struggle with high jitter at 720P without resize nodes. However, this scenario only occurs if the LPM fix fails entirely (low probability given direct root cause match), and the `image_jitter_threshold_ms=60ms` would need adjustment. Acceptable risk given the rollback path.

### Architectural Considerations

- **No code changes to voice_mapper.py** — All changes are configuration (YAML, launch file, system config, comments). This minimizes regression risk to the main application.
- **Reversibility** — Every change has a documented rollback: git revert for repo files, backup for extlinux.conf, file deletion for udev rules, physical cable move.
- **Phase 1 is a gate** — If USB 3.0 doesn't achieve stable SuperSpeed after Phase 1, Phase 2 is blocked. This prevents deploying 30fps config on a USB 2.0 bus.

### Trade-offs Accepted

1. **720P VSLAM may fail at native resolution** — The original `cudaErrorInvalidPitchValue` was caused by mono8→rgb8 format mismatch (fixed by ImageFormatConverterNodes, which are preserved). Residual risk: CUDA pitch alignment issue specific to 1280×720 dimensions. Risk accepted because: higher resolution VSLAM is significantly better for feature matching, and rollback is trivial (step 2.10).
2. **Global autosuspend disable** — `usbcore.autosuspend=-1` affects ALL USB devices, not just the camera. Accepted because the robot has no power-constrained USB devices — all peripherals run continuously.
3. **Conservative FPS leaves headroom** — 60 MB/s uses only 15% of USB 3.0 capacity. This is intentional for the first USB 3.0 plan; a follow-up can push higher if stability is confirmed.
4. **`image_jitter_threshold_ms=60` is an estimate** — Actual USB 3.0 jitter may be lower or higher than expected. May need tuning after live measurement.

### Risk Summary

**Highest risk:** The CUDA pitch error persisting at 720P (R2) — this is the only change that might need rollback during Phase 2. All Phase 1 changes have high confidence based on confirmed root cause analysis from Research 004.

**Lowest risk:** Phase 1 (USB transport fix) — directly addresses confirmed U1/U2 LPM root cause with standard Linux USB power management controls. 50+ NVIDIA forum threads report the same issue; LPM disable is the most commonly recommended fix.

## Overview

### Problem Statement

The OAK-D Pro camera on the ROSMASTER A1 robot operates at USB 2.0 (480 Mbps, 5-10 fps) instead of USB 3.0 (5 Gbps, 30 fps) due to two compounding issues:

1. **Physical port limitation:** The camera is currently connected to the USB-C port, which uses a FUSB301 Type-C CC controller that lacks SuperSpeed lane muxing — it physically cannot provide USB 3.0 regardless of software configuration.

2. **U1/U2 LPM protocol failure:** When connected to Type-A ports (which route through VIA Labs VL822 hub with native USB 3.0 support), the camera successfully negotiates SuperSpeed but falls back to USB 2.0 within seconds due to Link Power Management handshake failures between the Intel Movidius MyriadX VPU, VL822 hub firmware, and tegra-xusb host controller. The VL822 hub has `usb3_hardware_lpm_u1=enabled` and `usb3_hardware_lpm_u2=enabled` by default with zero USB kernel boot parameters or udev power management rules configured.

This limits VSLAM to ~5-8 Hz jittery stereo input (instead of smooth 30 fps), LLM vision to 5 fps MJPEG, and prevents the robot from leveraging the OAK-D Pro's full sensor capabilities.

### Objectives

1. Achieve stable USB 3.0 SuperSpeed (5 Gbps) for the OAK-D Pro camera with zero fallbacks to USB 2.0
2. Persist the USB stability fix across reboots via kernel boot parameters and udev rules
3. Increase stereo L/R FPS to 30 fps for smooth VSLAM tracking
4. Increase RGB FPS to 15 fps MJPEG for better LLM vision temporal coverage
5. Test VSLAM at native 1280×720 resolution (removing the 640×480 resize workaround)
6. Update VSLAM jitter threshold to match USB 3.0's smoother frame delivery

### Scope

- **In scope:**
  - Physical camera port migration (USB-C → Type-A)
  - Udev rules for VIA Labs hub LPM disable (`/etc/udev/rules.d/90-usb3-camera-stability.rules`)
  - Kernel boot parameters in `/boot/extlinux/extlinux.conf`
  - `scripts/oakd_params.yaml` — USB speed and FPS updates
  - `scripts/oakd_vslam.launch.py` — remove resize nodes (keep format converters), update jitter threshold
  - `scripts/start_robot.sh` — update comments for USB 3.0
  - Verification of USB 3.0 link stability, camera FPS, and VSLAM tracking

- **Out of scope:**
  - Camera resolution changes beyond FPS (stays 720P stereo, 1080P RGB)
  - Nav2 costmap tuning with higher-FPS depth data
  - IMU noise characterization (Allan variance)
  - External powered USB 3.0 hub (fallback only if software fix fails)
  - OAK-D Pro PoE migration
  - Hybrid SLAM mode tuning

## Requirements

### Functional

| ID | Requirement | Acceptance Criteria |
|----|-------------|---------------------|
| F1 | OAK-D Pro maintains USB 3.0 SuperSpeed link | `lsusb -t` shows `5000M` for Movidius device; no `480M` fallback after 10+ minutes |
| F2 | No U1/U2 LPM failures in kernel log | `dmesg | grep -i "U1 failed\|U2 failed"` returns zero matches after 10+ minutes |
| F3 | USB LPM fix persists across reboot | After `sudo reboot`, F1 and F2 still pass without manual intervention |
| F4 | Camera stereo streams at 30 fps | `ros2 topic hz /oak/left/image_rect --window 10` reports ≥ 25 Hz average |
| F5 | Camera RGB stream at 15 fps | `ros2 topic hz /oak/rgb/image_raw --window 10` reports ≥ 12 Hz average |
| F6 | Camera auto-detects USB speed | `oakd_params.yaml` uses `i_usb_speed: "AUTO"` — works on both USB 3.0 and 2.0 buses |
| F7 | VSLAM receives native 720P stereo | `/oak/left/image_rect` publishes at 1280×720; no resize node in pipeline |
| F8 | VSLAM tracking functional | `/visual_slam/tracking/odometry` topic publishes at ≥ 10 Hz |

### Non-Functional

| ID | Requirement | Acceptance Criteria |
|----|-------------|---------------------|
| NF1 | USB stability over extended run | No camera disconnects (dmesg USB disconnect events) for ≥ 30 minutes |
| NF2 | Total USB bandwidth ≤ 200 MB/s | Well within VL822 5 Gbps (~400 MB/s effective) single-hub capacity |
| NF3 | Changes are reversible | All system changes can be undone by removing udev file + reverting extlinux.conf from backup |
| NF4 | No impact on other USB devices | LiDAR (`/scan`), audio (PulseAudio), serial (CH34x) continue functioning normally |
| NF5 | Service auto-start still works | `voice_mapper.service` starts correctly after reboot with new USB config |

### Out of Scope

- Nav2 parameter tuning for higher-FPS depth input
- Depth camera point cloud (`/oak/points`) setup
- IMU noise characterization (Allan variance for BMI270)
- Multi-camera USB 3.0 support
- USB-C host mode investigation (FUSB301 SuperSpeed lane muxing)
- JetPack version upgrade

## Technical Design

### Architecture Overview

The fix operates at three layers of the USB stack:

```
Layer 3 — Application Config (Phase 2)
  oakd_params.yaml:  i_usb_speed AUTO, stereo 30fps, RGB 15fps
  oakd_vslam.launch.py:  remove ResizeNode (keep mono8→rgb8 converters), feed 720P to VSLAM via converters
  start_robot.sh:  updated comments

Layer 2 — OS Persistence (Phase 1)
  /etc/udev/rules.d/90-usb3-camera-stability.rules
    → Disables hardware LPM U1/U2 on VL822 hub (0822)
    → Disables autosuspend on Movidius device
  /boot/extlinux/extlinux.conf APPEND line
    → usbcore.autosuspend=-1 (global suspend disable)
    → usbcore.usbfs_memory_mb=1000 (camera streaming buffer)
    → usbcore.quirks=03e7:2485:n,2109:0822:n (NO_LPM for Movidius + VL822)

Layer 1 — Physical (Phase 1)
  Move OAK-D Pro USB cable: USB-C port → any Type-A port
  USB-C lacks SuperSpeed lane muxing (FUSB301 limitation)
  Type-A routes through VL822 hub with native USB 3.0 support
```

### Codebase Patterns

```yaml
codebase_patterns:
  - pattern: YAML params_file for camera config
    location: "scripts/oakd_params.yaml"
    usage: All camera parameters delivered via params_file — CLI overrides are silently ignored by camera.launch.py
  - pattern: Standalone launch files for composable nodes
    location: "scripts/oakd_vslam.launch.py"
    usage: VSLAM launch via ComposableNodeContainer with all params/remappings inline
  - pattern: Shell script hardware launcher
    location: "scripts/start_robot.sh"
    usage: Sequential hardware bringup with background PIDs and cleanup trap
  - pattern: SCP deployment to robot
    location: "Development Workflow in copilot-instructions.md"
    usage: "scp scripts/* jetson@192.168.7.250:~/robot_scripts/"
  - pattern: SSH commands for system config
    location: "Prior plans (002, 003)"
    usage: SSH one-liners for /etc/ and /boot/ file modifications on robot
```

### Data Contracts

No data entities in scope — data contracts not applicable.

### File Changes Summary

| # | File | Location | Change Type | Description |
|---|------|----------|-------------|-------------|
| 1 | `90-usb3-camera-stability.rules` | Robot: `/etc/udev/rules.d/` | **New file (SSH)** | Udev rules to disable USB3 hardware LPM on VL822 hub and autosuspend on Movidius |
| 2 | `extlinux.conf` | Robot: `/boot/extlinux/` | **Modify (SSH)** | Add USB kernel boot parameters to APPEND line |
| 3 | `oakd_params.yaml` | Repo: `scripts/` | **Modify** | Change `i_usb_speed` to AUTO, increase stereo to 30fps, RGB to 15fps |
| 4 | `oakd_vslam.launch.py` | Repo: `scripts/` | **Modify** | Remove ResizeNode composable nodes (keep ImageFormatConverterNodes), update VSLAM remappings to converter outputs, lower jitter threshold |
| 5 | `start_robot.sh` | Repo: `scripts/` | **Modify** | Update comments to reflect USB 3.0 operation |

### File 1: `/etc/udev/rules.d/90-usb3-camera-stability.rules` (New — SSH)

**Full file content:**

```udev
# 90-usb3-camera-stability.rules
# Fix USB 3.0 SuperSpeed fallback caused by U1/U2 LPM handshake failures
# between Intel Movidius MyriadX, VIA Labs VL822 hub, and tegra-xusb.
# See: docs/research/004-orin-nano-super-usb3-cameras.md

# VIA Labs VL822 USB 3.1 Gen 2 Hub (P3768 carrier board, Tier 1)
# Disable hardware U1/U2 LPM and autosuspend
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2109", ATTR{idProduct}=="0822", \
  RUN+="/bin/sh -c 'echo on > /sys%p/power/control; echo -1 > /sys%p/power/autosuspend_delay_ms; echo disabled > /sys%p/power/usb3_hardware_lpm_u1 2>/dev/null; echo disabled > /sys%p/power/usb3_hardware_lpm_u2 2>/dev/null'"

# Intel Movidius MyriadX (OAK-D cameras) — disable autosuspend, keep powered
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="03e7", \
  RUN+="/bin/sh -c 'echo on > /sys%p/power/control; echo -1 > /sys%p/power/autosuspend_delay_ms'"
```

### File 2: `/boot/extlinux/extlinux.conf` (Modify — SSH)

**Change:** Append three parameters to the existing `APPEND` line.

**Before (current line, truncated):**
```
      APPEND ${cbootargs} root=PARTUUID=... rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 ...
```

**After:**
```
      APPEND ${cbootargs} root=PARTUUID=... rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 ... usbcore.autosuspend=-1 usbcore.usbfs_memory_mb=1000 usbcore.quirks=03e7:2485:n,2109:0822:n
```

**Parameters explained:**
- `usbcore.autosuspend=-1` — Globally disable USB autosuspend (prevents any device from entering suspend)
- `usbcore.usbfs_memory_mb=1000` — Increase USBfs buffer from 16 MB to 1000 MB (required for USB 3.0 camera streaming)
- `usbcore.quirks=03e7:2485:n,2109:0822:n` — Set USB_QUIRK_NO_LPM (`n` flag) for Movidius MyriadX and VL822 hub, preventing LPM capability advertisement at enumeration. Note: No VL817 present on P3768 carrier board; VL822's USB 2.0 companion (2109:2822) inherits autosuspend=-1 globally.

**Safety:** Backup `extlinux.conf` before modification.

### File 3: `scripts/oakd_params.yaml` (Modify — Repo)

**Before:**
```yaml
    camera:
      i_pipeline_type: RGBD
      i_usb_speed: "HIGH"            # USB 2.0 (480 Mbps) — matches physical bus
      i_enable_imu: true
      i_enable_ir: false
      i_nn_type: none
    rgb:
      i_resolution: "1080"
      i_fps: 5.0                     # 5fps — low to save USB bandwidth for stereo
      i_low_bandwidth: true
      i_low_bandwidth_quality: 50
    left:
      i_resolution: "720"
      i_fps: 15.0                    # 15fps — actual ~10Hz on single-hub USB 2.0
      i_low_bandwidth: false
    right:
      i_resolution: "720"
      i_fps: 15.0
      i_low_bandwidth: false
```

**After:**
```yaml
    camera:
      i_pipeline_type: RGBD
      i_usb_speed: "AUTO"            # Auto-detect: USB 3.0 (5 Gbps) or USB 2.0 fallback
      i_enable_imu: true
      i_enable_ir: false
      i_nn_type: none
    rgb:
      i_resolution: "1080"
      i_fps: 15.0                    # 15fps — USB 3.0 has plenty of bandwidth
      i_low_bandwidth: true
      i_low_bandwidth_quality: 50
    left:
      i_resolution: "720"
      i_fps: 30.0                    # 30fps — full OV9282 rate over USB 3.0
      i_low_bandwidth: false
    right:
      i_resolution: "720"
      i_fps: 30.0
      i_low_bandwidth: false
```

**Bandwidth estimate (USB 3.0):**
- Stereo L/R 720P@30fps raw: 2 × 1280×720×1×30 = ~52.7 MB/s
- RGB 1080P@15fps MJPEG (quality 50): ~3-5 MB/s
- Depth 720P@30fps raw: 1280×720×2×30 = ~52.7 MB/s (if published)
- IMU: negligible
- **Total: ~60 MB/s** — 15% of VL822's ~400 MB/s effective bandwidth

### File 4: `scripts/oakd_vslam.launch.py` (Modify — Repo)

**Before (lines 14-129):** Contains 5 ComposableNodes in the container:
1. `convert_left` — ImageFormatConverterNode: mono8 → rgb8 (output: `/vslam/left/image_rgb`) **KEEP**
2. `convert_right` — ImageFormatConverterNode: mono8 → rgb8 (output: `/vslam/right/image_rgb`) **KEEP**
3. `resize_left` — ResizeNode: 1280×720 → 640×480 (output: `/vslam/left/image_resized`) **REMOVE**
4. `resize_right` — ResizeNode: 1280×720 → 640×480 (output: `/vslam/right/image_resized`) **REMOVE**
5. `visual_slam_node` — reads from `/vslam/left/image_resized` topics **UPDATE REMAPPINGS**

⚠️ The ImageFormatConverterNodes (1-2) are **required** — OAK-D Pro OV9282 publishes mono8 but Isaac VSLAM NitrosSubscriber negotiates rgb8. Without format conversion, VSLAM fails with `cudaMemcpy2D pitch mismatch`.

**After:** Remove both ResizeNode composable nodes (3-4). Keep both ImageFormatConverterNodes (1-2). Update VSLAM node remappings to read from converter outputs (`/vslam/left/image_rgb`) instead of resize outputs. Lower `image_jitter_threshold_ms` from 200ms to 60ms (USB 3.0 delivers ~33ms frame intervals at 30fps with low jitter).

**Full replacement `generate_launch_description()` function:**
```python
def generate_launch_description():
    # Convert left mono8 → rgb8 (required: VSLAM NitrosSubscriber expects rgb8)
    convert_left = ComposableNode(
        name='convert_left',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'encoding_desired': 'rgb8',
            'image_width': 1280,
            'image_height': 720,
        }],
        remappings=[
            ('image_raw', '/oak/left/image_rect'),
            ('image', '/vslam/left/image_rgb'),
        ],
    )
    # Convert right mono8 → rgb8
    convert_right = ComposableNode(
        name='convert_right',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'encoding_desired': 'rgb8',
            'image_width': 1280,
            'image_height': 720,
        }],
        remappings=[
            ('image_raw', '/oak/right/image_rect'),
            ('image', '/vslam/right/image_rgb'),
        ],
    )
    # Isaac VSLAM node — reads rgb8-converted 720P stereo (no resize)
    # USB 3.0 delivers 1280x720@30fps with low jitter (~33ms intervals)
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'num_cameras': 2,
            'enable_image_denoising': False,
            'rectified_images': True,
            'enable_imu_fusion': True,
            'enable_localization_n_mapping': True,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'base_frame': 'base_link',
            'imu_frame': 'oak_imu_frame',
            'camera_optical_frames': [
                'oak_left_camera_optical_frame',
                'oak_right_camera_optical_frame',
            ],
            # IMU noise params — RealSense reference values as starting point
            # TODO: Characterize OAK-D Pro BMI270 via Allan variance
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 60.0,  # USB 3.0: ~33ms intervals at 30fps, low jitter
        }],
        remappings=[
            ('visual_slam/image_0', '/vslam/left/image_rgb'),
            ('visual_slam/camera_info_0', '/oak/left/camera_info'),
            ('visual_slam/image_1', '/vslam/right/image_rgb'),
            ('visual_slam/camera_info_1', '/oak/right/camera_info'),
            ('visual_slam/imu', '/oak/imu/data'),
        ],
    )
    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            convert_left,
            convert_right,
            visual_slam_node,
        ],
        output='screen',
    )
    return launch.LaunchDescription([visual_slam_launch_container])
```

**Rollback plan:** If native 720P causes any issues, revert to the resize-node version from git history. The original `cudaErrorInvalidPitchValue` was caused by mono8→rgb8 format mismatch (fixed by ImageFormatConverterNodes, which are preserved). If the error recurs at 720P despite format conversion, it indicates a CUDA pitch alignment issue specific to 1280×720 dimensions — in that case, re-add resize nodes to downscale to 640×480.

### File 5: `scripts/start_robot.sh` (Modify — Repo)

**Before (lines 65-67):**
```bash
#     oakd_params.yaml sets: depth pipeline, USB 2.0 HIGH speed, stereo
#     720P@10fps (raw), synced rect pair, IR off, no NN.
#     RGB disabled to conserve USB 2.0 bandwidth (~17.6 MB/s stereo pair).
```

**After:**
```bash
#     oakd_params.yaml sets: depth pipeline, USB AUTO speed, stereo
#     720P@30fps (raw), RGB 1080P@15fps MJPEG, synced rect pair, IR off, no NN.
#     USB 3.0 target: ~60 MB/s total (15% of VL822 5Gbps hub capacity).
#     Falls back to USB 2.0 gracefully if USB 3.0 link is unavailable.
```

## Dependencies

| # | Dependency | Type | Status | Notes |
|---|-----------|------|--------|-------|
| D1 | Physical access to robot | Physical | Required | Must move USB cable from USB-C to Type-A port |
| D2 | SSH access to robot | Network | ✅ Available | `ssh jetson@192.168.7.250`, password `yahboom` |
| D3 | sudo access on robot | Permission | ✅ Available | `jetson` user has sudo for udev and extlinux |
| D4 | Plan 003 complete | Code | ✅ Complete | `oakd_params.yaml` and `oakd_vslam.launch.py` exist with correct base structure |
| D5 | DepthAI ROS2 driver installed | Software | ✅ v2.12.2 | `depthai_ros_driver` on robot |
| D6 | Isaac VSLAM installed | Software | ✅ v3.2.6 | `ros-humble-isaac-ros-visual-slam` on robot |
| D7 | Git history for rollback | Code | ✅ Available | Current `oakd_vslam.launch.py` with resize nodes preserved in git |

## Risks

| # | Risk | Likelihood | Impact | Mitigation |
|---|------|-----------|--------|------------|
| R1 | LPM disable does not fix SuperSpeed fallback — tegra-xusb driver itself causes failures independent of LPM | Low | High | If USB 3.0 fails after LPM disable: try external powered USB 3.0 hub (TI TUSB8041-based) as bypass. Document in verification. |
| R2 | Native 720P causes issues — the original `cudaErrorInvalidPitchValue` was a mono8/rgb8 format mismatch (fixed by ImageFormatConverterNodes, which are preserved). Remaining risk is a CUDA pitch alignment issue specific to 1280×720 dimensions. | Low | Medium | Re-add resize nodes (640×480) from git history. VSLAM still works at lower resolution with converters. |
| R3 | Camera disconnects increase after removing autosuspend | Very Low | Medium | Autosuspend removal should reduce disconnects, not increase. If unexpected behavior: revert `extlinux.conf` from backup, remove udev rules. |
| R4 | Other USB devices affected by global `autosuspend=-1` | Very Low | Low | All robot USB devices (LiDAR, audio, serial) are always-on anyway. No power-sensitive USB devices on this platform. Verify in Phase 1. |
| R5 | Robot IP changes after reboot (DHCP) | Low | Low | Re-discover via `nmap -sn 192.168.7.0/24` or check router DHCP leases. |
| R6 | `extlinux.conf` edit breaks boot | Very Low | Critical | Always create backup before modification. Recovery: connect monitor + keyboard, edit from GRUB/recovery. |
| R7 | 30fps stereo + 15fps RGB exceeds USB 2.0 fallback bandwidth | Medium (only if LPM fix fails) | Medium | `i_usb_speed: AUTO` gracefully adapts pipeline to USB 2.0 frame rates. Camera will reduce FPS automatically rather than crash. |

## Execution Plan

### Phase 1: USB 3.0 Transport Fix

**Status:** ⏳ Not Started
**Size:** Small
**Files to Modify:** 2 (robot system files via SSH)
**Prerequisites:** Physical access to robot, SSH access confirmed
**Entry Point:** SSH to `jetson@192.168.7.250`
**Verification:** `lsusb -t` shows Movidius at `5000M`; `dmesg` has no U1/U2 failures for 10+ minutes

| Step | Task | Files | Acceptance Criteria |
|------|------|-------|---------------------|
| 1.1 | **Stop voice_mapper service** — `sudo systemctl stop voice_mapper.service` to release camera | None | Service stopped; camera USB idle |
| 1.2 | **Physical cable move** — Unplug OAK-D Pro from USB-C port, plug into any Type-A port on Jetson carrier board | Hardware only | `lsusb -t` shows Movidius under VL822 hub on Bus 2 (path `2-1.x`) at `5000M` |
| 1.3 | **Create udev rules file** — Write `/etc/udev/rules.d/90-usb3-camera-stability.rules` with LPM disable rules for VL822 (2109:0822) and autosuspend disable for Movidius (03e7:*). Reload with `sudo udevadm control --reload-rules`. | `/etc/udev/rules.d/90-usb3-camera-stability.rules` (new) | File exists; `udevadm control --reload-rules` succeeds |
| 1.4 | **Backup extlinux.conf** — `sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.bak.$(date +%Y%m%d)` | `/boot/extlinux/extlinux.conf` | Backup file exists |
| 1.5 | **Add kernel boot parameters** — Append `usbcore.autosuspend=-1 usbcore.usbfs_memory_mb=1000 usbcore.quirks=03e7:2485:n,2109:0822:n` to the APPEND line in `/boot/extlinux/extlinux.conf` | `/boot/extlinux/extlinux.conf` | `grep APPEND` shows all three parameters at end of line |
| 1.6 | **Reboot robot** — `sudo reboot` and wait ~60s for boot | None | SSH reconnects after ~60s |
| 1.7 | **Verify USB 3.0 SuperSpeed** — Check `lsusb -t` for Movidius at `5000M`. Check sysfs: `cat /sys/bus/usb/devices/2-1/power/usb3_hardware_lpm_u1` → `disabled`. Check `cat /sys/module/usbcore/parameters/autosuspend` → `-1`. Check `cat /sys/module/usbcore/parameters/usbfs_memory_mb` → `1000`. | None (read-only checks) | Movidius at 5000M; LPM disabled on VL822 hub; kernel params active |
| 1.8 | **Verify no USB 2.0 fallback** — Monitor `dmesg -w` for 5+ minutes. Search for `"U1 failed"`, `"U2 failed"`, `"USB disconnect"` on Movidius device path. | None (read-only checks) | Zero U1/U2 failure messages; zero Movidius disconnect events |
| 1.9 | **Verify other USB devices** — Check LiDAR (`/scan` topic hz), audio (PulseAudio device list), serial (CH34x present in `lsusb`) | None (read-only checks) | LiDAR ~10Hz, audio device present, serial devices present |
| 1.10 | **Start service** — `sudo systemctl start voice_mapper.service` to confirm service starts with new USB config | None | Service running; "Mapper ready" in journal logs |

**Rollback procedure:**
```bash
# Revert extlinux.conf
sudo cp /boot/extlinux/extlinux.conf.bak.YYYYMMDD /boot/extlinux/extlinux.conf
# Remove udev rules
sudo rm /etc/udev/rules.d/90-usb3-camera-stability.rules
sudo udevadm control --reload-rules
# Move cable back to USB-C (if needed)
sudo reboot
```

### Phase 2: Camera & VSLAM Configuration Update

**Status:** ⏳ Not Started
**Size:** Small
**Files to Modify:** 3 (repo files deployed via SCP)
**Prerequisites:** Phase 1 complete — USB 3.0 SuperSpeed confirmed stable
**Entry Point:** `scripts/oakd_params.yaml` in local repo
**Verification:** Stereo at ≥25 Hz, RGB at ≥12 Hz, VSLAM odometry publishing

| Step | Task | Files | Acceptance Criteria |
|------|------|-------|---------------------|
| 2.1 | **Stop voice_mapper service** — `sudo systemctl stop voice_mapper.service` | None | Service stopped |
| 2.2 | **Update oakd_params.yaml** — Change `i_usb_speed` from `HIGH` to `AUTO`. Change stereo L/R `i_fps` from 15.0 to 30.0. Change RGB `i_fps` from 5.0 to 15.0. Update header comments to reflect USB 3.0 bandwidth budget. | `scripts/oakd_params.yaml` | File updated locally; `i_usb_speed: "AUTO"`, stereo fps 30.0, RGB fps 15.0 |
| 2.3 | **Update oakd_vslam.launch.py** — Remove `resize_left` and `resize_right` ComposableNode blocks. **Keep** `convert_left` and `convert_right` ImageFormatConverterNodes (mono8→rgb8 required by VSLAM). Update VSLAM remappings from `/vslam/left/image_resized` → `/vslam/left/image_rgb` (converter output, not raw topic). Update camera_info remappings from `/vslam/left/camera_info_resized` → `/oak/left/camera_info`. Remove ResizeNode from `composable_node_descriptions` list. Change `image_jitter_threshold_ms` from 200.0 to 60.0. Update header comment to note USB 3.0 native 720P. | `scripts/oakd_vslam.launch.py` | File updated locally; no ResizeNode references; **ImageFormatConverterNodes preserved**; VSLAM reads `/vslam/left/image_rgb` (rgb8 converted); jitter threshold 60ms |
| 2.4 | **Update start_robot.sh comments** — Update camera section comments (lines 63-67) to reflect USB 3.0 AUTO speed, 30fps stereo, 15fps RGB, ~60 MB/s bandwidth. | `scripts/start_robot.sh` | Comments accurate for USB 3.0 config |
| 2.5 | **Deploy to robot** — `scp scripts/oakd_params.yaml scripts/oakd_vslam.launch.py scripts/start_robot.sh jetson@192.168.7.250:~/robot_scripts/` | None (SCP transfer) | Files on robot match local repo |
| 2.6 | **Start service** — `sudo systemctl start voice_mapper.service` | None | Service running |
| 2.7 | **Verify stereo FPS** — `ros2 topic hz /oak/left/image_rect --window 10` | None (read-only) | ≥ 25 Hz average (target 30 Hz) |
| 2.8 | **Verify RGB FPS** — `ros2 topic hz /oak/rgb/image_raw --window 10` | None (read-only) | ≥ 12 Hz average (target 15 Hz) |
| 2.9 | **Verify VSLAM native 720P** — Start VSLAM via voice command ("Start visual SLAM"). Check `/visual_slam/tracking/odometry` publishes at ≥ 10 Hz. Check dmesg/logs for `cudaErrorInvalidPitchValue` — if present, CUDA pitch issue confirmed as resolution-dependent. | None (read-only) | VSLAM odometry ≥ 10 Hz OR clear CUDA error for rollback |
| 2.10 | **Rollback if CUDA error** — If step 2.9 shows `cudaErrorInvalidPitchValue`: revert `oakd_vslam.launch.py` to the resize-node version from git, re-deploy, re-test VSLAM. Document that 720P native requires a different fix. | `scripts/oakd_vslam.launch.py` (git revert) | VSLAM works at 640×480 via resize nodes |
| 2.11 | **Extended stability test** — Leave robot running for 30+ minutes. Verify no camera disconnects in dmesg, stable topic hz on stereo and RGB, VSLAM tracking maintained. | None (read-only) | Zero disconnects; stable FPS; VSLAM odometry continuous |

**Rollback procedure:**
```bash
# Revert repo files to pre-plan state
git checkout HEAD -- scripts/oakd_params.yaml scripts/oakd_vslam.launch.py scripts/start_robot.sh
# Re-deploy originals
scp scripts/oakd_params.yaml scripts/oakd_vslam.launch.py scripts/start_robot.sh jetson@192.168.7.250:~/robot_scripts/
# Restart service
ssh jetson@192.168.7.250 'sudo systemctl restart voice_mapper.service'
```

### Phase 3: Documentation Update

**Status:** ⏳ Not Started
**Size:** Small
**Files to Modify:** 1
**Prerequisites:** Phase 1 and Phase 2 complete with verified results
**Entry Point:** `PROGRESS.md`
**Verification:** Documentation matches actual system state

| Step | Task | Files | Acceptance Criteria |
|------|------|-------|---------------------|
| 3.1 | **Update PROGRESS.md** — Record USB 3.0 fix as completed. Update Hardware Status table (camera speed). Move "USB 3.0 Port Migration" from In Progress to Completed. Update Next Steps. Add session notes. | `PROGRESS.md` | All sections reflect actual USB 3.0 status and FPS |

## Standards

⚠️ Could not access organizational standards from pch-standards-space. Proceeding without standards context.

No organizational standards applicable to this plan.

## Handoff

| Field | Value |
|-------|-------|
| Created By | pch-planner |
| Created Date | 2026-02-24 |
| Status | 🔍 In Review |
| Reviewed By | pch-plan-reviewer |
| Review Date | 2026-02-24 |
| Next Agent | pch-plan-reviewer (review in progress) |
| Plan Location | /docs/plans/004-usb3-lpm-fix-oakd-pro.md |
