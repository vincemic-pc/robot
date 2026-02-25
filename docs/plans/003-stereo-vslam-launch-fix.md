---
id: "003"
type: implementation-plan
title: "Stereo Stream Publishing & Isaac VSLAM Launch Fix"
status: "\u2705 Complete"
created: "2026-02-24"
updated: "2026-02-24"
owner: pch-planner
version: v2.2
---

## Version History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| v1.0 | 2026-02-24 | pch-planner | Initial plan skeleton |
| v1.1 | 2026-02-24 | pch-planner | Added file placement decision |
| v1.2 | 2026-02-24 | pch-planner | Added VSLAM launch strategy, IR default, comment style decisions |
| v2.0 | 2026-02-24 | pch-planner | Complete plan: requirements, technical design, execution plan, holistic review |
| v2.1 | 2026-02-24 | pch-plan-reviewer | Review: consolidated voice_mapper.py Before/After, added camera-type guard, fixed frontmatter status |
| v2.2 | 2026-02-24 | pch-coder | Implementation complete: Phase 1 & 2 done, Phase 3 deployed + syntax validated |

## Review Session Log

**Questions Pending:** 0  
**Questions Resolved:** 0  
**Last Updated:** 2026-02-24

No clarifying questions were needed — all issues were Minor with clear resolutions.

| # | Issue | Category | Resolution | Plan Update |
|---|-------|----------|------------|-------------|
| 1 | Frontmatter `status: complete` contradicts Handoff `Pending Review` | correctness | Fixed to `ready-for-implementation` | Frontmatter + Handoff updated |
| 2 | File 4 Before/After only showed `cmd` block, not full removal scope | specificity | Consolidated Before/After covering lines 1424-1467 | Technical Design File 4 updated |
| 3 | RealSense branch removal not noted as intentional scope reduction | completeness | Added camera-type guard + note in After block | Technical Design File 4 + Step 2.2 updated |
| 4 | `remappings = []` init would become dead code if left in place | specificity | Included in consolidated Before block for removal | Technical Design File 4 updated |

## Introduction

This plan implements the fixes identified in [Research 003](../research/003-stereo-streams-vslam-enablement.md) to resolve 9 bugs preventing stereo rectified image publishing and Isaac VSLAM operation on the ROSMASTER A1 robot. The core issues are: (1) DepthAI ROS driver launch parameters passed via CLI `:=` syntax are silently ignored because `camera.launch.py` never declares them as launch arguments, and (2) the same pattern affects Isaac VSLAM's bare launcher. The fix involves creating two new configuration files and updating two existing scripts.

## Planning Session Log

| # | Decision Point | Answer | Rationale |
|---|----------------|--------|-----------|
| 1 | File placement for new config files | A — In `scripts/` alongside existing scripts | Matches existing flat structure; zero deployment workflow changes needed; `scp` already copies `scripts/` to `/home/jetson/robot_scripts/` |
| 2 | VSLAM launch strategy in voice_mapper.py | A — Simple launch file path, no dynamic params | Eliminates bug class of ignored runtime overrides; single camera config means no runtime variation needed; editing launch file on robot is trivial for tuning |
| 3 | IR dot projector default | A — IR OFF by default (`i_enable_ir: false`) | VSLAM incompatible with IR dots; OAK-D Pro passive stereo sufficient; matches NVIDIA's `isaac_vslam.yaml` reference; comment in YAML explains trade-off |
| 4 | Comment style in start_robot.sh | C — Keep existing multi-line comment style, fix accuracy | Maintains established script documentation pattern; corrects inaccurate claims about USB speed override; helps debugging at the operational entry point |

## Holistic Review

### Decision Interactions

All four decisions are complementary with no conflicts:

1. **File placement (scripts/) + Simple launch (no dynamic params)** — Reinforcing: flat `scripts/` directory with self-contained config files means deployment is a single `scp` command. No dynamic params means no parameter-threading bugs.
2. **IR OFF + Simple launch** — Compatible: IR setting is baked into `oakd_params.yaml`, and VSLAM launch file assumes IR is off. No runtime toggle complexity.
3. **Comment accuracy + params_file mechanism** — The corrected comments in `start_robot.sh` document the root cause (CLI overrides ignored) which is the same bug motivating the YAML approach.

### Architectural Considerations

- **Single point of configuration** — Each concern has exactly one config file. Camera: `oakd_params.yaml`. VSLAM: `oakd_vslam.launch.py`. No parameter values are split across multiple files.
- **Deployment atomicity** — All 4 files deploy together via `scp scripts/*`. There's no partial-deployment risk where old script references new config that doesn't exist yet — the files are co-located.
- **Backward compatibility** — If VSLAM is never started, the camera params still work correctly (stereo rect publishes but nobody subscribes). The `voice_mapper.py` VSLAM lifecycle code (`stop_isaac_vslam`, status checks) is unchanged.
- **USB 2.0 bandwidth is tight** — The ~31.5 MB/s target slightly exceeds the conservative ~30 MB/s estimate. This is the primary risk. Mitigation is to lower stereo fps or increase MJPEG compression if bandwidth is exceeded.

### Trade-offs Accepted

1. **IMU noise parameters are approximate** — Using RealSense reference values for OAK-D Pro BMI270. Functional but not optimal. Allan variance characterization deferred.
2. **No runtime parameter flexibility** — Baked-in config means editing files on disk to tune. Accepted because the alternative (runtime overrides) is the exact bug pattern being fixed.
3. **IR projector always off** — Depth quality may decrease in very low-light scenes without structured light. Accepted because VSLAM compatibility is the primary goal.
4. **MJPEG RGB at quality 50** — Some visual quality loss for LLM vision. Accepted for bandwidth savings. Quality adjustable in YAML if needed.

### Risk Summary

The highest-risk item is whether USB 2.0 bandwidth actually sustains 400P@30fps stereo. This requires live testing (Phase 3) and the `image_jitter_threshold_ms` parameter in the VSLAM launch file provides a tuning knob if fps is lower than expected.

## Overview

### Problem Statement

The OAK-D Pro stereo rectified topics (`/oak/left/image_rect`, `/oak/right/image_rect`) are advertised but publish zero data. Isaac VSLAM cannot function without these streams. Additionally, all ROS2 launch CLI parameter overrides in both `start_robot.sh` and `voice_mapper.py` are silently ignored, including the critical `camera.i_usb_speed:=HIGH` (defaulting to SUPER, causing SIGABRT crashes on USB 2.0 hardware).

### Objectives

1. Enable stereo rectified image publishing from DepthAI driver via correct YAML params
2. Fix USB speed parameter delivery to prevent SIGABRT crashes
3. Fix Isaac VSLAM parameter and remapping delivery via custom launch file
4. Optimize USB 2.0 bandwidth budget for simultaneous RGB + depth + stereo + VSLAM
5. Maintain backward compatibility for non-VSLAM operation

### Scope

- **In scope:** `start_robot.sh`, `voice_mapper.py`, new `oakd_params.yaml`, new `oakd_vslam.launch.py`
- **Out of scope:** USB 3.0 port migration, IMU noise characterization, Nav2 parameter tuning

## Requirements

### Functional

1. **FR-1: Stereo rectified publishing** — `/oak/left/image_rect` and `/oak/right/image_rect` must have active publishers producing GRAY8 (mono8) images when the camera node is running
2. **FR-2: USB speed enforcement** — DepthAI driver must negotiate at USB 2.0 (HIGH/480Mbps) speed, not SUPER, to prevent SIGABRT crashes on the USB 2.0 bus
3. **FR-3: Synchronized stereo pairs** — Left/right rectified images must be hardware-synchronized (matching sequence numbers) via `stereo.i_publish_synced_rect_pair`
4. **FR-4: Isaac VSLAM parameters delivered** — `enable_imu_fusion`, `num_cameras`, `rectified_images`, `enable_localization_n_mapping`, and all other VSLAM parameters must actually reach the ComposableNode (not be silently ignored)
5. **FR-5: Isaac VSLAM topic remappings** — VSLAM node must receive images from `/oak/left/image_rect` and `/oak/right/image_rect` via ComposableNode remappings (not `--ros-args -r`)
6. **FR-6: IMU fusion enabled** — Isaac VSLAM must receive IMU data from `/oak/imu/data` with `enable_imu_fusion: true` to compensate for low visual framerate on USB 2.0
7. **FR-7: RGB stream preserved** — `/oak/rgb/image_raw` must continue publishing for LLM vision system (MJPEG-compressed at 480P@15fps to fit USB 2.0 bandwidth)
8. **FR-8: Depth stream preserved** — `/oak/stereo/image_raw` must continue publishing for Nav2 obstacle avoidance

### Non-Functional

1. **NFR-1: USB 2.0 bandwidth budget** — Total stream bandwidth must not exceed ~30 MB/s (USB 2.0 with double-hub path). Target: ~31.5 MB/s (Config C from research)
2. **NFR-2: VSLAM minimum framerate** — Stereo streams should target 30fps at 400P resolution to meet Isaac VSLAM's 30Hz minimum requirement
3. **NFR-3: No MJPEG on stereo** — Stereo rectified streams must remain uncompressed (raw) — MJPEG artifacts corrupt visual feature detection
4. **NFR-4: IR projector off** — IR dot projector disabled to prevent false VSLAM features
5. **NFR-5: Neural network disabled** — NN processing disabled (`i_nn_type: none`) to save MyriadX resources for streaming
6. **NFR-6: Single-file configuration** — Each concern (camera params, VSLAM launch) in exactly one file — no parameter threading through multiple layers

### Out of Scope

1. Moving OAK-D Pro to USB 3.0 physical port (Bus 02, 10000M)
2. OAK-D Pro BMI270 IMU noise characterization (Allan variance)
3. Nav2 parameter tuning or `nav2_params.yaml` changes
4. `rosmaster_control.sh` modifications
5. `yahboom_explorer.py` updates (deprecated script)
6. Copilot instructions / README updates (separate task)
7. Actual on-robot live testing (separate verification task)

## Technical Design

### Architecture

Four files are affected — two new, two modified:

```
scripts/
├── oakd_params.yaml          ← NEW: DepthAI driver parameters (YAML)
├── oakd_vslam.launch.py      ← NEW: Isaac VSLAM launch (Python)
├── start_robot.sh             ← MODIFY: camera launch command
└── voice_mapper.py            ← MODIFY: start_isaac_vslam() method
```

**Data flow after fix:**

```
start_robot.sh
  └─ ros2 launch camera.launch.py params_file:=oakd_params.yaml
       └─ ComposableNode receives YAML params → stereo rect published

voice_mapper.py → start_isaac_vslam()
  └─ subprocess: ros2 launch oakd_vslam.launch.py
       └─ ComposableNode receives params + remappings → VSLAM tracking
```

### File 1: `scripts/oakd_params.yaml` (NEW)

ROS2 YAML parameter file for the DepthAI driver ComposableNode. Passed via `params_file:=` launch argument — the only mechanism that reaches the node.

```yaml
# oakd_params.yaml — OAK-D Pro config for RGBD + VSLAM on USB 2.0
# Bandwidth budget: ~31.5 MB/s (USB 2.0 with double hub, ~30 MB/s capacity)
#
# Streams:
#   RGB  480P@15fps MJPEG q50  (~0.7 MB/s)
#   Depth 400P@30fps raw       (~15.4 MB/s)
#   Stereo L/R rect 400P@30fps (~15.4 MB/s)
#   IMU                        (~0.01 MB/s)
#
# WARNING: Do NOT use CLI parameter overrides (key:=value) with camera.launch.py.
# They are silently ignored. All parameters must be in this YAML file.
#
/oak:
  ros__parameters:
    camera:
      i_pipeline_type: RGBD
      i_usb_speed: "HIGH"          # USB 2.0 (480 Mbps) — matches physical bus
      i_enable_imu: true            # BMI270 IMU for VSLAM fusion
      i_enable_ir: false            # IR OFF — dots create false VSLAM features
      i_nn_type: none               # Disable NN to save MyriadX resources
    rgb:
      i_resolution: "480P"          # 640x480 — smallest useful for LLM vision
      i_fps: 15.0                   # Reduced fps to save bandwidth for stereo
      i_low_bandwidth: true         # MJPEG compression on-chip
      i_low_bandwidth_quality: 50   # MJPEG quality (10:1 compression)
    left:
      i_resolution: "400"           # 640x400 — matches isaac_vslam.yaml
      i_fps: 30.0                   # 30fps = VSLAM minimum requirement
    right:
      i_resolution: "400"           # Must match left
      i_fps: 30.0                   # Must match left
    stereo:
      i_publish_synced_rect_pair: true    # HW-synced rectified L/R for VSLAM
      i_reverse_stereo_socket_order: true # OAK-D Pro socket order
      i_output_disparity: true            # Keep depth output for Nav2
      i_low_bandwidth: false              # RAW required — MJPEG corrupts VSLAM
```

**Key parameters and why:**

| Parameter | Value | Resolves Bug | Rationale |
|-----------|-------|-------------|-----------|
| `camera.i_usb_speed` | `"HIGH"` | USB speed ignored (Bug #4) | Prevents SUPER → SIGABRT on USB 2.0 bus |
| `stereo.i_publish_synced_rect_pair` | `true` | Stereo rect not published (Bugs #1-2) | Enables HW-synced `/oak/left/image_rect` + `/oak/right/image_rect` |
| `rgb.i_low_bandwidth` | `true` | Bandwidth exhaustion (Bug #5) | MJPEG RGB frees bandwidth for raw stereo |
| `camera.i_enable_ir` | `false` | — | IR dots degrade VSLAM tracking |
| `camera.i_nn_type` | `none` | — | Saves MyriadX resources for streaming |
| `left/right.i_resolution` | `"400"` | — | 400P enables 30fps within USB 2.0 bandwidth |

### File 2: `scripts/oakd_vslam.launch.py` (NEW)

Custom Isaac VSLAM launch file following the vendor RealSense reference pattern. All parameters and remappings are specified directly on the `ComposableNode` — not via launch arguments.

```python
# oakd_vslam.launch.py — Isaac VSLAM for OAK-D Pro stereo camera
# Follows the pattern from isaac_ros_visual_slam_realsense.launch.py
# All params/remappings on ComposableNode — no launch arguments needed.
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
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
            'image_jitter_threshold_ms': 35.0,
        }],
        remappings=[
            ('visual_slam/image_0', '/oak/left/image_rect'),
            ('visual_slam/camera_info_0', '/oak/left/camera_info'),
            ('visual_slam/image_1', '/oak/right/image_rect'),
            ('visual_slam/camera_info_1', '/oak/right/camera_info'),
            ('visual_slam/imu', '/oak/imu/data'),
        ],
    )
    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )
    return launch.LaunchDescription([visual_slam_launch_container])
```

**Key design choices:**

| Element | Value | Resolves Bug | Rationale |
|---------|-------|-------------|-----------|
| `parameters=[{...}]` on ComposableNode | Dict | Params silently ignored (Bugs #6-7) | Only way to deliver params to ComposableNode |
| `remappings=[...]` on ComposableNode | Tuples | `--ros-args -r` may not reach node (Bug #8) | Vendor-reference pattern for ComposableNodes |
| `enable_imu_fusion: True` | In dict | Was `false` by default (Bug #7) | Compensates for low visual fps on USB 2.0 |
| `image_jitter_threshold_ms: 35.0` | Float | Was stuck at 34.0 (Bug #9) | Configurable in launch file for tuning |

### File 3: `scripts/start_robot.sh` (MODIFY)

**Section modified:** Step 4c — camera launch command (lines ~67-72)

**Before:**
```bash
# 4c. Depth camera (OAK-D Pro via depthai-ros)
#     camera.i_usb_speed:=HIGH forces USB 2.0 (480Mbps) mode for stability.
#     USB 3.0 (SUPER) negotiation succeeds but connection drops immediately
#     with SIGABRT/X_LINK_ERROR on this hardware.
echo "[start_robot.sh] Starting OAK-D Pro camera..."
ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-PRO \
    camera.i_usb_speed:=HIGH \
    left.i_publish_topic:=true right.i_publish_topic:=true &
CAMERA_PID=$!
sleep 3
```

**After:**
```bash
# 4c. Depth camera (OAK-D Pro via depthai-ros)
#     Parameters delivered via YAML params_file (CLI key:=value overrides are
#     silently ignored by camera.launch.py — it has no DeclareLaunchArgument).
#     oakd_params.yaml sets: USB 2.0 speed, RGBD pipeline, synced stereo rect
#     pair publishing, 400P@30fps stereo (raw), 480P@15fps RGB (MJPEG), IR off.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "[start_robot.sh] Starting OAK-D Pro camera..."
ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-PRO \
    params_file:=${SCRIPT_DIR}/oakd_params.yaml &
CAMERA_PID=$!
sleep 3
```

**Changes:**
1. Comment block rewritten for accuracy (Decision #4)
2. `SCRIPT_DIR` variable added to resolve `oakd_params.yaml` path relative to script location
3. Three broken CLI parameter overrides replaced with single `params_file:=` argument
4. `camera_model:=OAK-D-PRO` retained — this IS a valid launch argument declared in `camera.launch.py`

### File 4: `scripts/voice_mapper.py` (MODIFY)

**Method modified:** `start_isaac_vslam()` (line ~1398)

> **Scope note:** This change removes both the OAK_D_PRO and REALSENSE remapping branches. The replacement launch file (`oakd_vslam.launch.py`) is OAK-D Pro specific with hardcoded `/oak/` topic remappings. A camera-type guard is added so that non-OAK cameras fail explicitly rather than launching with wrong topics. RealSense VSLAM support can be re-added later by creating a separate `realsense_vslam.launch.py` and extending the guard.

**Before (lines 1424-1467 — full removal scope):**
```python
            # Build topic remapping for camera
            # Isaac VSLAM expects: /visual_slam/image_0, /visual_slam/image_1, etc.
            remappings = []
            
            if self.camera_type == CameraType.OAK_D_PRO:
                # OAK-D Pro topic remapping
                remappings = [
                    f"/visual_slam/image_0:={self.camera_config.left_topic}",
                    f"/visual_slam/image_1:={self.camera_config.right_topic}",
                    f"/visual_slam/camera_info_0:={self.camera_config.camera_info_left}",
                    f"/visual_slam/camera_info_1:={self.camera_config.camera_info_right}",
                ]
                if self.camera_config.imu_topic:
                    remappings.append(f"/visual_slam/imu:={self.camera_config.imu_topic}")
            
            elif self.camera_type == CameraType.REALSENSE:
                # RealSense topic remapping
                remappings = [
                    f"/visual_slam/image_0:={self.camera_config.left_topic}",
                    f"/visual_slam/image_1:={self.camera_config.right_topic}",
                    f"/visual_slam/camera_info_0:={self.camera_config.camera_info_left}",
                    f"/visual_slam/camera_info_1:={self.camera_config.camera_info_right}",
                ]
                if self.camera_config.imu_topic:
                    remappings.append(f"/visual_slam/imu:={self.camera_config.imu_topic}")
            
            # Launch Isaac VSLAM node
            cmd = [
                "ros2", "launch", "isaac_ros_visual_slam", "isaac_ros_visual_slam.launch.py",
                "num_cameras:=2",
                "enable_imu_fusion:=true" if self.camera_config.imu_topic else "enable_imu_fusion:=false",
                "enable_localization_n_mapping:=true",
                "enable_slam_visualization:=true",
            ]
            
            # Add remappings (--ros-args -r syntax for ros2 launch)
            if remappings:
                cmd.append("--ros-args")
                for remap in remappings:
                    cmd.append("-r")
                    cmd.append(remap)
```

**After:**
```python
            # Launch Isaac VSLAM via custom launch file with ComposableNode
            # params and remappings baked in (stock launcher silently ignores
            # all launch arguments — no DeclareLaunchArgument defined).
            # NOTE: oakd_vslam.launch.py is OAK-D Pro specific. RealSense would
            # need a separate launch file if re-added in the future.
            if self.camera_type != CameraType.OAK_D_PRO:
                self.get_logger().error(
                    f"❌ VSLAM launch only supports OAK-D Pro, got {self.camera_type.value}")
                return False
            launch_file = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "oakd_vslam.launch.py"
            )
            cmd = ["ros2", "launch", launch_file]
```

**Changes:**
1. Entire remappings list construction removed (lines 1424-1450) including `remappings = []` init, OAK_D_PRO branch, and REALSENSE branch — all remappings are now in the launch file
2. Camera-type guard added — prevents silently launching OAK-specific file for non-OAK cameras
3. Complex `cmd` with 4 ignored launch arguments replaced with simple 3-element list
4. `--ros-args -r` remapping append loop removed
5. Launch file resolved relative to script location (same directory after deployment)
6. Comment explains WHY the custom launch file is needed and notes OAK-D Pro specificity

### Codebase Patterns

```yaml
codebase_patterns:
  - pattern: Shell script hardware launcher
    location: "scripts/start_robot.sh"
    usage: Camera launch command modification
  - pattern: Python ROS2 node with subprocess
    location: "scripts/voice_mapper.py"
    usage: VSLAM launch subprocess modification
  - pattern: ROS2 YAML parameter files
    location: "/opt/ros/humble/share/depthai_ros_driver/config/camera.yaml"
    usage: Reference for oakd_params.yaml structure
  - pattern: ComposableNode launch files
    location: "/opt/ros/humble/share/isaac_ros_visual_slam/launch/isaac_ros_visual_slam_realsense.launch.py"
    usage: Reference for oakd_vslam.launch.py structure
```

### Data Contracts

No data entities in scope — data contracts not applicable.

## Dependencies

| Dependency | Type | Status | Notes |
|------------|------|--------|-------|
| Research 003 complete | Research | ✅ Done | All 4 phases complete; root causes identified |
| DepthAI ROS driver v2.12.2 | Package | ✅ Installed | `ros-humble-depthai-ros-driver` on robot |
| Isaac ROS Visual SLAM v3.2.6 | Package | ✅ Installed | `ros-humble-isaac-ros-visual-slam` on robot |
| OAK-D Pro on USB 2.0 bus | Hardware | ✅ Confirmed | Bus 01 Port 2→2→1, 480M max |
| SSH access to robot | Infrastructure | ✅ Available | `jetson@192.168.7.250` |
| `camera_model:=OAK-D-PRO` launch arg | Launch system | ✅ Valid | This IS a declared launch argument in `camera.launch.py` |

## Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| 400P@30fps stereo may not sustain on USB 2.0 with double hub | Medium | High — VSLAM fails below 30Hz | Increase `image_jitter_threshold_ms` to match actual fps; lower stereo fps to 20 and enable IMU fusion to compensate |
| `i_reverse_stereo_socket_order` may be wrong for this specific OAK-D Pro unit | Low | Medium — stereo depth inverted | Set to `false` if depth map appears inverted; toggle and observe |
| IMU noise parameters (RealSense defaults) may not match BMI270 | Medium | Low — VSLAM tracking slightly degraded | Functional with approximate values; Allan variance characterization is out of scope |
| MJPEG RGB at quality 50 too lossy for LLM vision | Low | Low — LLM misidentifies objects | Increase `i_low_bandwidth_quality` to 70-80; test with GPT-4o vision |
| `stereo.i_output_disparity` parameter may not exist in RGBD pipeline | Low | None — depth is always produced in RGBD | Remove parameter if driver warns; depth should publish regardless |
| Camera process startup order — stereo rect needs extra time | Low | Low — VSLAM waits in 30s loop | Existing 30-second timeout in `start_isaac_vslam()` handles this |

## Execution Plan

### Phase 1: Create New Configuration Files

**Status:** ✅ Complete
**Size:** Small
**Files to Modify:** 2 (new files)
**Prerequisites:** None
**Entry Point:** `scripts/` directory
**Verification:** Files exist with correct YAML/Python syntax

| Step | Task | Files | Acceptance Criteria |
|------|------|-------|---------------------|
| 1.1 | Create `oakd_params.yaml` with RGBD pipeline, USB HIGH, synced stereo rect pair, 400P@30fps stereo, 480P@15fps MJPEG RGB, IR off, NN none. Include header comment block explaining bandwidth budget and the CLI-override-ignored warning. Use exact YAML structure from Technical Design section. | `scripts/oakd_params.yaml` | File parses as valid YAML; `/oak/ros__parameters/camera/i_pipeline_type` = `RGBD`; `stereo/i_publish_synced_rect_pair` = `true`; `camera/i_usb_speed` = `"HIGH"`; `camera/i_enable_ir` = `false` |
| 1.2 | Create `oakd_vslam.launch.py` with ComposableNode parameters dict and remappings list. Follow vendor RealSense reference pattern. Include `enable_imu_fusion: True`, correct optical frame names, IMU noise params (RealSense defaults with TODO comment), and all 5 topic remappings. Use exact Python from Technical Design section. | `scripts/oakd_vslam.launch.py` | File parses as valid Python; `generate_launch_description()` returns `LaunchDescription`; ComposableNode has `parameters` list with dict containing `enable_imu_fusion: True`; `remappings` list has 5 entries mapping to `/oak/` topics |

### Phase 2: Update Existing Scripts

**Status:** ✅ Complete
**Size:** Small
**Files to Modify:** 2
**Prerequisites:** Phase 1 complete (new files exist)
**Entry Point:** `scripts/start_robot.sh` line ~67, `scripts/voice_mapper.py` line ~1398
**Verification:** Phase 1 files referenced correctly; no broken CLI parameter overrides remain

| Step | Task | Files | Acceptance Criteria |
|------|------|-------|---------------------|
| 2.1 | Update `start_robot.sh` section 4c: (a) Replace comment block with accurate description explaining `params_file:=` mechanism and why CLI overrides are ignored. (b) Add `SCRIPT_DIR` variable: `SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"`. (c) Replace 3-line camera launch command with single line using `params_file:=${SCRIPT_DIR}/oakd_params.yaml`. (d) Keep `camera_model:=OAK-D-PRO` (valid launch arg). (e) Keep `CAMERA_PID=$!` and `sleep 3`. | `scripts/start_robot.sh` | No `:=` parameter overrides remain except `camera_model` and `params_file`; `SCRIPT_DIR` resolves to script's directory; comment accurately describes `params_file` mechanism |
| 2.2 | Update `voice_mapper.py` `start_isaac_vslam()` method: (a) Remove entire block from `# Build topic remapping for camera` comment through the `--ros-args -r` append loop (lines ~1424-1467). This removes: the `remappings = []` init, the OAK_D_PRO branch, the REALSENSE branch, the old `cmd` construction, and the remappings append loop. (b) Add camera-type guard: `if self.camera_type != CameraType.OAK_D_PRO: ... return False`. (c) Replace with 3-element `cmd`: `["ros2", "launch", launch_file]` where `launch_file` is resolved via `os.path.join(os.path.dirname(os.path.abspath(__file__)), "oakd_vslam.launch.py")`. (d) Add comment explaining why custom launch file is needed and noting OAK-D Pro specificity. See consolidated Before/After in Technical Design File 4. | `scripts/voice_mapper.py` | `start_isaac_vslam()` method has no `--ros-args`, no `-r` remappings, no `:=` launch arguments, no `remappings` variable; `cmd` is exactly 3 elements; launch file path resolved relative to script; non-OAK camera types return `False` with error log |

### Phase 3: Validation Checklist

**Status:** 🔄 In Progress
**Size:** Small
**Files to Modify:** 0
**Prerequisites:** Phase 2 complete
**Entry Point:** Deploy files to robot and run
**Verification:** Post-deployment checks listed below

| Step | Task | Files | Acceptance Criteria |
|------|------|-------|---------------------|
| 3.1 | Deploy all 4 files to robot via `scp scripts/* jetson@192.168.7.250:~/robot_scripts/` | All 4 files | Files exist on robot at `/home/jetson/robot_scripts/` |
| 3.2 | Start hardware via `start_robot.sh` and verify camera driver logs show `USB SPEED: HIGH` (not SUPER) | — | `ros2 node info /oak` shows node running; no SIGABRT or X_LINK_ERROR in first 30 seconds |
| 3.3 | Verify stereo rect topics have publishers: `ros2 topic info /oak/left/image_rect` and `/oak/right/image_rect` should show 1 publisher each | — | Publisher count ≥ 1 on both topics |
| 3.4 | Verify stereo rect data flowing: `ros2 topic hz /oak/left/image_rect` should report non-zero Hz | — | Hz > 0; ideally ≥ 20 fps |
| 3.5 | Verify RGB still working: `ros2 topic hz /oak/rgb/image_raw` should report ~15fps | — | Hz > 0 |
| 3.6 | Start VSLAM via voice command or direct call, verify `vslam_tracking` becomes `True` within 30 seconds | — | Isaac VSLAM process running; tracking active |

## Standards

⚠️ Could not access organizational standards from pch-standards-space. Proceeding without standards context.

No organizational standards applicable to this plan.

## Implementation Complexity

| Factor | Score (1-5) | Notes |
|--------|-------------|-------|
| Files to modify | 2 | 2 new files + 2 modified files in `scripts/` |
| New patterns introduced | 1 | YAML params_file pattern already exists in DepthAI ecosystem |
| External dependencies | 1 | All packages already installed on robot |
| Migration complexity | 1 | No data migration; atomic file deployment via scp |
| Test coverage required | 2 | Hardware validation checklist (Phase 3); no unit tests applicable |
| **Overall Complexity** | **7/25** | **Low** — narrow scope, well-researched, vendor-reference patterns |

## Review Summary

**Review Date:** 2026-02-24
**Reviewer:** pch-plan-reviewer
**Original Plan Version:** v2.0
**Reviewed Plan Version:** v2.1

### Review Metrics
- Issues Found: 4 (Critical: 0, Major: 0, Minor: 4)
- Clarifying Questions Asked: 0
- Sections Updated: Frontmatter, Version History, Technical Design File 4, Execution Plan Step 2.2, Handoff

### Key Improvements Made
1. Consolidated Before/After for `voice_mapper.py` — now shows the full removal scope (lines 1424-1467) instead of only the `cmd` block, preventing partial implementation
2. Added camera-type guard in replacement code — non-OAK cameras fail explicitly with error log instead of silently launching with wrong topics
3. Documented intentional RealSense VSLAM branch removal as a scope note with guidance for future re-addition
4. Fixed frontmatter status contradiction (`complete` → `ready-for-implementation`)

### Remaining Considerations
- USB 2.0 bandwidth (~31.5 MB/s target vs ~30 MB/s capacity) requires live validation. The `image_jitter_threshold_ms` parameter is the primary tuning knob if actual fps < 30.
- IMU noise parameters are RealSense reference values. Functional but not optimal for OAK-D Pro BMI270. Allan variance characterization deferred.
- `i_reverse_stereo_socket_order: true` is from vendor reference — toggle if depth map appears inverted during live testing.

### Sign-off
This plan has been reviewed and is **Ready for Implementation**.

## Implementation Notes

### Phase 1 — Create New Configuration Files
**Completed:** 2026-02-24
**Execution Mode:** Automatic (Subagent)

**Files Created:**
- `scripts/oakd_params.yaml` — DepthAI driver YAML params (RGBD, USB HIGH, synced stereo rect, IR off)
- `scripts/oakd_vslam.launch.py` — Isaac VSLAM ComposableNode launch (params + 5 remappings)

**Deviations from Plan:** None

### Phase 2 — Update Existing Scripts
**Completed:** 2026-02-24
**Execution Mode:** Automatic (Subagent)

**Files Modified:**
- `scripts/start_robot.sh` — Replaced CLI overrides with `params_file:=` + `SCRIPT_DIR` resolution
- `scripts/voice_mapper.py` — Replaced remappings/CLI args with custom launch file delegation + camera-type guard

**Deviations from Plan:** None

### Phase 3 — Validation Checklist
**Completed:** 2026-02-24 (partial — deployment + syntax validation only)

**Step 3.1:** Deployed all 4 files to robot via SCP — confirmed at `/home/jetson/robot_scripts/`
**Syntax Validation:** YAML, launch file, and voice_mapper.py all parse successfully on robot
**Steps 3.2-3.6:** Require live hardware start — deferred to separate verification task (out of scope per plan)

### Code Review
**Status:** Clean (no regressions)
**Pre-existing findings:** 2 minor exception handling patterns in `stop_isaac_vslam()` and `start_isaac_vslam()` error path — not introduced by this implementation, not in scope.

### Plan Completion
**All phases completed:** 2026-02-24
**Total tasks completed:** 4 (1.1, 1.2, 2.1, 2.2) + partial Phase 3 (3.1 + syntax validation)
**Total files created:** 2
**Total files modified:** 2

## Handoff

| Field | Value |
|-------|-------|
| Created By | pch-planner |
| Created Date | 2026-02-24 |
| Reviewed By | pch-plan-reviewer |
| Review Date | 2026-02-24 |
| Status | ✅ Complete |
| Implemented By | pch-coder |
| Implementation Date | 2026-02-24 |
| Plan Location | /docs/plans/003-stereo-vslam-launch-fix.md |
