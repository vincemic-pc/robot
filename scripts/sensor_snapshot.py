"""Sensor snapshot pipeline for LLM-driven autonomous exploration.

Captures camera/depth/LiDAR/odometry into immutable snapshots for VLM
consumption.  Includes image annotation, 12-sector LiDAR summaries,
affordance scoring, and MAD-based frame similarity.

Plan 006: LLM-Driven Autonomous Exploration Navigation.
"""

import time
import math
import base64
import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data Models
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SensorSnapshot:
    """Immutable snapshot of all sensor data for one VLM decision cycle."""
    timestamp: float                          # time.time()
    annotated_image_b64: str                  # 640x480 JPEG with depth overlay + heading, base64
    lidar_summary: str                        # 12-sector text (000° front: 2.9m CLEAR ...)
    doorway_gaps: list                        # [{direction_deg, width_m, distance_m}, ...]
    robot_state: dict                         # {x, y, heading_deg, heading_cardinal, speed_mps,
                                              #  distance_traveled_m, explore_time_s}
    affordance_scores: dict                   # {forward: 0.9, left: 0.1, ...}
    previous_action_result: Optional[dict]    # {success, actual_distance_m, stopped_reason, ...}


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# 7 depth sample points on a 640x480 image
DEPTH_SAMPLE_POINTS = [
    (320, 240, "center"),          # center
    (160, 240, "left-center"),     # left-center
    (480, 240, "right-center"),    # right-center
    (320, 400, "floor-ahead"),     # floor ahead
    (160, 120, "upper-left"),      # upper-left
    (480, 120, "upper-right"),     # upper-right
    (320, 100, "upper-center"),    # upper-center
]

# 12 LiDAR sectors (30° each), starting from front (0°) going counter-clockwise
SECTOR_LABELS = [
    "000° front",
    "030° front-left",
    "060° left-front",
    "090° left",
    "120° left-rear",
    "150° rear-left",
    "180° rear",
    "210° rear-right",
    "240° right-rear",
    "270° right",
    "300° right-front",
    "330° front-right",
]

# Qualitative distance labels for LiDAR sectors
DIST_WALL = 0.5       # < 0.5m  → WALL
DIST_OBSTACLE = 1.0   # < 1.0m  → OBSTACLE
DIST_NEAR = 2.0       # < 2.0m  → NEAR
                       # >= 2.0m → CLEAR

# Affordance scoring thresholds
AFFORDANCE_BLOCKED = 0.5   # distance below which score = 0.1
AFFORDANCE_CLEAR = 2.0     # distance above which score = 1.0

# Direction-to-obstacle_distances key mapping for affordance scores
AFFORDANCE_DIRECTIONS = {
    "forward": "front",
    "forward_left": "front_left",
    "forward_right": "front_right",
    "left": "left",
    "right": "right",
    "backward": "back",
}

# Cardinal directions from heading degrees
CARDINAL_DIRECTIONS = [
    (0, "N"), (45, "NE"), (90, "E"), (135, "SE"),
    (180, "S"), (225, "SW"), (270, "W"), (315, "NW"), (360, "N"),
]


# ---------------------------------------------------------------------------
# Standalone Functions
# ---------------------------------------------------------------------------

def annotate_image(image_msg, depth_msg, heading_deg, heading_cardinal, bridge):
    """Overlay 7 depth values and heading arrow on 640x480 RGB.

    Returns base64-encoded JPEG string, or None on failure.
    """
    if not HAS_CV or image_msg is None:
        return None

    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        cv_image = cv2.resize(cv_image, (640, 480))

        # --- Depth overlays ---
        depth_image = None
        if depth_msg is not None:
            try:
                depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            except Exception:
                depth_image = None

        for px, py, label in DEPTH_SAMPLE_POINTS:
            depth_text = "?"
            if depth_image is not None:
                depth_m = _sample_depth(depth_image, px, py)
                if depth_m is not None:
                    depth_text = f"{depth_m:.1f}m"

            # White text with black outline for readability
            _draw_outlined_text(cv_image, depth_text, (px, py),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 255), (0, 0, 0))

        # --- Heading arrow in top-right corner ---
        arrow_cx, arrow_cy = 580, 50
        arrow_len = 30
        # Convert heading to image angle: 0°=up, CW positive
        angle_rad = math.radians(-heading_deg + 90)  # to image coords (right=0, up=90)
        ax = int(arrow_cx + arrow_len * math.cos(angle_rad))
        ay = int(arrow_cy - arrow_len * math.sin(angle_rad))
        cv2.arrowedLine(cv_image, (arrow_cx, arrow_cy), (ax, ay),
                        (0, 255, 0), 2, tipLength=0.35)
        _draw_outlined_text(cv_image, heading_cardinal, (arrow_cx - 15, arrow_cy + 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), (0, 0, 0))

        # --- Encode to base64 JPEG ---
        _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return base64.b64encode(buffer).decode('utf-8')

    except Exception as e:
        logger.error(f"annotate_image error: {e}")
        return None


def build_lidar_summary(ranges, angle_min, angle_inc):
    """Build 12-sector (30° each) LiDAR text summary with qualitative labels."""
    if ranges is None or len(ranges) == 0:
        return "LIDAR: no data"

    ranges = np.array(ranges, dtype=np.float64)
    num_points = len(ranges)

    # Clean invalid readings — same pattern as scan_callback
    min_range = 0.12
    max_range = 12.0
    ranges_clean = np.where(
        np.isinf(ranges) | np.isnan(ranges) | (ranges < min_range),
        max_range, ranges
    )

    # Determine angle of each point in degrees (0-360)
    angles_deg = np.degrees(angle_min + np.arange(num_points) * angle_inc) % 360

    lines = ["LIDAR SCAN (12 sectors, 30° each):"]

    for sector_idx in range(12):
        sector_start = sector_idx * 30.0
        sector_end = sector_start + 30.0

        # Select points in this sector
        if sector_end <= 360:
            mask = (angles_deg >= sector_start) & (angles_deg < sector_end)
        else:
            mask = (angles_deg >= sector_start) | (angles_deg < (sector_end % 360))

        sector_ranges = ranges_clean[mask]

        if len(sector_ranges) == 0:
            dist = max_range
        else:
            dist = float(np.percentile(sector_ranges, 10))

        # Qualitative label
        if dist < DIST_WALL:
            label = "WALL"
        elif dist < DIST_OBSTACLE:
            label = "OBSTACLE"
        elif dist < DIST_NEAR:
            label = "NEAR"
        else:
            label = "CLEAR"

        lines.append(f"  {SECTOR_LABELS[sector_idx]}: {dist:.1f}m {label}")

    return "\n".join(lines)


def compute_affordance_scores(obstacle_distances):
    """Map 6 navigation directions to feasibility scores (0.1–1.0) from LiDAR."""
    scores = {}
    for direction, obs_key in AFFORDANCE_DIRECTIONS.items():
        dist = obstacle_distances.get(obs_key, 10.0)
        if dist <= AFFORDANCE_BLOCKED:
            score = 0.1
        elif dist >= AFFORDANCE_CLEAR:
            score = 1.0
        else:
            # Linear interpolation
            score = 0.1 + 0.9 * (dist - AFFORDANCE_BLOCKED) / (AFFORDANCE_CLEAR - AFFORDANCE_BLOCKED)
        scores[direction] = round(score, 2)
    return scores


def compute_frame_similarity(frame_a, frame_b):
    """MAD-based frame similarity (0.0=different, 1.0=identical) on 160x120 grayscale."""
    if not HAS_CV or frame_a is None or frame_b is None:
        return 0.0

    try:
        # Convert to grayscale if needed
        if len(frame_a.shape) == 3:
            gray_a = cv2.cvtColor(frame_a, cv2.COLOR_BGR2GRAY)
        else:
            gray_a = frame_a
        if len(frame_b.shape) == 3:
            gray_b = cv2.cvtColor(frame_b, cv2.COLOR_BGR2GRAY)
        else:
            gray_b = frame_b

        # Downsample to 160x120
        small_a = cv2.resize(gray_a, (160, 120))
        small_b = cv2.resize(gray_b, (160, 120))

        # Mean absolute difference, normalized to 0-1
        mad = np.mean(np.abs(small_a.astype(np.float32) - small_b.astype(np.float32))) / 255.0
        return round(1.0 - mad, 4)

    except Exception as e:
        logger.error(f"compute_frame_similarity error: {e}")
        return 0.0


# ---------------------------------------------------------------------------
# Event Trigger (Phase 5, Task 5.1)
# ---------------------------------------------------------------------------

# Trigger thresholds
DOORWAY_MIN_WIDTH_M = 0.7      # Gap must be >= 0.7m wide to count as doorway
T_INTERSECTION_OPEN_SECTORS = 3 # Need 3+ adjacent open sectors for intersection
DEAD_END_FORWARD_MAX_M = 1.0   # All forward sectors < 1.0m = dead end
POSITION_TRIGGER_M = 1.5       # Trigger if moved > 1.5m since last VLM call
TRIGGER_COOLDOWN_S = 2.0       # No re-trigger within 2 seconds


class EventTrigger:
    """Monitors LiDAR/position for events that should trigger immediate VLM calls.

    Triggers: doorway detected, T-intersection, dead end, position-based movement.
    """

    def __init__(self):
        self._last_trigger_time = 0.0
        self._last_trigger_position = None  # (x, y) at last VLM call

    def reset(self):
        """Reset trigger state (call when exploration starts)."""
        self._last_trigger_time = 0.0
        self._last_trigger_position = None

    def record_vlm_call(self, position):
        """Record that a VLM call was made at this position (resets position trigger)."""
        self._last_trigger_position = (position.get('x', 0.0), position.get('y', 0.0))

    def check_triggers(self, obstacle_distances, detected_gaps, position,
                       last_trigger_time=None):
        """Check all trigger conditions. Returns list of trigger reason strings.

        Args:
            obstacle_distances: dict with keys front, front_left, front_right, left, right, back
            detected_gaps: list of dicts with angle, width, distance keys
            position: dict with x, y keys
            last_trigger_time: override for testing (else uses internal state)

        Returns:
            List of trigger reason strings (empty if no triggers fired).
        """
        now = time.time()
        ref_time = last_trigger_time if last_trigger_time is not None else self._last_trigger_time

        # Cooldown check
        if now - ref_time < TRIGGER_COOLDOWN_S:
            return []

        triggers = []

        # 1. Doorway detection — any gap >= 0.7m width
        for gap in (detected_gaps or []):
            width = gap.get('width', 0.0)
            if width >= DOORWAY_MIN_WIDTH_M:
                triggers.append(f"doorway(width={width:.1f}m)")
                break  # One doorway trigger is enough

        # 2. T-intersection — 3+ adjacent LiDAR directions are open (>= 2.0m)
        open_directions = self._count_adjacent_open(obstacle_distances)
        if open_directions >= T_INTERSECTION_OPEN_SECTORS:
            triggers.append(f"intersection({open_directions}_open)")

        # 3. Dead end — all forward sectors < 1.0m
        if self._is_dead_end(obstacle_distances):
            triggers.append("dead_end")

        # 4. Position-based — moved > 1.5m since last VLM call
        if self._last_trigger_position is not None:
            dx = position.get('x', 0.0) - self._last_trigger_position[0]
            dy = position.get('y', 0.0) - self._last_trigger_position[1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > POSITION_TRIGGER_M:
                triggers.append(f"moved({dist:.1f}m)")

        if triggers:
            self._last_trigger_time = now

        return triggers

    def _count_adjacent_open(self, obstacle_distances):
        """Count max run of adjacent open directions (distance >= 2.0m).

        Uses the 6 obstacle_distances keys arranged as a ring:
        front, front_left, left, back, right, front_right
        """
        ring = ["front", "front_left", "left", "back", "right", "front_right"]
        n = len(ring)
        open_flags = [obstacle_distances.get(k, 10.0) >= 2.0 for k in ring]

        # Find max run of consecutive True in circular array
        if not any(open_flags):
            return 0
        max_run = 0
        run = 0
        # Double the array for circular detection
        for i in range(2 * n):
            if open_flags[i % n]:
                run += 1
                max_run = max(max_run, run)
            else:
                run = 0
        return min(max_run, n)  # Cap at ring size

    def _is_dead_end(self, obstacle_distances):
        """Check if all forward-facing sectors are close (< 1.0m)."""
        forward_keys = ["front", "front_left", "front_right"]
        return all(
            obstacle_distances.get(k, 10.0) < DEAD_END_FORWARD_MAX_M
            for k in forward_keys
        )


# ---------------------------------------------------------------------------
# Builder
# ---------------------------------------------------------------------------

class SensorSnapshotBuilder:
    """Builds SensorSnapshot from VoiceMapper sensor state references."""

    def __init__(self, vm):
        """Initialize with a VoiceMapper reference for sensor access."""
        self.vm = vm
        self._bridge = CvBridge() if HAS_CV else None
        self._last_frame = None          # For frame similarity comparison
        self._explore_start_time = None  # Set when exploration starts
        self._start_position = None      # Set when exploration starts
        self._previous_action_result = None

    def set_previous_action_result(self, result):
        """Store the result of the last executed action for the next snapshot."""
        self._previous_action_result = result

    def reset_exploration_tracking(self):
        """Reset distance/time tracking when exploration starts."""
        self._explore_start_time = time.time()
        pos = self.vm.current_position
        self._start_position = (pos['x'], pos['y'])
        self._last_frame = None

    def capture(self):
        """Assemble a full SensorSnapshot from current VoiceMapper state."""
        vm = self.vm
        now = time.time()

        # --- Position & heading ---
        pos = vm.current_position
        heading_rad = pos.get('theta', 0.0)
        heading_deg = math.degrees(heading_rad) % 360
        heading_cardinal = _heading_to_cardinal(heading_deg)

        # --- Speed from odometry ---
        speed_mps = 0.0
        if vm.latest_odom is not None:
            speed_mps = abs(vm.latest_odom.twist.twist.linear.x)

        # --- Distance traveled & explore time ---
        distance_traveled = 0.0
        explore_time = 0.0
        if self._start_position is not None:
            dx = pos['x'] - self._start_position[0]
            dy = pos['y'] - self._start_position[1]
            distance_traveled = math.sqrt(dx * dx + dy * dy)
        if self._explore_start_time is not None:
            explore_time = now - self._explore_start_time

        # --- Robot state dict ---
        robot_state = {
            'x': round(pos['x'], 2),
            'y': round(pos['y'], 2),
            'heading_deg': round(heading_deg, 1),
            'heading_cardinal': heading_cardinal,
            'speed_mps': round(speed_mps, 3),
            'distance_traveled_m': round(distance_traveled, 2),
            'explore_time_s': round(explore_time, 1),
        }

        # --- Annotated image ---
        annotated_b64 = annotate_image(
            vm.latest_image, vm.latest_depth,
            heading_deg, heading_cardinal,
            self._bridge
        )
        if annotated_b64 is None:
            logger.warning("No annotated image — camera or depth unavailable")
            annotated_b64 = ""

        # --- LiDAR summary ---
        lidar_summary = "LIDAR: no data"
        if vm.latest_scan is not None:
            scan = vm.latest_scan
            lidar_summary = build_lidar_summary(
                scan.ranges, scan.angle_min, scan.angle_increment
            )

        # --- Doorway gaps ---
        doorway_gaps = []
        for gap in getattr(vm, 'detected_gaps', []):
            doorway_gaps.append({
                'direction_deg': round(gap.get('angle', 0), 1),
                'width_m': round(gap.get('width', 0), 2),
                'distance_m': round(gap.get('distance', 0), 2),
            })

        # --- Affordance scores ---
        affordance = compute_affordance_scores(
            getattr(vm, 'obstacle_distances', {})
        )

        # --- Store current frame for future similarity comparison ---
        if HAS_CV and vm.latest_image is not None and self._bridge is not None:
            try:
                self._last_frame = self._bridge.imgmsg_to_cv2(
                    vm.latest_image, desired_encoding="bgr8"
                )
            except Exception:
                pass

        return SensorSnapshot(
            timestamp=now,
            annotated_image_b64=annotated_b64,
            lidar_summary=lidar_summary,
            doorway_gaps=doorway_gaps,
            robot_state=robot_state,
            affordance_scores=affordance,
            previous_action_result=self._previous_action_result,
        )

    def get_last_frame(self):
        """Return the last captured camera frame (for similarity comparison)."""
        return self._last_frame


# ---------------------------------------------------------------------------
# Internal Helpers
# ---------------------------------------------------------------------------

def _sample_depth(depth_image, px, py):
    """Sample depth at pixel (px, py) using 5x5 median. Returns metres or None."""
    h, w = depth_image.shape[:2]

    # Scale coordinates if depth image resolution differs from 640x480
    scale_x = w / 640.0
    scale_y = h / 480.0
    dx = int(px * scale_x)
    dy = int(py * scale_y)

    dx = max(0, min(dx, w - 1))
    dy = max(0, min(dy, h - 1))

    region_size = 5
    x_start = max(0, dx - region_size)
    x_end = min(w, dx + region_size + 1)
    y_start = max(0, dy - region_size)
    y_end = min(h, dy + region_size + 1)

    region = depth_image[y_start:y_end, x_start:x_end]
    valid = region[region > 0]

    if len(valid) == 0:
        return None

    depth_mm = float(np.median(valid))
    depth_m = depth_mm / 1000.0

    if 0.1 < depth_m < 10.0:
        return depth_m
    return None


def _draw_outlined_text(img, text, pos, font, scale, fg_color, bg_color):
    """Draw text with an outline for readability on any background."""
    x, y = pos
    thickness = 1
    # Black outline
    cv2.putText(img, text, (x, y), font, scale, bg_color, thickness + 2, cv2.LINE_AA)
    # White foreground
    cv2.putText(img, text, (x, y), font, scale, fg_color, thickness, cv2.LINE_AA)


def _heading_to_cardinal(heading_deg):
    """Convert heading in degrees (0=N, CW) to cardinal direction string."""
    heading_deg = heading_deg % 360
    best = "N"
    best_diff = 360
    for deg, cardinal in CARDINAL_DIRECTIONS:
        diff = abs(heading_deg - deg)
        if diff < best_diff:
            best_diff = diff
            best = cardinal
    return best
