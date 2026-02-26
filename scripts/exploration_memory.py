"""Exploration memory for LLM-driven autonomous exploration.

Tracks rooms, frontiers, dead ends, and recent action history across an
exploration session.  Serialises to a compact text block (~200 tokens) for
inclusion in each VLM prompt, and persists to YAML between sessions.

Plan 006: LLM-Driven Autonomous Exploration Navigation.
"""

import time
import logging
from dataclasses import dataclass, field
from typing import Optional

import yaml

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data Model
# ---------------------------------------------------------------------------

@dataclass
class ExplorationState:
    """Persistent exploration memory serialised into VLM prompt."""
    rooms: list = field(default_factory=list)
    # [{name, explored_pct, objects, entered_from}, ...]
    current_room: str = "start"
    dead_ends: list = field(default_factory=list)
    # [{direction_deg, reason, position:{x,y}}, ...]
    frontiers: list = field(default_factory=list)
    # [{id, direction_deg, distance_m, type, width_m}, ...]
    recent_actions: list = field(default_factory=list)
    # Last 5: [{action, params, result, position:{x,y}}, ...]
    total_discoveries: int = 0
    exploration_duration_s: float = 0.0


# ---------------------------------------------------------------------------
# Exploration Memory
# ---------------------------------------------------------------------------

MAX_RECENT_ACTIONS = 5
DEAD_END_MERGE_DIST = 0.5  # metres — merge nearby dead ends


class ExplorationMemory:
    """Maintains and serialises exploration state for the VLM decision loop."""

    def __init__(self, origin_x: float = 0.0, origin_y: float = 0.0):
        self.state = ExplorationState()
        self._start_time = time.time()

        # Seed the initial "start" room at the robot's origin
        self.state.rooms.append({
            'name': 'start',
            'explored_pct': 0,
            'objects': [],
            'entered_from': None,
            'position': {'x': round(origin_x, 2), 'y': round(origin_y, 2)},
        })

    # ------------------------------------------------------------------
    # Action tracking
    # ------------------------------------------------------------------

    def update_after_action(self, decision, result, position):
        """Record the outcome of the latest VLM action.

        Args:
            decision: NavigationDecision (or dict with tool_name, parameters).
            result: dict with at least {success: bool, ...}.
            position: dict with {x, y}.
        """
        tool_name = decision.tool_name if hasattr(decision, 'tool_name') else decision.get('tool_name', '?')
        params = decision.parameters if hasattr(decision, 'parameters') else decision.get('parameters', {})

        # Build a compact params summary
        params_summary = _summarise_params(tool_name, params)

        entry = {
            'action': tool_name,
            'params': params_summary,
            'result': _summarise_result(result),
            'position': {'x': round(position.get('x', 0), 2),
                         'y': round(position.get('y', 0), 2)},
        }
        self.state.recent_actions.append(entry)

        # Keep only the last N
        if len(self.state.recent_actions) > MAX_RECENT_ACTIONS:
            self.state.recent_actions = self.state.recent_actions[-MAX_RECENT_ACTIONS:]

        # Update exploration duration
        self.state.exploration_duration_s = time.time() - self._start_time

    # ------------------------------------------------------------------
    # Discoveries & rooms
    # ------------------------------------------------------------------

    def record_discovery(self, label, notes, significance, position):
        """Log a discovery. If significance=='room', create a new room.

        Args:
            label: str — discovery name (e.g. "kitchen").
            notes: str — brief description.
            significance: str — "landmark" | "room" | "obstacle" | "point_of_interest".
            position: dict with {x, y}.
        """
        self.state.total_discoveries += 1

        if significance == 'room':
            # Check for duplicate room name
            existing = [r for r in self.state.rooms if r['name'] == label]
            if not existing:
                self.state.rooms.append({
                    'name': label,
                    'explored_pct': 0,
                    'objects': [],
                    'entered_from': self.state.current_room,
                    'position': {'x': round(position.get('x', 0), 2),
                                 'y': round(position.get('y', 0), 2)},
                })
            self.state.current_room = label
            logger.info("New room discovered: %s (entered from %s)",
                        label, self.state.rooms[-1].get('entered_from', '?'))
        else:
            # Add object to current room
            current = self._get_current_room()
            if current is not None:
                obj_entry = f"{label}"
                if obj_entry not in current['objects']:
                    current['objects'].append(obj_entry)

        logger.info("Discovery #%d: [%s] %s — %s",
                     self.state.total_discoveries, significance, label, notes)

    # ------------------------------------------------------------------
    # Frontiers
    # ------------------------------------------------------------------

    def update_frontiers(self, frontier_list):
        """Replace frontier list with fresh data from SLAM analysis.

        Args:
            frontier_list: list of dicts with {id, direction_deg, distance_m,
                           type, width_m}.
        """
        self.state.frontiers = []
        for i, f in enumerate(frontier_list):
            self.state.frontiers.append({
                'id': f.get('id', i + 1),
                'direction_deg': round(f.get('direction_deg', 0), 1),
                'distance_m': round(f.get('distance_m', 0), 1),
                'type': f.get('type', 'unknown'),
                'width_m': round(f.get('width_m', 0), 1),
            })

    # ------------------------------------------------------------------
    # Dead ends
    # ------------------------------------------------------------------

    def record_dead_end(self, direction_deg, position, reason="blocked"):
        """Mark a direction from a position as a dead end.

        Merges with an existing dead-end if within DEAD_END_MERGE_DIST.
        """
        px = round(position.get('x', 0), 2)
        py = round(position.get('y', 0), 2)

        for de in self.state.dead_ends:
            dx = de['position']['x'] - px
            dy = de['position']['y'] - py
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < DEAD_END_MERGE_DIST and abs(de['direction_deg'] - direction_deg) < 30:
                return  # Already recorded nearby

        self.state.dead_ends.append({
            'direction_deg': round(direction_deg, 1),
            'reason': reason,
            'position': {'x': px, 'y': py},
        })
        logger.info("Dead end recorded: %.0f° at (%.1f, %.1f) — %s",
                     direction_deg, px, py, reason)

    # ------------------------------------------------------------------
    # Prompt serialisation
    # ------------------------------------------------------------------

    def to_prompt_text(self):
        """Serialise exploration state to a compact text block (~200 tokens)."""
        s = self.state
        lines = ["EXPLORATION MEMORY:"]

        # Rooms
        room_parts = []
        for r in s.rooms:
            room_parts.append(f"{r['name']} ({r['explored_pct']}% explored)")
        lines.append(f"  Rooms: {', '.join(room_parts) if room_parts else 'none'}")
        lines.append(f"  Current room: {s.current_room}")

        # Frontiers
        if s.frontiers:
            frontier_parts = []
            for f in s.frontiers[:5]:  # Cap at 5 to stay within token budget
                frontier_parts.append(
                    f"#{f['id']} at {f['direction_deg']:.0f}° ({f['type']}, "
                    f"{f['distance_m']:.1f}m, w={f['width_m']:.1f}m)"
                )
            lines.append(f"  Frontiers: {len(s.frontiers)} open — {'; '.join(frontier_parts)}")
        else:
            lines.append("  Frontiers: none detected")

        # Dead ends
        if s.dead_ends:
            de_parts = []
            for de in s.dead_ends[-5:]:  # Last 5
                de_parts.append(f"{de['direction_deg']:.0f}° ({de['reason']})")
            lines.append(f"  Dead ends: {', '.join(de_parts)}")
        else:
            lines.append("  Dead ends: none")

        # Recent actions
        if s.recent_actions:
            lines.append("  Recent:")
            for i, a in enumerate(s.recent_actions):
                idx = len(s.recent_actions) - i
                lines.append(f"    [{idx}] {a['action']}({a['params']}) -> {a['result']}")
        else:
            lines.append("  Recent: no actions yet")

        # Stats
        duration_min = s.exploration_duration_s / 60.0
        lines.append(f"  Discoveries: {s.total_discoveries} | "
                     f"Duration: {duration_min:.1f}min")

        return "\n".join(lines)

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save_to_disk(self, path):
        """Save exploration state to YAML file."""
        data = {
            'rooms': self.state.rooms,
            'current_room': self.state.current_room,
            'dead_ends': self.state.dead_ends,
            'frontiers': self.state.frontiers,
            'recent_actions': self.state.recent_actions,
            'total_discoveries': self.state.total_discoveries,
            'exploration_duration_s': self.state.exploration_duration_s,
        }
        try:
            with open(path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            logger.info("Exploration memory saved to %s", path)
        except Exception as e:
            logger.error("Failed to save exploration memory: %s", e)

    def load_from_disk(self, path):
        """Load exploration state from YAML file."""
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            if not data:
                return
            self.state.rooms = data.get('rooms', self.state.rooms)
            self.state.current_room = data.get('current_room', self.state.current_room)
            self.state.dead_ends = data.get('dead_ends', [])
            self.state.frontiers = data.get('frontiers', [])
            self.state.recent_actions = data.get('recent_actions', [])
            self.state.total_discoveries = data.get('total_discoveries', 0)
            self.state.exploration_duration_s = data.get('exploration_duration_s', 0.0)
            logger.info("Exploration memory loaded from %s", path)
        except FileNotFoundError:
            logger.info("No exploration memory file at %s — starting fresh", path)
        except Exception as e:
            logger.error("Failed to load exploration memory: %s", e)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _get_current_room(self):
        """Return the dict for the current room, or None."""
        for r in self.state.rooms:
            if r['name'] == self.state.current_room:
                return r
        return None


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------

def _summarise_params(tool_name, params):
    """One-line summary of tool parameters for the recent-actions list."""
    if tool_name == 'move_toward':
        return f"{params.get('direction', '?')}, {params.get('speed', '?')}"
    if tool_name == 'rotate':
        return f"{params.get('degrees', '?')}°"
    if tool_name == 'navigate_to_goal':
        gt = params.get('goal_type', '?')
        if gt == 'frontier':
            return f"frontier #{params.get('frontier_id', '?')}"
        return f"{gt}"
    if tool_name == 'stop_robot':
        return params.get('reason', '?')
    if tool_name == 'observe_scene':
        return params.get('focus', 'general')
    if tool_name == 'check_path_clear':
        return f"{params.get('direction', '?')}, {params.get('distance_m', '?')}m"
    if tool_name == 'report_discovery':
        return f"{params.get('label', '?')} [{params.get('significance', '?')}]"
    return str(params)[:40]


def _summarise_result(result):
    """One-line summary of an action result for the recent-actions list."""
    if not result or not isinstance(result, dict):
        return "unknown"

    success = result.get('success', False)
    parts = []

    if success:
        parts.append("success")
    else:
        parts.append("failed")

    dist = result.get('actual_distance_m')
    if dist is not None:
        parts.append(f"+{dist:.2f}m")

    if result.get('safety_override'):
        parts.append("safety_override")

    reason = result.get('stopped_reason')
    if reason:
        parts.append(reason)

    return ", ".join(parts)
