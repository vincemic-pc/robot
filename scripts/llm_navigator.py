"""LLM Navigator for autonomous exploration.

Builds prompts, calls OpenAI GPT-4o-mini with tool/function calling, and
parses navigation decisions.  Stateless per call -- no growing chat history.

Plan 006: LLM-Driven Autonomous Exploration Navigation.
"""

import time
import json
import logging
from dataclasses import dataclass
from typing import Optional

from openai import OpenAI

logger = logging.getLogger(__name__)

# Model and timeout constants
MODEL = "gpt-4o-mini"
API_TIMEOUT = 8  # seconds


@dataclass
class NavigationDecision:
    """Parsed result from a VLM tool call."""
    tool_name: str              # "move_toward", "rotate", etc.
    parameters: dict            # Tool-specific args
    reasoning: str              # VLM's chain-of-thought text (logged, not executed)
    timestamp: float
    raw_response: Optional[dict] = None


# ---------------------------------------------------------------------------
# Tool Definitions (OpenAI function calling, strict: true)
# ---------------------------------------------------------------------------

_DIRECTIONS = ["forward", "forward_left", "left", "forward_right", "right", "backward"]


def _tool(name, desc, props, required):
    """Build one OpenAI function-calling tool definition."""
    return {
        "type": "function",
        "function": {
            "name": name, "description": desc, "strict": True,
            "parameters": {
                "type": "object", "properties": props,
                "required": required, "additionalProperties": False,
            },
        },
    }


TOOL_DEFINITIONS = [
    _tool("move_toward",
          "Move the robot in a direction at a specified speed. "
          "Use for short-range navigation (under 2m). "
          "For longer distances, use navigate_to_goal.",
          {
              "direction": {
                  "type": "string", "enum": _DIRECTIONS,
                  "description": "Direction relative to current heading",
              },
              "speed": {
                  "type": "string", "enum": ["slow", "medium", "fast"],
                  "description": "slow=0.08 m/s (near obstacles), "
                                 "medium=0.12 m/s (normal), fast=0.18 m/s (open)",
              },
              "duration_s": {
                  "type": "number",
                  "description": "Duration in seconds (1.0-8.0)",
              },
          },
          ["direction", "speed", "duration_s"]),

    _tool("navigate_to_goal",
          "Delegate to Nav2 path planner for longer distances (>2m). "
          "Robot autonomously plans a collision-free path.",
          {
              "goal_type": {
                  "type": "string", "enum": ["frontier", "coordinates", "retrace"],
                  "description": "frontier: go to frontier by ID. "
                                 "coordinates: go to (x,y). retrace: return to start.",
              },
              "frontier_id": {
                  "type": "integer",
                  "description": "Frontier ID (when goal_type=frontier)",
              },
              "x": {"type": "number", "description": "Map x in metres (when coordinates)"},
              "y": {"type": "number", "description": "Map y in metres (when coordinates)"},
              "reason": {"type": "string", "description": "Why this goal"},
          },
          ["goal_type", "reason"]),

    _tool("rotate",
          "Turn the robot (Ackerman-adapted arc turn with minimal forward motion). "
          "Negative = left, positive = right.",
          {
              "degrees": {
                  "type": "integer",
                  "enum": [-180, -135, -90, -45, 45, 90, 135, 180],
                  "description": "Rotation in degrees. Negative=left, positive=right.",
              },
          },
          ["degrees"]),

    _tool("stop_robot",
          "Intentionally halt the robot. Use when uncertain or done exploring.",
          {
              "reason": {
                  "type": "string",
                  "enum": ["obstacle_detected", "exploration_complete",
                           "need_to_observe", "user_requested", "uncertain"],
                  "description": "Why the robot should stop",
              },
          },
          ["reason"]),

    _tool("observe_scene",
          "Request detailed visual analysis using the high-quality vision model. "
          "Robot stops and looks carefully. Use when you need more detail.",
          {
              "focus": {
                  "type": "string",
                  "enum": ["general", "doorways", "obstacles", "objects", "floor_surface"],
                  "description": "What to focus the observation on",
              },
          },
          ["focus"]),

    _tool("check_path_clear",
          "Query LiDAR to check if a direction is traversable. "
          "Instant local sensor check, no API call.",
          {
              "direction": {
                  "type": "string", "enum": _DIRECTIONS,
                  "description": "Direction to check relative to heading",
              },
              "distance_m": {
                  "type": "number",
                  "description": "How far to check in metres (0.5-5.0)",
              },
          },
          ["direction", "distance_m"]),

    _tool("report_discovery",
          "Announce and log a finding. Use significance='room' when entering a new "
          "room -- the label becomes the room name.",
          {
              "label": {
                  "type": "string",
                  "description": "What was found (e.g. 'kitchen', 'bookshelf')",
              },
              "notes": {
                  "type": "string",
                  "description": "Brief description",
              },
              "significance": {
                  "type": "string",
                  "enum": ["landmark", "room", "obstacle", "point_of_interest"],
                  "description": "Category of discovery",
              },
          },
          ["label", "notes", "significance"]),
]

VALID_TOOL_NAMES = {t["function"]["name"] for t in TOOL_DEFINITIONS}


# ---------------------------------------------------------------------------
# System Prompt (static, ~260 tokens)
# ---------------------------------------------------------------------------

SYSTEM_PROMPT = """\
You are the autonomous navigation intelligence for an indoor exploration robot \
(ROSMASTER A1, Ackerman steering). You continuously perceive the environment through \
a camera, LiDAR, and depth sensors, and decide where to explore next. Your goal is \
to systematically discover and map all accessible rooms and areas.

Each decision cycle, you receive:
- An annotated camera image (RGB with depth values at 7 points and heading indicator)
- A 12-sector LiDAR summary with distances and qualitative labels (CLEAR/NEAR/OBSTACLE/WALL)
- Doorway/gap detections from LiDAR analysis
- Your current position, heading, speed, and exploration statistics
- The result of your previous action (success/failure, safety overrides)
- Your exploration memory (rooms discovered, frontiers, dead ends)

EXPLORATION STRATEGY:
1. Prioritize unexplored areas -- seek doorways, openings, and paths to new rooms
2. Be systematic -- don't revisit explored areas unless seeking a missed path
3. When you see a doorway or opening, investigate it -- doorways lead to new rooms
4. In open spaces, sweep the perimeter before crossing the center
5. If stuck or in a dead end, backtrack to the last junction and try a different direction
6. Announce significant discoveries (new rooms, interesting objects, layout features)
7. Maintain awareness of your return path

SAFETY CONSTRAINTS (non-negotiable):
- Maximum speed: 0.18 m/s. Use slow (0.08) near obstacles, medium (0.12) normally.
- A safety layer monitors LiDAR at 10 Hz and WILL override your commands if too close.
- If your action was safety-overridden, do NOT retry the same direction. Choose another path.
- Never push through narrow gaps (<0.5m width) -- the robot is 0.25m wide.
- Ackerman steering: the robot cannot spin in place. All turns require forward motion.
- If uncertain, call stop_robot or check_path_clear before moving.

REASONING (required):
Before each action, briefly state:
1. What you PERCEIVE (key observations from camera + LiDAR)
2. What you REASON (why this action advances exploration)
Keep reasoning to 1-3 sentences. Be concise."""


# ---------------------------------------------------------------------------
# LLM Navigator
# ---------------------------------------------------------------------------

class LLMNavigator:
    """Builds prompts, calls OpenAI, and parses tool-call navigation decisions."""

    def __init__(self, openai_client: OpenAI):
        self.client = openai_client

    def _build_system_prompt(self):
        return SYSTEM_PROMPT

    def _build_user_message(self, snapshot, memory, skip_image=False):
        """Build dynamic user message with sensor context and optional image.

        Returns list of content parts for the OpenAI user message.
        """
        rs = snapshot.robot_state
        lines = []

        # Robot state
        lines.append("CURRENT STATE:")
        lines.append(
            f"  Position: ({rs['x']}, {rs['y']}) metres from start | "
            f"Heading: {rs['heading_deg']:.0f} deg ({rs['heading_cardinal']})"
        )
        lines.append(
            f"  Speed: {rs['speed_mps']:.2f} m/s | "
            f"Distance traveled: {rs['distance_traveled_m']:.1f}m | "
            f"Exploration time: {rs['explore_time_s']:.0f}s"
        )

        # LiDAR
        lines.append("")
        lines.append(snapshot.lidar_summary)

        # Doorway gaps
        if snapshot.doorway_gaps:
            gap_parts = [
                f"{g['direction_deg']:.0f} deg, width {g['width_m']:.1f}m, "
                f"dist {g['distance_m']:.1f}m"
                for g in snapshot.doorway_gaps[:3]
            ]
            lines.append(f"  Gaps: {'; '.join(gap_parts)}")

        # Affordance scores
        lines.append("")
        aff = ", ".join(f"{d}={s:.1f}" for d, s in snapshot.affordance_scores.items())
        lines.append(f"AFFORDANCES: {aff}")

        # Previous action result
        if snapshot.previous_action_result:
            lines.append("")
            lines.append(f"PREVIOUS ACTION RESULT: "
                         f"{_format_action_result(snapshot.previous_action_result)}")

        # Exploration memory
        lines.append("")
        lines.append(memory.to_prompt_text())

        parts = [{"type": "text", "text": "\n".join(lines)}]

        # Camera image (optional)
        if not skip_image and snapshot.annotated_image_b64:
            parts.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{snapshot.annotated_image_b64}",
                    "detail": "low",
                },
            })

        return parts

    def decide(self, snapshot, memory, skip_image=False):
        """Full VLM decision cycle: build prompt -> call API -> parse response.

        Returns NavigationDecision on success, or None on failure.
        """
        messages = [
            {"role": "system", "content": self._build_system_prompt()},
            {"role": "user", "content": self._build_user_message(
                snapshot, memory, skip_image=skip_image)},
        ]

        try:
            t0 = time.time()
            response = self.client.chat.completions.create(
                model=MODEL,
                messages=messages,
                tools=TOOL_DEFINITIONS,
                tool_choice="required",
                max_tokens=300,
                temperature=0.4,
            )
            latency = time.time() - t0

            decision = self._parse_tool_call(response)
            if decision is not None:
                logger.info(
                    "VLM decision (%.1fs): %s(%s) -- %s",
                    latency, decision.tool_name,
                    json.dumps(decision.parameters),
                    decision.reasoning[:120] if decision.reasoning else "no reasoning",
                )
            else:
                logger.warning("VLM returned no valid tool call (%.1fs)", latency)
            return decision

        except Exception as e:
            logger.error("VLM API error: %s", e)
            return None

    def _parse_tool_call(self, response):
        """Extract tool name + args from an OpenAI chat completion response."""
        try:
            choice = response.choices[0]
            message = choice.message
            reasoning = message.content or ""

            if not message.tool_calls or len(message.tool_calls) == 0:
                logger.warning("No tool_calls in VLM response")
                return None

            tc = message.tool_calls[0]
            tool_name = tc.function.name
            try:
                parameters = json.loads(tc.function.arguments)
            except json.JSONDecodeError:
                logger.error("Failed to parse tool arguments: %s", tc.function.arguments)
                return None

            if tool_name not in VALID_TOOL_NAMES:
                logger.error("Unknown tool name from VLM: %s", tool_name)
                return None

            raw = {
                'model': response.model,
                'tool_name': tool_name,
                'arguments': parameters,
                'reasoning': reasoning,
                'finish_reason': choice.finish_reason,
                'usage': {
                    'prompt_tokens': response.usage.prompt_tokens,
                    'completion_tokens': response.usage.completion_tokens,
                    'total_tokens': response.usage.total_tokens,
                } if response.usage else None,
            }

            return NavigationDecision(
                tool_name=tool_name,
                parameters=parameters,
                reasoning=reasoning,
                timestamp=time.time(),
                raw_response=raw,
            )
        except (IndexError, AttributeError) as e:
            logger.error("Failed to parse VLM response: %s", e)
            return None


def _format_action_result(result):
    """Format a previous action result dict into a human-readable string."""
    if not result:
        return "none"
    parts = []
    parts.append("success" if result.get('success') else "FAILED")
    dist = result.get('actual_distance_m')
    if dist is not None:
        parts.append(f"moved {dist:.2f}m")
    if result.get('safety_override'):
        parts.append(result.get('safety_message', 'safety override triggered'))
    reason = result.get('stopped_reason')
    if reason:
        parts.append(f"stopped: {reason}")
    return ", ".join(parts)
