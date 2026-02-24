# OpenAI-Powered Robot Intelligence Architecture

## Overview

This document outlines strategies for leveraging OpenAI's capabilities to create a more intelligent, autonomous ROSMASTER A1 robot with enhanced spatial awareness, task planning, and memory.

---

## Current State

The robot currently uses OpenAI for:
- **Voice command interpretation** via Whisper STT
- **GPT-4o as the "brain"** for natural language understanding
- **Vision analysis** of camera frames
- **TTS for speech responses**

### Limitations
- No persistent memory between sessions
- Limited spatial reasoning (no semantic map)
- Reactive rather than proactive behavior
- No long-term task planning

---

## Proposed Enhancements

### 1. Semantic Spatial Memory

**Concept**: Build a persistent "mental map" that associates locations with semantic meaning.

```python
# Spatial Memory Structure
{
    "locations": {
        "kitchen_entrance": {
            "pose": {"x": 2.5, "y": 1.2, "theta": 0.0},
            "landmarks": ["refrigerator", "counter", "sink"],
            "last_visited": "2025-11-29T10:30:00",
            "description": "Kitchen entrance with white tile floor"
        },
        "living_room_couch": {
            "pose": {"x": -1.0, "y": 3.5, "theta": 1.57},
            "landmarks": ["blue couch", "coffee table", "TV"],
            "objects_seen": ["remote control", "book", "mug"],
            "last_visited": "2025-11-29T10:25:00"
        }
    },
    "objects": {
        "remote_control": {
            "last_seen_location": "living_room_couch",
            "last_seen_time": "2025-11-29T10:25:00",
            "confidence": 0.85
        }
    }
}
```

**Implementation**:
1. Periodically capture camera frames during exploration
2. Send frames to GPT-4o Vision with prompt: "List all identifiable objects and describe the scene"
3. Associate detected objects with current odometry position
4. Store in JSON/SQLite database for persistence

**Voice Queries Enabled**:
- "Where did you last see the remote?"
- "What's in the kitchen?"
- "Take me to where you saw books"

---

### 2. Assistants API with Retrieval (RAG)

**Concept**: Use OpenAI Assistants API with file search to give the robot queryable long-term memory.

```python
from openai import OpenAI

client = OpenAI()

# Create assistant with retrieval capability
assistant = client.beta.assistants.create(
    name="Robot Memory Assistant",
    instructions="""You are the memory system for a mobile robot.
    You have access to exploration logs, object sightings, and spatial maps.
    When asked about locations or objects, search your knowledge base.""",
    model="gpt-4o",
    tools=[{"type": "file_search"}]
)

# Upload exploration logs as searchable documents
vector_store = client.beta.vector_stores.create(name="Robot Memories")
client.beta.vector_stores.files.upload_and_poll(
    vector_store_id=vector_store.id,
    file=open("exploration_log_2025_11.jsonl", "rb")
)
```

**Benefits**:
- Scales to months/years of exploration data
- Natural language queries over all memories
- OpenAI handles embedding and retrieval

---

### 3. Function Calling for Structured Actions

**Concept**: Define robot capabilities as OpenAI functions for more reliable action execution.

```python
tools = [
    {
        "type": "function",
        "function": {
            "name": "navigate_to_location",
            "description": "Navigate the robot to a named location or coordinates",
            "parameters": {
                "type": "object",
                "properties": {
                    "location_name": {
                        "type": "string",
                        "description": "Named location like 'kitchen' or 'charging_station'"
                    },
                    "coordinates": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"}
                        }
                    }
                }
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "search_for_object",
            "description": "Systematically search the environment for a specific object",
            "parameters": {
                "type": "object",
                "properties": {
                    "object_name": {"type": "string"},
                    "search_strategy": {
                        "type": "string",
                        "enum": ["last_known_location", "systematic_sweep", "likely_locations"]
                    }
                },
                "required": ["object_name"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "remember_object_location",
            "description": "Store the current location of an observed object in memory",
            "parameters": {
                "type": "object",
                "properties": {
                    "object_name": {"type": "string"},
                    "description": {"type": "string"},
                    "confidence": {"type": "number", "minimum": 0, "maximum": 1}
                },
                "required": ["object_name"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "create_patrol_route",
            "description": "Create a patrol route through specified waypoints",
            "parameters": {
                "type": "object",
                "properties": {
                    "waypoints": {
                        "type": "array",
                        "items": {"type": "string"}
                    },
                    "repeat": {"type": "boolean"},
                    "interval_minutes": {"type": "integer"}
                }
            }
        }
    }
]
```

---

### 4. Multi-Step Task Planning

**Concept**: Use GPT-4o to decompose complex tasks into executable steps.

**Example Prompt**:
```
You are a robot task planner. The user wants you to accomplish a goal.
Break it down into atomic actions the robot can perform.

Available actions:
- navigate_to(location)
- rotate(degrees)
- capture_image()
- analyze_scene()
- speak(message)
- wait(seconds)
- search_area(pattern)

Current state:
- Position: living room (x=1.2, y=3.4)
- Battery: 78%
- Known locations: kitchen, bedroom, front_door, charging_station

User goal: "Check if anyone left the stove on and come tell me"

Provide a JSON plan:
```

**Response**:
```json
{
    "plan_name": "stove_safety_check",
    "steps": [
        {"action": "speak", "params": {"message": "I'll go check the stove for you"}},
        {"action": "navigate_to", "params": {"location": "kitchen"}},
        {"action": "rotate", "params": {"degrees": 360}},
        {"action": "capture_image", "params": {}},
        {"action": "analyze_scene", "params": {"focus": "stove burners, flames, indicator lights"}},
        {"action": "navigate_to", "params": {"location": "living_room"}},
        {"action": "speak", "params": {"message": "${analysis_result}"}}
    ],
    "failure_handling": {
        "navigation_failed": "speak('I couldn't reach the kitchen, the path may be blocked')",
        "camera_failed": "speak('I'm having trouble with my camera')"
    }
}
```

---

### 5. Continuous Scene Understanding

**Concept**: Periodically analyze the environment even without explicit commands.

```python
class ProactiveIntelligence:
    def __init__(self):
        self.last_analysis_time = time.time()
        self.analysis_interval = 30  # seconds
        self.anomaly_threshold = 0.7
        
    async def background_analysis(self, frame):
        """Run periodic scene analysis"""
        prompt = """Analyze this image from a home robot's perspective.
        
        Report:
        1. Notable objects and their approximate positions
        2. Any safety concerns (obstacles, spills, fire hazards)
        3. Changes from typical home state (doors open/closed, lights)
        4. People or pets detected
        
        Format as JSON with confidence scores."""
        
        response = await client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": "You are a home monitoring robot."},
                {"role": "user", "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{frame_b64}"}}
                ]}
            ]
        )
        
        analysis = json.loads(response.choices[0].message.content)
        
        # Proactive alerts
        if analysis.get("safety_concerns"):
            self.alert_user(analysis["safety_concerns"])
            
        # Update object memory
        for obj in analysis.get("objects", []):
            self.memory.update_object_location(obj["name"], self.current_pose)
```

---

### 6. Conversational Context & Personality

**Concept**: Maintain conversation history and develop consistent personality.

```python
system_prompt = """You are ROSMASTER, an intelligent home robot assistant.

Personality traits:
- Helpful and proactive
- Curious about the environment
- Safety-conscious
- Remembers past interactions

You have spatial awareness through SLAM mapping and can navigate autonomously.
You remember where you've seen objects and can find them again.

Current knowledge:
- Map coverage: 85% of home explored
- Known rooms: {room_list}
- Recently seen objects: {recent_objects}
- Current battery: {battery}%
- Current location: {location}

When responding:
1. Consider what you know about the environment
2. Suggest proactive actions when appropriate
3. Ask clarifying questions for ambiguous requests
4. Explain your reasoning for complex tasks
"""

# Maintain conversation history
conversation_history = [
    {"role": "system", "content": system_prompt},
    # Previous exchanges preserved for context
]
```

---

### 7. Vision-Language Navigation (VLN)

**Concept**: Navigate using natural language descriptions rather than just coordinates.

```python
async def vision_language_navigate(self, instruction: str):
    """
    Navigate using natural language: 
    "Go to the room with the blue couch" or
    "Find the door with the plant next to it"
    """
    
    while not self.goal_reached:
        # Capture current view
        frame = self.get_camera_frame()
        
        prompt = f"""You are navigating a robot. The instruction is: "{instruction}"
        
        Based on this image, provide navigation guidance:
        1. Do you see the target or landmarks leading to it?
        2. Suggested action: turn_left, turn_right, go_forward, arrived
        3. Confidence (0-1)
        4. Reasoning
        
        Respond in JSON format."""
        
        response = await self.query_gpt4_vision(frame, prompt)
        guidance = json.loads(response)
        
        if guidance["action"] == "arrived":
            self.speak(f"I've arrived. {guidance['reasoning']}")
            break
            
        self.execute_navigation_action(guidance["action"])
```

---

### 8. Learning from Corrections

**Concept**: Remember user corrections and improve over time.

```python
class LearningMemory:
    def __init__(self):
        self.corrections_db = "corrections.json"
        
    def record_correction(self, context, wrong_action, correct_action, feedback):
        """Store user corrections for future reference"""
        correction = {
            "timestamp": datetime.now().isoformat(),
            "context": context,  # What was happening
            "wrong_action": wrong_action,
            "correct_action": correct_action,
            "user_feedback": feedback
        }
        self.corrections.append(correction)
        self.save()
        
    def get_relevant_corrections(self, current_context):
        """Retrieve past corrections relevant to current situation"""
        # Use embeddings to find similar past situations
        return self.semantic_search(current_context, self.corrections)

# In system prompt, include recent corrections:
"""
Past corrections to remember:
- When asked to "go to the kitchen", user corrected that the kitchen is through the hallway, not the living room
- User prefers robot to announce before moving
- The "office" and "study" refer to the same room
"""
```

---

## Implementation Roadmap

### Phase 1: Enhanced Memory (Week 1-2)
- [ ] Implement persistent JSON-based spatial memory
- [ ] Add object detection logging during exploration
- [ ] Create "where is X?" query capability

### Phase 2: Function Calling (Week 3-4)
- [ ] Define comprehensive tool schema
- [ ] Refactor action execution to use function calls
- [ ] Add structured error handling

### Phase 3: Assistants API Integration (Week 5-6)
- [ ] Set up Vector Store for exploration logs
- [ ] Create retrieval-augmented queries
- [ ] Implement long-term memory persistence

### Phase 4: Proactive Intelligence (Week 7-8)
- [ ] Background scene analysis
- [ ] Anomaly detection and alerts
- [ ] Predictive task suggestions

### Phase 5: Task Planning (Week 9-10)
- [ ] Multi-step plan generation
- [ ] Plan execution engine
- [ ] Failure recovery

---

## Cost Considerations

| Feature | API Calls | Estimated Cost/Day |
|---------|-----------|-------------------|
| Voice commands (50/day) | GPT-4o text | ~$0.50 |
| Vision analysis (100 frames/day) | GPT-4o vision | ~$2.00 |
| Continuous monitoring (720 frames/day @ 30s) | GPT-4o vision | ~$15.00 |
| Assistants API retrieval | Retrieval + GPT-4o | ~$1.00 |

**Optimization strategies**:
- Use GPT-4o-mini for simple queries
- Cache repeated scene analyses
- Rate-limit background analysis
- Use local models (Ollama) for routine checks, GPT-4o for complex reasoning

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROSMASTER A1 Intelligence Stack               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │   Whisper    │    │   GPT-4o     │    │   GPT-4o     │       │
│  │    STT       │───▶│   Brain      │◀──▶│   Vision     │       │
│  └──────────────┘    └──────┬───────┘    └──────────────┘       │
│                             │                                    │
│                    ┌────────▼────────┐                          │
│                    │  Function Call  │                          │
│                    │    Router       │                          │
│                    └────────┬────────┘                          │
│         ┌──────────────────┼──────────────────┐                 │
│         ▼                  ▼                  ▼                 │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐           │
│  │ Navigation  │   │   Memory    │   │   Task      │           │
│  │  Actions    │   │   System    │   │  Planner    │           │
│  └──────┬──────┘   └──────┬──────┘   └──────┬──────┘           │
│         │                 │                 │                   │
│         ▼                 ▼                 ▼                   │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐           │
│  │   Nav2      │   │  Assistants │   │   Plan      │           │
│  │  Stack      │   │  Vector DB  │   │  Executor   │           │
│  └─────────────┘   └─────────────┘   └─────────────┘           │
│                                                                  │
├─────────────────────────────────────────────────────────────────┤
│                        ROS2 Layer                                │
│  /cmd_vel  /scan  /odom  /map  /camera/image  /tf               │
└─────────────────────────────────────────────────────────────────┘
```

---

### 9. SLAM-Integrated Room Naming & Recall

**Concept**: When SLAM discovers a new area, prompt the user to name it. Store room boundaries with semantic labels for natural navigation.

```python
class SemanticSLAMManager:
    def __init__(self):
        self.rooms_db = "rooms.json"  # Persistent room storage
        self.rooms = self.load_rooms()
        self.frontier_threshold = 2.0  # meters of new area = potential room
        self.last_map_area = 0
        
    def load_rooms(self) -> dict:
        """Load room definitions from persistent storage"""
        if os.path.exists(self.rooms_db):
            with open(self.rooms_db) as f:
                return json.load(f)
        return {"rooms": {}, "unnamed_areas": []}
    
    def save_rooms(self):
        with open(self.rooms_db, 'w') as f:
            json.dump(self.rooms, f, indent=2)
    
    def on_map_update(self, occupancy_grid):
        """Called when SLAM map updates - detect new areas"""
        current_area = self.calculate_explored_area(occupancy_grid)
        new_area = current_area - self.last_map_area
        
        if new_area > self.frontier_threshold:
            # Significant new area discovered
            centroid = self.get_new_area_centroid(occupancy_grid)
            self.prompt_for_room_name(centroid, new_area)
            
        self.last_map_area = current_area
    
    async def prompt_for_room_name(self, centroid: tuple, area: float):
        """Ask user to name the newly discovered area"""
        # Take a photo of the new area
        frame = self.capture_frame()
        
        # Get GPT-4o's suggestion based on visual analysis
        suggestion = await self.get_room_suggestion(frame)
        
        self.speak(f"I've discovered a new area, about {area:.1f} square meters. "
                   f"It looks like a {suggestion}. What would you like to call this room?")
        
        # Wait for user response
        user_name = await self.listen_for_response(timeout=30)
        
        if user_name:
            self.register_room(user_name, centroid, area, frame)
            self.speak(f"Got it! I'll remember this as the {user_name}.")
        else:
            # Store as unnamed for later
            self.rooms["unnamed_areas"].append({
                "centroid": centroid,
                "area": area,
                "discovered": datetime.now().isoformat(),
                "suggestion": suggestion
            })
    
    async def get_room_suggestion(self, frame) -> str:
        """Use GPT-4o Vision to suggest room type"""
        response = await client.chat.completions.create(
            model="gpt-4o",
            messages=[{
                "role": "user",
                "content": [
                    {"type": "text", "text": "What type of room is this? Reply with just the room type (e.g., 'kitchen', 'bedroom', 'hallway', 'bathroom', 'living room', 'office')."},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{self.encode_frame(frame)}"}}
                ]
            }],
            max_tokens=50
        )
        return response.choices[0].message.content.strip().lower()
    
    def register_room(self, name: str, centroid: tuple, area: float, frame=None):
        """Register a named room in the semantic map"""
        self.rooms["rooms"][name.lower()] = {
            "centroid": {"x": centroid[0], "y": centroid[1]},
            "area_sqm": area,
            "created": datetime.now().isoformat(),
            "last_visited": datetime.now().isoformat(),
            "aliases": [],  # Alternative names
            "objects_seen": [],
            "notes": ""
        }
        self.save_rooms()
    
    def get_room_at_position(self, x: float, y: float) -> str:
        """Find which room contains a given position"""
        for name, room in self.rooms["rooms"].items():
            cx, cy = room["centroid"]["x"], room["centroid"]["y"]
            # Simple distance check (could use polygon boundaries)
            if math.sqrt((x - cx)**2 + (y - cy)**2) < 2.0:
                return name
        return "unknown area"
    
    def get_room_position(self, room_name: str) -> tuple:
        """Get navigation target for a room name"""
        name = room_name.lower()
        
        # Check exact match
        if name in self.rooms["rooms"]:
            c = self.rooms["rooms"][name]["centroid"]
            return (c["x"], c["y"])
        
        # Check aliases
        for rname, room in self.rooms["rooms"].items():
            if name in [a.lower() for a in room.get("aliases", [])]:
                c = room["centroid"]
                return (c["x"], c["y"])
        
        # Fuzzy match
        for rname in self.rooms["rooms"]:
            if name in rname or rname in name:
                c = self.rooms["rooms"][rname]["centroid"]
                return (c["x"], c["y"])
        
        return None
    
    def add_room_alias(self, room_name: str, alias: str):
        """Add alternative name for a room"""
        if room_name.lower() in self.rooms["rooms"]:
            self.rooms["rooms"][room_name.lower()]["aliases"].append(alias)
            self.save_rooms()
    
    def list_known_rooms(self) -> list:
        """Get all known room names"""
        return list(self.rooms["rooms"].keys())
```

**Integration with Voice Commands**:

```python
# In the GPT-4o system prompt, include room knowledge:
system_prompt = f"""
Known rooms in this environment:
{self.slam_manager.list_known_rooms()}

When the user asks to go somewhere:
1. Match their request to a known room name or alias
2. If no match, ask for clarification
3. Navigate to the room's centroid position

When exploring new areas:
1. Announce discovery of unmapped space
2. Ask the user to name significant new areas
3. Remember object locations within each room
"""

# Function definitions for room operations
tools = [
    {
        "type": "function",
        "function": {
            "name": "navigate_to_room",
            "description": "Navigate to a named room in the mapped environment",
            "parameters": {
                "type": "object",
                "properties": {
                    "room_name": {
                        "type": "string",
                        "description": "Name of the room to navigate to"
                    }
                },
                "required": ["room_name"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "name_current_location",
            "description": "Assign a name to the robot's current location/room",
            "parameters": {
                "type": "object",
                "properties": {
                    "name": {"type": "string"},
                    "aliases": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": "Alternative names for this location"
                    }
                },
                "required": ["name"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "list_rooms",
            "description": "List all named rooms the robot knows about",
            "parameters": {"type": "object", "properties": {}}
        }
    },
    {
        "type": "function", 
        "function": {
            "name": "get_room_info",
            "description": "Get information about a specific room",
            "parameters": {
                "type": "object",
                "properties": {
                    "room_name": {"type": "string"}
                },
                "required": ["room_name"]
            }
        }
    }
]
```

**Room Data Persistence** (`rooms.json`):

```json
{
    "rooms": {
        "kitchen": {
            "centroid": {"x": 3.5, "y": 2.1},
            "area_sqm": 12.5,
            "created": "2025-11-29T10:30:00",
            "last_visited": "2025-11-29T14:22:00",
            "aliases": ["cooking area"],
            "objects_seen": ["refrigerator", "stove", "coffee maker", "fruit bowl"],
            "notes": "User mentioned they usually have breakfast here"
        },
        "master bedroom": {
            "centroid": {"x": -2.0, "y": 5.5},
            "area_sqm": 18.0,
            "created": "2025-11-29T10:45:00",
            "last_visited": "2025-11-29T11:00:00",
            "aliases": ["bedroom", "main bedroom"],
            "objects_seen": ["bed", "nightstand", "lamp", "closet"],
            "notes": ""
        },
        "garage": {
            "centroid": {"x": 8.0, "y": 0.0},
            "area_sqm": 25.0,
            "created": "2025-11-29T11:15:00",
            "last_visited": "2025-11-29T11:15:00",
            "aliases": ["car port"],
            "objects_seen": ["car", "toolbox", "bicycle", "storage boxes"],
            "notes": "Concrete floor, slightly sloped"
        }
    },
    "unnamed_areas": [
        {
            "centroid": [1.2, -3.0],
            "area": 4.5,
            "discovered": "2025-11-29T12:00:00",
            "suggestion": "hallway"
        }
    ],
    "map_metadata": {
        "total_area_sqm": 85.0,
        "last_updated": "2025-11-29T14:22:00",
        "slam_map_file": "~/maps/home_2025_11_29.yaml"
    }
}
```

**Example Voice Interactions**:

> **Robot** (during exploration): "I've found a new area, about 12 square meters. It looks like a kitchen with appliances. What would you like me to call this room?"
> 
> **User**: "That's the kitchen"
> 
> **Robot**: "Got it! I'll remember this as the kitchen."

---

> **User**: "Go to the kitchen"
> 
> **Robot**: "Navigating to the kitchen." *(uses Nav2 to reach stored centroid)*

---

> **User**: "What rooms do you know?"
> 
> **Robot**: "I know 5 rooms: the kitchen, living room, master bedroom, bathroom, and garage. There's also one unnamed area near the front that looks like a hallway."

---

> **User**: "That hallway is actually the mudroom"
> 
> **Robot**: "Thanks! I've renamed that area to mudroom."

---

> **User**: "Where's the bedroom?"
> 
> **Robot**: "Do you mean the master bedroom or the guest bedroom? I know both."

---

## Sample Enhanced Voice Interactions

**Today** (Basic):
> "Go forward" → Robot moves forward
> "What do you see?" → Robot describes current view

**Future** (Intelligent):
> "I lost my keys, can you help me find them?"
> → "I last saw keys on the kitchen counter yesterday at 3pm. I also remember seeing them near the front door last week. Should I check those locations?"

> "Make sure the house is secure before I go to bed"
> → Robot creates and executes a plan: check doors, verify windows, turn off stove, report back

> "Remember that this is where we keep the dog food"
> → "Got it! I've noted that dog food is stored in the pantry, lower shelf on the left."

> "What changed in the living room since this morning?"
> → "I notice the couch pillows are rearranged, and there's a new box by the TV stand. Also, the blinds are now closed."

---

## References

- [OpenAI Function Calling Guide](https://platform.openai.com/docs/guides/function-calling)
- [Assistants API with File Search](https://platform.openai.com/docs/assistants/tools/file-search)
- [GPT-4 Vision](https://platform.openai.com/docs/guides/vision)
- [ROS2 Nav2 Documentation](https://navigation.ros.org/)
