# RobotStatus.msg

# Define the RobotStatus message type
# This message contains the status of the robot, including its current state and battery level.

float32 battery_level  # Battery level as a percentage (0 to 100)
string current_state   # Current state of the robot (e.g., "idle", "moving", "charging")
bool is_operational     # Indicates if the robot is operational (true) or not (false)
string error_message    # Error message if the robot encounters an issue, otherwise empty