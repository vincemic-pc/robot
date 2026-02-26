# VelocityRequest.msg — Command to the velocity arbiter
#
# All velocity commands flow through this message. The arbiter selects
# the highest-priority active request and applies safety filtering.

std_msgs/Header header

# Desired velocity (before safety filtering and PID correction)
geometry_msgs/Twist twist

# Priority level: lower number = higher priority
# 0=EMERGENCY, 1=SAFETY, 2=NAVIGATION, 3=EXPLORATION, 4=MANUAL
uint8 priority

# Priority constants
uint8 PRIORITY_EMERGENCY=0
uint8 PRIORITY_SAFETY=1
uint8 PRIORITY_NAVIGATION=2
uint8 PRIORITY_EXPLORATION=3
uint8 PRIORITY_MANUAL=4

# Identifier of the requesting source (for diagnostics/debugging)
string source

# How long this request remains valid (seconds). 0 = single-shot (one cycle only).
# Arbiter publishes zero Twist when the active request expires.
float32 duration_s

# Target heading in radians (world frame). NaN = no heading control.
# When set, the arbiter uses heading PID to control angular velocity.
float32 target_heading

# Whether velocity PID should be applied to this request.
# When true, arbiter compares commanded vs measured velocity and applies PID correction.
bool use_velocity_pid true
