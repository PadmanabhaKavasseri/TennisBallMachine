# Motor identification
string motor              # "stepper", "top_wheel", "bottom_wheel", "linear_actuator"

# Command type
string command            # "move_steps", "move_angle", "set_rpm", "set_percent", "home", "enable", "disable", "stop"

# Value (meaning depends on command)
float64 value            # Steps, angle, RPM, percentage, etc.

# Optional parameters
string direction         # "cw", "ccw" (for stepper steps)
float64 speed           # Speed parameter if needed
