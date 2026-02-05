# Joint Velocity Feedback message
# Compares commanded velocity vs actual robot velocity

std_msgs/Header header

# Joint names in order
string[] joint_names

# ========== Commanded Velocity ==========
# Velocity command sent to the robot [rad/s]
float64[] commanded_velocity


# ========== Actual Velocity ==========
# Actual velocity from joint states [rad/s]
float64[] actual_velocity


# ========== Position Info ==========
# Current joint positions [rad]
float64[] current_position



