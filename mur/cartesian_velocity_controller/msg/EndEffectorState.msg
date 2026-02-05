# End Effector complete state message
# Contains position, velocity, acceleration, jerk, orientation and angular velocities

std_msgs/Header header

# Position in global frame [m]
geometry_msgs/Point position

# Linear velocity [m/s]
geometry_msgs/Vector3 linear_velocity

# Linear acceleration [m/s^2]
geometry_msgs/Vector3 linear_acceleration

# Linear jerk [m/s^3]
geometry_msgs/Vector3 linear_jerk

# Orientation as quaternion
geometry_msgs/Quaternion orientation

# Orientation as RPY angles [rad]
geometry_msgs/Vector3 orientation_rpy

# Angular velocity [rad/s]
geometry_msgs/Vector3 angular_velocity

# Angular acceleration [rad/s^2]
geometry_msgs/Vector3 angular_acceleration

# Angular jerk [rad/s^3]
geometry_msgs/Vector3 angular_jerk

