# Unique identifier for the obstacle
uint32 id

# 3D position of the obstacle
geometry_msgs/Point position

# 3D dimensions of the obstacle (assuming a rectangular shape)
geometry_msgs/Vector3 dimensions

# Velocity of the obstacle (if it's moving)
geometry_msgs/Vector3 velocity

# Type of obstacle (e.g., car, pedestrian, static object)
uint8 type

# Timestamp when this obstacle was detected
builtin_interfaces/Time stamp

# Confidence level of the detection (0.0 to 1.0)
float32 confidence