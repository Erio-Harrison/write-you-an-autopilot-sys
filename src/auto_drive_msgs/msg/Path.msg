# Header for timestamp and frame information
std_msgs/Header header

# An array of poses that represent the path
geometry_msgs/PoseStamped[] poses

# Optionally, you might want to include:
# Path length
float64 length

# Estimated time to traverse the path
builtin_interfaces/Duration estimated_time

# Path status or type (e.g., global path, local path)
uint8 path_type