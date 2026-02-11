# Policy output containing high-level manipulation commands
# This message represents the output from the VLA policy model

std_msgs/Header header

# End-effector target pose (optional, depends on policy output format)
geometry_msgs/Pose eef_target_pose
bool has_eef_target

# Joint angle deltas (optional alternative to pose-based control)
float64[] joint_deltas
bool has_joint_deltas

# Gripper command (normalized)
float64 gripper_command  # 0.0 = closed, 1.0 = open
bool gripper_active

# Optional base motion hint from policy
geometry_msgs/Twist base_velocity_hint
bool has_base_hint

# Reference frame for the target (e.g., "base_link", "camera_link")
string reference_frame

# Confidence score from policy (if available)
float32 confidence
