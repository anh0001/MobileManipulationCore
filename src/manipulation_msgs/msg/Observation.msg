# Aggregated sensor observation for policy inference

std_msgs/Header header

# RGB image (processed to model input size)
sensor_msgs/Image rgb_image
bool has_rgb_image

# Depth image (processed to model input size)
sensor_msgs/Image depth_image
bool has_depth_image

# Robot joint states
sensor_msgs/JointState joint_states
bool has_joint_states

# Transforms in the base frame
geometry_msgs/TransformStamped base_to_camera
bool has_base_to_camera
geometry_msgs/TransformStamped base_to_ee
bool has_base_to_ee

# Frame identifiers for the observation
string base_frame
string camera_frame
string ee_frame
