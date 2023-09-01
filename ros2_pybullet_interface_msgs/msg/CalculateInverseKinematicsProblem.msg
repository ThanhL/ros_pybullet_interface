std_msgs/Header header
string link_name
float64[] target_position
float64[] target_orientation
float64[] lower_limits
float64[] upper_limits
float64[] joint_ranges
float64[] reset_poses
float64[] joint_damping
int64 solver  # use 0 for p.IK_DLS or 1 for p.IK_SDLS
float64[] current_position
int64 max_num_iterations
float64 residual_threshold

float64 dt  # used to calculate joint velocity (vel=(solution-currentPosition)/dt), this must be given if robot is in velocity control