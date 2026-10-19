# Atomic target state for fly_px4_1. The legacy Odometry topic remains available.
# state.header.stamp is the epoch of x/y/v, NOT the last observation timestamp.
# state.header.frame_id = world; state.child_frame_id = assignment.
nav_msgs/Odometry state
time measurement_stamp
time depth_stamp
string assignment
bool bearing_only
