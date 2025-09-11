# From initiator namespace to responder namespace
string from_ns
string to_ns
# Time stamps in ROS time (sim time)
time t0
# arrival time at responder (unknown to initiator; for logging)
time t1
# arrival time of reply at initiator
time t2
# Measured r1 = || p1(t2) - p2(t1) || (delta assumed 0 in minimal version)
float64 r1
# Peer pose at t1 (as provided by responder)
geometry_msgs/PoseStamped peer_pose_t1
# Success flag and variance of r1
bool success
float64 variance
