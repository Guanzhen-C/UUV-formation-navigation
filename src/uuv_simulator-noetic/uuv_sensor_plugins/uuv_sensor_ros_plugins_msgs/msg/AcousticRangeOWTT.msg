string from_ns
string to_ns
# Transmit time at sender and receive time at receiver (ROS sim time)
time t_tx
time t_rx

# One-way measured range (meters) and its variance
float64 range
float64 variance

# Transmitter's position estimate and covariance at t_tx
uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped tx_pos

# Cross-covariance between sender 15-dim error state and its position (15x3, row-major)
float64[45] tx_cross_cov_x_p


