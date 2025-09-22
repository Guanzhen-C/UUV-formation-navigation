string from_ns
# Transmit time at sender (ROS sim time)
time t_tx

# Transmitter's position estimate and covariance at t_tx
uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped tx_pos

# Cross-covariance between transmitter 15-dim error state and its position (15x3, row-major)
float64[45] tx_cross_cov_x_p

