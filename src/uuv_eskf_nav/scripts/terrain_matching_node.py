#!/usr/bin/env python3
import rospy
import numpy as np
import tf
import tf.transformations as trans
from sensor_msgs.msg import LaserScan, PointCloud2, FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from terrain_map_server import TerrainMapServer
import threading

ATM_PRESSURE_KPA = 101.325
KPA_PER_METER = 9.80638

class ParticleFilter:
    def __init__(self, num_particles, initial_pose, initial_cov, map_server):
        self.N = num_particles
        self.map = map_server
        
        # State: [x, y] (N x 2 matrix)
        # We assume Z is known from pressure, and Orientation is known from IMU.
        self.particles = np.zeros((self.N, 2))
        self.weights = np.ones(self.N) / self.N
        
        # Initialize particles around initial pose
        self.particles[:, 0] = np.random.normal(initial_pose[0], initial_cov[0], self.N)
        self.particles[:, 1] = np.random.normal(initial_pose[1], initial_cov[1], self.N)
        
        # Process Noise (Motion Model noise)
        self.process_noise = [0.5, 0.5] # meters std dev per step
        
        # Measurement Noise (Sensor + Map mismatch)
        self.sigma_z = 2.0 # meters

    def predict(self, dx, dy):
        """
        Move particles based on DVL/Odom delta
        """
        noise_x = np.random.normal(0, self.process_noise[0], self.N)
        noise_y = np.random.normal(0, self.process_noise[1], self.N)
        
        self.particles[:, 0] += dx + noise_x
        self.particles[:, 1] += dy + noise_y

    def update(self, observation_points_body, current_depth, R_world_body):
        """
        Weight particles based on bathymetry match.
        
        observation_points_body: (M, 3) array of points in BODY frame (x, y, z)
        current_depth: AUV depth (negative z) from pressure sensor
        R_world_body: (3, 3) Rotation matrix from IMU
        """
        if len(observation_points_body) == 0:
            return

        # 1. Rotate observation points to World Orientation (but still centered at 0,0)
        #    We only need the relative offsets.
        #    P_w_relative = R_wb * P_b
        obs_rotated = np.dot(observation_points_body, R_world_body.T) # (M, 3)
        
        # Pre-calculate observed absolute depths (Z) for all points
        # Z_meas = Z_auv + Z_relative
        z_meas = current_depth + obs_rotated[:, 2] # (M,) array

        # Pre-calculate relative X, Y offsets
        dx_obs = obs_rotated[:, 0] # (M,)
        dy_obs = obs_rotated[:, 1] # (M,)

        # 2. For each particle, check map match
        # OPTIMIZATION: We can't easily vectorize the map query if it uses bilinear interp per point.
        # But we can loop over particles.
        
        total_weight = 0.0
        
        for i in range(self.N):
            px, py = self.particles[i]
            
            # Construct query coordinates for all beam points
            # Qx = Px + dx_obs
            qx = px + dx_obs
            qy = py + dy_obs
            
            # Query Map Heights (Batch query would be better, but map server is simple now)
            # Calculating Mean Squared Error (MSE) or Mean Absolute Error (MAE)
            error_sum = 0.0
            valid_points = 0
            
            # Sampling: To speed up, maybe don't use ALL points if M is huge (e.g. > 1000)
            # Using a stride
            step = max(1, len(z_meas) // 50) # Target ~50 points
            
            for k in range(0, len(z_meas), step):
                # Check if query is inside map bounds
                z_map = self.map.get_elevation(qx[k], qy[k])
                
                # Simple outlier rejection (if z_map is default/nodata)
                if z_map > -9000: 
                    diff = z_meas[k] - z_map
                    error_sum += diff * diff
                    valid_points += 1
            
            if valid_points > 0:
                mse = error_sum / valid_points
                # Gaussian likelihood
                lik = np.exp(-mse / (2 * self.sigma_z * self.sigma_z))
                self.weights[i] *= lik + 1e-30 # Avoid zero
            else:
                # Particle out of map bounds
                self.weights[i] *= 1e-30
                
            total_weight += self.weights[i]

        # Normalize
        self.weights /= total_weight
        
        # Calculate Effective Sample Size (N_eff)
        n_eff = 1.0 / np.sum(self.weights ** 2)
        
        # Resample if needed
        if n_eff < self.N / 2.0:
            self.resample()

    def resample(self):
        indices = np.random.choice(self.N, size=self.N, p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.N) / self.N

    def get_estimate(self):
        """ Returns mean and covariance """
        mean = np.average(self.particles, weights=self.weights, axis=0)
        cov = np.cov(self.particles, rowvar=False, aweights=self.weights)
        return mean, cov

from std_msgs.msg import Float32

class TerrainMatchingNode:
    def __init__(self):
        rospy.init_node('terrain_matching_node')
        
        # Params
        asc_path = rospy.get_param('~map_path', '/home/cgz/catkin_ws/src/uuv_eskf_nav/gebco_2025_n40.4838_s39.4745_w131.501_e132.4997.asc')
        self.robot_name = rospy.get_namespace().strip('/')
        # TF frames
        self.world_frame = rospy.get_param('~world_frame', 'odom') # ESKF usually outputs in 'odom' frame
        self.base_frame = f"{self.robot_name}/base_link"
        
        # Load Map
        self.map_server = TerrainMapServer(asc_path)
        
        # TF Listener
        self.tf_listener = tf.TransformListener()
        
        # Filter State
        self.pf = None
        self.last_eskf_pos = None # [x, y]
        self.current_depth = 0.0
        self.lock = threading.Lock()
        
        # GT State for Error Calculation
        self.current_gt_pos = None # [x, y]

        # Data Buffers
        self.sss_left_data = None
        self.sss_right_data = None
        
        # Publishers
        self.pose_pub = rospy.Publisher('terrain_nav/pose', PoseWithCovarianceStamped, queue_size=10)
        self.error_pub = rospy.Publisher('terrain_nav/error_norm', Float32, queue_size=10)
        self.eskf_error_pub = rospy.Publisher('eskf/error_norm', Float32, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('pressure', FluidPressure, self.pressure_cb)
        
        # Navigation Input (ESKF)
        rospy.Subscriber('/eskf/odometry/filtered', Odometry, self.eskf_cb)
        
        # Ground Truth Input (ONLY for verification)
        rospy.Subscriber('pose_gt', Odometry, self.gt_cb)
        
        # Sensor Subs
        rospy.Subscriber('sss_left', LaserScan, self.sss_left_cb)
        rospy.Subscriber('sss_right', LaserScan, self.sss_right_cb)
        
        # Timer for Update Loop (1Hz - 5Hz)
        rospy.Timer(rospy.Duration(0.5), self.update_loop)

        rospy.loginfo("Terrain Matching Node Initialized (Waiting for ESKF).")

    def pressure_cb(self, msg):
        # Use project standard conversion
        if msg.fluid_pressure >= ATM_PRESSURE_KPA:
            depth = (msg.fluid_pressure - ATM_PRESSURE_KPA) / KPA_PER_METER
        else:
            depth = 0.0
        self.current_depth = -depth

    def gt_cb(self, msg):
        """ Ground Truth callback - ONLY for error calculation """
        self.current_gt_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def eskf_cb(self, msg):
        """ ESKF Callback - Drives Prediction and Initialization """
        cur_x = msg.pose.pose.position.x
        cur_y = msg.pose.pose.position.y
        cur_pos = np.array([cur_x, cur_y])
        
        # Calculate and Publish ESKF Error
        #if self.current_gt_pos is not None:
        #   eskf_error = np.linalg.norm(cur_pos - self.current_gt_pos)
        #   self.eskf_error_pub.publish(Float32(eskf_error))

        # 1. Init PF if needed
        if self.pf is None:
            rospy.loginfo(f"Initializing PF using ESKF pose at ({cur_x:.2f}, {cur_y:.2f})")
            # Increase initial uncertainty because ESKF start might be drifted
            self.pf = ParticleFilter(2000, [cur_x, cur_y], [20.0, 20.0], self.map_server)
            self.last_eskf_pos = cur_pos
            return

        # 2. Predict step
        if self.last_eskf_pos is not None:
            delta = cur_pos - self.last_eskf_pos
            # Only update if moved enough
            if np.linalg.norm(delta) > 0.1:
                with self.lock:
                    self.pf.predict(delta[0], delta[1])
                self.last_eskf_pos = cur_pos

    def sss_left_cb(self, msg):
        self.sss_left_data = msg
    def sss_right_cb(self, msg):
        self.sss_right_data = msg

    def laserscan_to_points(self, msg, sensor_frame_id):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        n = min(len(angles), len(msg.ranges))
        angles = angles[:n]
        ranges = np.array(msg.ranges[:n])
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        if len(ranges) == 0: return np.empty((0, 3))
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)
        points = np.column_stack((x, y, z))
        return points

    def update_loop(self, event):
        if self.pf is None: return
        
        # Aggregate observations
        obs_points_body = []
        
        def process_scan(scan_msg, frame_id):
            if scan_msg is None: return
            points_sensor = self.laserscan_to_points(scan_msg, frame_id)
            if len(points_sensor) == 0: return
            try:
                # Transform Sensor -> Body
                (trans_v, rot_v) = self.tf_listener.lookupTransform(self.base_frame, frame_id, rospy.Time(0))
                R_bs = trans.quaternion_matrix(rot_v)[:3, :3]
                T_bs = np.array(trans_v)
                points_body = np.dot(points_sensor, R_bs.T) + T_bs
                obs_points_body.append(points_body)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        process_scan(self.sss_left_data, f"{self.robot_name}/sonarleft_link")
        process_scan(self.sss_right_data, f"{self.robot_name}/sonarright_link")
        
        if not obs_points_body: return
        all_obs_body = np.vstack(obs_points_body)

        # Get Orientation from ESKF (odom -> base_link)
        try:
            (trans_v, rot_v) = self.tf_listener.lookupTransform(self.world_frame, self.base_frame, rospy.Time(0))
            R_wb = trans.quaternion_matrix(rot_v)[:3, :3]
            
            with self.lock:
                self.pf.update(all_obs_body, self.current_depth, R_wb)
                mean, cov = self.pf.get_estimate()
                
                # Publish Pose
                out_msg = PoseWithCovarianceStamped()
                out_msg.header.stamp = rospy.Time.now()
                out_msg.header.frame_id = "world" # Our map is always 'world' fixed
                out_msg.pose.pose.position.x = mean[0]
                out_msg.pose.pose.position.y = mean[1]
                out_msg.pose.pose.position.z = self.current_depth 
                out_msg.pose.pose.orientation.w = 1.0
                out_msg.pose.covariance[0] = cov[0,0]
                out_msg.pose.covariance[7] = cov[1,1]
                self.pose_pub.publish(out_msg)
                
                # Error Calculation (Validation)
                if self.current_gt_pos is not None:
                    # 1. PF Error
                    est_pos = np.array([mean[0], mean[1]])
                    error = np.linalg.norm(est_pos - self.current_gt_pos)
                    self.error_pub.publish(Float32(error))
                    
                    # 2. ESKF Error (using last received ESKF pose)
                    if self.last_eskf_pos is not None:
                        eskf_error = np.linalg.norm(self.last_eskf_pos - self.current_gt_pos)
                        self.eskf_error_pub.publish(Float32(eskf_error))
                    
                    print(f"\r[PF] Error: {error:.2f} m | StdDev: {np.sqrt(cov[0,0]):.2f} m | Particles: {self.pf.N}", end="")
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # rospy.logwarn_throttle(5.0, f"Waiting for TF: {self.world_frame} -> {self.base_frame}")
            pass

if __name__ == "__main__":
    try:
        node = TerrainMatchingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
