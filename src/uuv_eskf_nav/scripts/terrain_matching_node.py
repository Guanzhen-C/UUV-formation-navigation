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
        
        # Load Map
        self.map_server = TerrainMapServer(asc_path)
        
        # TF Listener
        self.tf_listener = tf.TransformListener()
        
        # Filter State
        self.pf = None
        self.last_odom_time = None
        self.last_pos = None # [x, y]
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
        
        # Subscribers
        rospy.Subscriber('pressure', FluidPressure, self.pressure_cb)
        rospy.Subscriber('pose_gt', Odometry, self.gt_cb) # Handle GT for init and error calc
        # Note: We use the SAME topic for Odom (Prediction) as GT in this demo.
        # In real system, separate Odom (DVL) and GT.
        
        # Sensor Subs
        rospy.Subscriber('sss_left', LaserScan, self.sss_left_cb)
        rospy.Subscriber('sss_right', LaserScan, self.sss_right_cb)
        
        # Timer for Update Loop (1Hz - 5Hz)
        rospy.Timer(rospy.Duration(0.5), self.update_loop)

        rospy.loginfo("Terrain Matching Node Initialized.")

    def pressure_cb(self, msg):
        # Use project standard conversion
        # msg.fluid_pressure is in kPa
        if msg.fluid_pressure >= ATM_PRESSURE_KPA:
            depth = (msg.fluid_pressure - ATM_PRESSURE_KPA) / KPA_PER_METER
        else:
            depth = 0.0
            
        # ROS coordinate: Depth is negative Z
        self.current_depth = -depth

    def gt_cb(self, msg):
        # 1. Update GT for error calc
        self.current_gt_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        
        # 2. Init PF if needed
        if self.pf is None:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            rospy.loginfo(f"Initializing PF at ({x:.2f}, {y:.2f})")
            # Init PF
            self.pf = ParticleFilter(200, [x, y], [10.0, 10.0], self.map_server) # 10m initial uncertainty
            self.last_pos = np.array([x, y])
            self.last_odom_time = msg.header.stamp
            return

        # 3. Predict step (Simulating DVL using GT delta)
        cur_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        
        if self.last_pos is not None:
            delta = cur_pos - self.last_pos
            # Only update if moved enough
            if np.linalg.norm(delta) > 0.1:
                with self.lock:
                    self.pf.predict(delta[0], delta[1])
                self.last_pos = cur_pos
                
    # Removed old odom_cb and merged into gt_cb for simplicity in this demo node.
    # def odom_cb(self, msg): ...
    
    def init_pose_cb(self, msg):
        # Merged into gt_cb
        pass

    def sss_left_cb(self, msg):
        self.sss_left_data = msg
    def sss_right_cb(self, msg):
        self.sss_right_data = msg

    def laserscan_to_points(self, msg, sensor_frame_id):
        """ Convert LaserScan to (N,3) points in SENSOR frame """
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        # Make sure shapes match (sometimes one extra point)
        n = min(len(angles), len(msg.ranges))
        angles = angles[:n]
        ranges = np.array(msg.ranges[:n])
        
        # Filter valid ranges
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        
        if len(ranges) == 0:
            return np.empty((0, 3))

        # Polar to Cartesian (Sensor Frame)
        # Assuming Laser scans in XY plane of the sensor link
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)
        
        points = np.column_stack((x, y, z))
        return points

    def update_loop(self, event):
        if self.pf is None: return
        
        # Aggregate observations
        obs_points_body = []
        
        # Helper to process a scan
        def process_scan(scan_msg, frame_id):
            if scan_msg is None: return
            points_sensor = self.laserscan_to_points(scan_msg, frame_id)
            if len(points_sensor) == 0: return
            
            # Transform Sensor -> Base Link (Body)
            try:
                (trans_v, rot_v) = self.tf_listener.lookupTransform(f"{self.robot_name}/base_link", frame_id, rospy.Time(0))
                R_bs = trans.quaternion_matrix(rot_v)[:3, :3]
                T_bs = np.array(trans_v)
                
                # P_b = R_bs * P_s + T_bs
                points_body = np.dot(points_sensor, R_bs.T) + T_bs
                obs_points_body.append(points_body)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        process_scan(self.sss_left_data, f"{self.robot_name}/sonarleft_link")
        process_scan(self.sss_right_data, f"{self.robot_name}/sonarright_link")
        
        if not obs_points_body:
            return
            
        all_obs_body = np.vstack(obs_points_body) # (N_total, 3)
        
        # Get Orientation (R_wb) from TF (World -> Base)
        try:
            (trans_v, rot_v) = self.tf_listener.lookupTransform("world", f"{self.robot_name}/base_link", rospy.Time(0))
            R_wb = trans.quaternion_matrix(rot_v)[:3, :3]
            
            # Run PF Update
            with self.lock:
                self.pf.update(all_obs_body, self.current_depth, R_wb)
                
                # Publish Result
                mean, cov = self.pf.get_estimate()
                
                out_msg = PoseWithCovarianceStamped()
                out_msg.header.stamp = rospy.Time.now()
                out_msg.header.frame_id = "world"
                out_msg.pose.pose.position.x = mean[0]
                out_msg.pose.pose.position.y = mean[1]
                # Z is not estimated, use current depth or map depth
                out_msg.pose.pose.position.z = self.current_depth 
                
                # Identity orientation (we don't estimate it)
                out_msg.pose.pose.orientation.w = 1.0
                
                # Covariance (6x6)
                # We only fill X, Y diagonal
                out_msg.pose.covariance[0] = cov[0,0]
                out_msg.pose.covariance[7] = cov[1,1]
                # Rest is 0 or default
                
                self.pose_pub.publish(out_msg)
                
                # Error Calculation
                if self.current_gt_pos is not None:
                    est_pos = np.array([mean[0], mean[1]])
                    error = np.linalg.norm(est_pos - self.current_gt_pos)
                    self.error_pub.publish(Float32(error))
                    # Print to console (throttle to avoid spam)
                    # rospy.loginfo_throttle(1.0, f"PF Error: {error:.2f} m | StdDev: {np.sqrt(cov[0,0]):.2f} m")
                    print(f"\r[PF] Error: {error:.2f} m | StdDev: {np.sqrt(cov[0,0]):.2f} m | Particles: {self.pf.N}", end="")
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

if __name__ == "__main__":
    try:
        node = TerrainMatchingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
