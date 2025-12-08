#!/usr/bin/env python3
import rospy
import numpy as np
import tf
import tf.transformations as trans
from sensor_msgs.msg import LaserScan, FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from terrain_map_server_gpu import TerrainMapServerGPU
import threading
import torch

ATM_PRESSURE_KPA = 101.325
KPA_PER_METER = 9.80638

class ParticleFilterGPU:
    def __init__(self, num_particles, initial_pose, initial_cov, map_server, device='cuda'):
        self.N = num_particles
        self.map = map_server
        self.device = device
        
        # State: [x, y] on GPU
        self.particles = torch.zeros((self.N, 2), device=self.device, dtype=torch.float32)
        self.weights = torch.ones(self.N, device=self.device, dtype=torch.float32) / self.N
        
        # Initialize
        init_mean = torch.tensor(initial_pose, device=self.device, dtype=torch.float32)
        init_std = torch.tensor(initial_cov, device=self.device, dtype=torch.float32)
        
        self.particles[:, 0] = torch.normal(init_mean[0], init_std[0], (self.N,), device=self.device)
        self.particles[:, 1] = torch.normal(init_mean[1], init_std[1], (self.N,), device=self.device)
        
        # Parameters
        self.process_noise = 2.0
        self.sigma_z = 0.5 # Measurement noise (Stricter)

    def predict(self, dx, dy):
        # Add delta
        self.particles[:, 0] += dx
        self.particles[:, 1] += dy
        
        # Add noise
        noise = torch.normal(0.0, self.process_noise, (self.N, 2), device=self.device)
        self.particles += noise

    def update(self, obs_points_body, current_depth, R_wb_np):
        """
        obs_points_body: (M, 3) numpy array
        current_depth: float
        R_wb_np: (3, 3) numpy array
        """
        if len(obs_points_body) == 0: return

        # 1. Pre-process Observation (CPU -> GPU)
        # Ensure float32
        obs_body_tensor = torch.from_numpy(obs_points_body).float().to(self.device) # (M, 3)
        R_wb_tensor = torch.from_numpy(R_wb_np).float().to(self.device) # (3, 3)
        
        # (M, 3) @ (3, 3).T -> (M, 3)
        obs_rel = torch.matmul(obs_body_tensor, R_wb_tensor.t())
        
        dx_obs = obs_rel[:, 0] # (M,)
        dy_obs = obs_rel[:, 1] # (M,)
        dz_obs = obs_rel[:, 2] # (M,)
        
        # Z_meas = Depth + dz
        z_meas = current_depth + dz_obs # (M,)
        
        # 2. Construct Query Grid (Broadcasting)
        # We want to query map at: Particle_pos + Obs_offset
        # shape: (N, M)
        # qx[i, j] = p_x[i] + dx[j]
        
        qx = self.particles[:, 0].view(-1, 1) + dx_obs.view(1, -1) # (N, M)
        qy = self.particles[:, 1].view(-1, 1) + dy_obs.view(1, -1) # (N, M)
        
        # Flatten for grid_sample: (N*M, )
        qx_flat = qx.view(-1)
        qy_flat = qy.view(-1)
        
        # 3. Query Map (GPU)
        z_map_flat = self.map.get_elevation_batch(qx_flat, qy_flat)
        z_map = z_map_flat.view(self.N, -1) # (N, M)
        
        # 4. Calculate Likelihood
        # Error = Z_meas - Z_map
        # z_meas is (M,), broadcasts to (N, M)
        diff = z_meas.view(1, -1) - z_map
        
        # Mean Squared Error per particle
        # Handle outliers? (e.g. map nodata). get_elevation_batch returns min_elev for OOB.
        # Let's assume valid.
        mse = torch.mean(diff**2, dim=1) # (N,)
        
        # Gaussian Likelihood
        lik = torch.exp(-mse / (2 * self.sigma_z**2))
        
        # Avoid zero
        lik += 1e-30
        
        # Update weights
        self.weights *= lik
        self.weights /= torch.sum(self.weights)
        
        # 5. Resample
        n_eff = 1.0 / torch.sum(self.weights**2)
        if n_eff < self.N / 2.0:
            self.resample()

    def resample(self):
        indices = torch.multinomial(self.weights, self.N, replacement=True)
        self.particles = self.particles[indices]
        self.weights = torch.ones(self.N, device=self.device) / self.N

    def get_estimate(self):
        # Mean
        mean = torch.sum(self.particles * self.weights.view(-1, 1), dim=0)
        
        # Covariance
        diff = self.particles - mean
        # Weighted Covariance
        # cov = (weights * diff.T) @ diff
        # Manual calculation for 2x2
        # w_diff = diff * torch.sqrt(self.weights.view(-1, 1)) # broadcasting
        # cov = torch.matmul(w_diff.t(), w_diff) 
        # Or simple:
        cov_xx = torch.sum(self.weights * diff[:, 0]**2)
        cov_yy = torch.sum(self.weights * diff[:, 1]**2)
        # Ignore xy for now
        
        # Return to CPU
        return mean.cpu().numpy(), np.array([[cov_xx.item(), 0], [0, cov_yy.item()]])

class TerrainMatchingNodeGPU:
    def __init__(self):
        rospy.init_node('terrain_matching_node_gpu')
        
        # Params
        asc_path = rospy.get_param('~map_path', '/home/cgz/catkin_ws/src/uuv_eskf_nav/terrain_with_noise.asc')
        self.robot_name = rospy.get_namespace().strip('/')
        self.world_frame = rospy.get_param('~world_frame', 'odom')
        self.base_frame = f"{self.robot_name}/base_link"
        
        # Check GPU
        if not torch.cuda.is_available():
            rospy.logerr("CUDA not available! Falling back to CPU or dying.")
            # self.device = 'cpu' # Could fallback
        self.device = 'cuda'
        
        # Load Map (GPU)
        self.map_server = TerrainMapServerGPU(asc_path, device=self.device)
        
        self.tf_listener = tf.TransformListener()
        self.pf = None
        self.last_eskf_pos = None
        self.current_depth = 0.0
        self.lock = threading.Lock()
        self.current_gt_pos = None

        self.sss_left_data = None
        self.sss_right_data = None
        
        self.pose_pub = rospy.Publisher('terrain_nav/pose', PoseWithCovarianceStamped, queue_size=10)
        self.error_pub = rospy.Publisher('terrain_nav/error_norm', Float32, queue_size=10)
        self.eskf_error_pub = rospy.Publisher('eskf/error_norm', Float32, queue_size=10)
        
        rospy.Subscriber('pressure', FluidPressure, self.pressure_cb)
        rospy.Subscriber('/eskf/odometry/filtered', Odometry, self.eskf_cb)
        rospy.Subscriber('pose_gt', Odometry, self.gt_cb)
        rospy.Subscriber('sss_left', LaserScan, self.sss_left_cb)
        rospy.Subscriber('sss_right', LaserScan, self.sss_right_cb)
        
        rospy.Timer(rospy.Duration(0.5), self.update_loop) # 2Hz is enough, but GPU can do 50Hz if you want

        rospy.loginfo(f"Terrain Matching Node (GPU) Initialized on {self.device}.")

    # ... Copy callbacks from CPU version, mostly identical ...
    def pressure_cb(self, msg):
        if msg.fluid_pressure >= ATM_PRESSURE_KPA:
            depth = (msg.fluid_pressure - ATM_PRESSURE_KPA) / KPA_PER_METER
        else:
            depth = 0.0
        self.current_depth = -depth

    def gt_cb(self, msg):
        self.current_gt_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def eskf_cb(self, msg):
        cur_x = msg.pose.pose.position.x
        cur_y = msg.pose.pose.position.y
        cur_pos = np.array([cur_x, cur_y])

        if self.pf is None:
            rospy.loginfo(f"Initializing GPU PF at ({cur_x:.2f}, {cur_y:.2f}) with 100000 particles")
            # 100,000 Particles!
            self.pf = ParticleFilterGPU(100000, [cur_x, cur_y], [20.0, 20.0], self.map_server, self.device)
            self.last_eskf_pos = cur_pos
            return

        if self.last_eskf_pos is not None:
            delta = cur_pos - self.last_eskf_pos
            if np.linalg.norm(delta) > 0.1:
                with self.lock:
                    self.pf.predict(delta[0], delta[1])
                self.last_eskf_pos = cur_pos

    def sss_left_cb(self, msg): self.sss_left_data = msg
    def sss_right_cb(self, msg): self.sss_right_data = msg

    def laserscan_to_points(self, msg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        n = min(len(angles), len(msg.ranges))
        ranges = np.array(msg.ranges[:n])
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        if len(ranges) == 0: return np.empty((0, 3))
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)
        return np.column_stack((x, y, z))

    def update_loop(self, event):
        if self.pf is None: return
        
        obs_points_body = []
        def process_scan(scan_msg, frame_id):
            if scan_msg is None: return
            points_sensor = self.laserscan_to_points(scan_msg)
            if len(points_sensor) == 0: return
            try:
                (trans_v, rot_v) = self.tf_listener.lookupTransform(self.base_frame, frame_id, rospy.Time(0))
                R_bs = trans.quaternion_matrix(rot_v)[:3, :3]
                T_bs = np.array(trans_v)
                points_body = np.dot(points_sensor, R_bs.T) + T_bs
                obs_points_body.append(points_body)
            except:
                pass

        process_scan(self.sss_left_data, f"{self.robot_name}/sonarleft_link")
        process_scan(self.sss_right_data, f"{self.robot_name}/sonarright_link")
        
        if not obs_points_body: return
        all_obs_body = np.vstack(obs_points_body)

        try:
            (trans_v, rot_v) = self.tf_listener.lookupTransform(self.world_frame, self.base_frame, rospy.Time(0))
            R_wb = trans.quaternion_matrix(rot_v)[:3, :3]
            
            with self.lock:
                self.pf.update(all_obs_body, self.current_depth, R_wb)
                mean, cov = self.pf.get_estimate()
                
                out_msg = PoseWithCovarianceStamped()
                out_msg.header.stamp = rospy.Time.now()
                out_msg.header.frame_id = "world"
                out_msg.pose.pose.position.x = mean[0]
                out_msg.pose.pose.position.y = mean[1]
                out_msg.pose.pose.position.z = self.current_depth 
                out_msg.pose.pose.orientation.w = 1.0
                out_msg.pose.covariance[0] = cov[0,0]
                out_msg.pose.covariance[7] = cov[1,1]
                self.pose_pub.publish(out_msg)
                
                if self.current_gt_pos is not None:
                    est_pos = np.array([mean[0], mean[1]])
                    error = np.linalg.norm(est_pos - self.current_gt_pos)
                    self.error_pub.publish(Float32(error))
                    
                    if self.last_eskf_pos is not None:
                        eskf_error = np.linalg.norm(self.last_eskf_pos - self.current_gt_pos)
                        self.eskf_error_pub.publish(Float32(eskf_error))
                    
                    print(f"\r[GPU-PF] Error: {error:.2f} m | StdDev: {np.sqrt(cov[0,0]):.2f} m | Particles: {self.pf.N}", end="")
                
        except Exception as e:
            rospy.logerr_throttle(1.0, f"GPU PF Update Error: {e}")
            pass

if __name__ == "__main__":
    try:
        node = TerrainMatchingNodeGPU()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
