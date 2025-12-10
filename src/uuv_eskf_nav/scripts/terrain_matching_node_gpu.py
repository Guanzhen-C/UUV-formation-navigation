#!/usr/bin/env python3
import rospy
import numpy as np
import tf
import tf.transformations as trans
from sensor_msgs.msg import PointCloud2, FluidPressure
from sensor_msgs import point_cloud2 as pc2
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
        self.last_n_eff = float(self.N)
        
        # State: [x, y] on GPU
        self.particles = torch.zeros((self.N, 2), device=self.device, dtype=torch.float32)
        self.weights = torch.ones(self.N, device=self.device, dtype=torch.float32) / self.N
        
        # Initialize
        init_mean = torch.tensor(initial_pose, device=self.device, dtype=torch.float32)
        init_std = torch.tensor(initial_cov, device=self.device, dtype=torch.float32)
        
        self.particles[:, 0] = torch.normal(init_mean[0], init_std[0], (self.N,), device=self.device)
        self.particles[:, 1] = torch.normal(init_mean[1], init_std[1], (self.N,), device=self.device)
        
        # Parameters - Adjusted for better stability with more particles
        # Increase process noise slightly to improve exploration with more particles
        self.process_noise = 0.12  # Increased from 0.08 to improve exploration with 2000 particles
        self.sigma_z = 1.8  # Increased from 1.5 to account for measurement uncertainty with more particles 

    def predict(self, dx, dy):
        # Add delta
        self.particles[:, 0] += dx
        self.particles[:, 1] += dy
        
        # Adaptive noise based on movement magnitude - higher noise during larger movements
        movement_magnitude = np.sqrt(dx**2 + dy**2)  # Use numpy since dx, dy are numpy values
        adaptive_noise = self.process_noise * (1.0 + 0.5 * np.tanh(movement_magnitude))
        
        # Add noise - ensure noise is a tensor on the correct device
        noise = torch.normal(0.0, adaptive_noise, (self.N, 2), device=self.device)
        self.particles += noise

    def update(self, obs_offsets_np, z_meas_np):
        """
        obs_offsets_np: (M, 2) numpy array of relative XY
        z_meas_np: (M, ) numpy array of absolute depths
        """
        if len(z_meas_np) == 0: return

        # 1. Pre-process Observation (CPU -> GPU)
        dx_obs = torch.from_numpy(obs_offsets_np[:, 0]).float().to(self.device) # (M,)
        dy_obs = torch.from_numpy(obs_offsets_np[:, 1]).float().to(self.device) # (M,)
        z_meas = torch.from_numpy(z_meas_np).float().to(self.device)            # (M,)
        
        # Calculate mean of measurements for bias removal
        meas_mean = torch.mean(z_meas)
        
        # Remove bias for relative matching
        z_meas_rel = z_meas - meas_mean
        
        # 2. Construct Query Grid (Broadcasting)
        # qx[i, j] = p_x[i] + dx[j] -> (N, M)
        qx = self.particles[:, 0].view(-1, 1) + dx_obs.view(1, -1) 
        qy = self.particles[:, 1].view(-1, 1) + dy_obs.view(1, -1)
        
        # Flatten for grid_sample: (N*M, )
        qx_flat = qx.view(-1)
        qy_flat = qy.view(-1)
        
        # 3. Query Map (GPU)
        z_map_flat = self.map.get_elevation_batch(qx_flat, qy_flat)
        z_map = z_map_flat.view(self.N, -1) # (N, M)
        
        # Calculate map mean for each particle to allow for bias
        z_map_means = torch.mean(z_map, dim=1, keepdim=True)  # (N, 1)
        
        # Use relative matching with per-particle bias
        z_map_rel = z_map - z_map_means  # (N, M) - (N, 1) -> (N, M)
        
        # 4. Calculate likelihood with improved robustness
        diff = z_meas_rel.view(1, -1) - z_map_rel  # (1, M) - (N, M) -> (N, M)
        
        # Calculate Absolute Difference MSE
        mse = torch.mean(diff**2, dim=1) # (N,)
        
        # Calculate MAE as well for robustness
        mae = torch.mean(torch.abs(diff), dim=1) # (N,)
        
        # Calculate terrain feature sensitivity based on local variance
        local_terrain_var = torch.var(z_meas_rel)
        # Use adaptive weighting: higher MSE weight when terrain has more variation
        mse_weight = 0.7 + 0.2 * torch.tanh(local_terrain_var)  # Emphasize MSE for better precision
        mae_weight = 1.0 - mse_weight
        
        # Combine MSE and MAE for more robust likelihood
        combined_error = mse_weight * mse + mae_weight * mae
        
        # IMPLEMENT IMPROVED ADAPTIVE SIGMA FOR MORE PARTICLES
        terrain_variation = torch.std(z_meas_rel)
        
        # Adjust sigma based on number of particles (more particles need more conservative matching)
        base_sigma_adjustment = 1.0 + 0.3 * (self.N / 1000.0 - 1.0)  # Increase sigma for more particles
        adaptive_sigma = self.sigma_z * base_sigma_adjustment * (1.0 + 0.3 * (1.0 - torch.tanh(terrain_variation)))
        
        # Gaussian Likelihood with adaptive sigma
        lik = torch.exp(-combined_error / (2 * adaptive_sigma**2))
        lik += 1e-30
        
        # IMPLEMENT IMPROVED ROBUSTNESS FOR MORE PARTICLES:
        # Cap the likelihood ratio to prevent extreme weight concentration
        max_lik = torch.max(lik)
        min_lik = torch.min(lik)
        mean_lik = torch.mean(lik)
        lik_ratio = max_lik / (min_lik + 1e-30)
        
        # Calculate weight entropy to measure distribution uniformity
        normalized_weights = self.weights / (torch.sum(self.weights) + 1e-30)
        weight_entropy = -torch.sum(normalized_weights * torch.log(normalized_weights + 1e-30))
        max_entropy = torch.log(torch.tensor(self.N, dtype=torch.float32))
        entropy_ratio = weight_entropy / max_entropy  # Between 0 and 1 (1 = uniform)
        
        # If the likelihood ratio is too high or entropy is too low, indicating potential divergence, adjust
        if lik_ratio > 1e4 or entropy_ratio < 0.1:  # Reduced threshold for more particles
            # Apply soft likelihood normalization to prevent extreme weights
            # Use a more conservative approach with more particles
            min_val = mean_lik * 0.0001  # More conservative minimum
            max_val = mean_lik * 1000   # More conservative maximum
            lik = torch.clamp(lik, min=min_val, max=max_val)
        
        # Update weights
        self.weights *= lik
        self.weights /= torch.sum(self.weights)
        
        # 5. Resample with enhanced stability measures for more particles
        n_eff = 1.0 / torch.sum(self.weights**2)
        self.last_n_eff = n_eff.item()
        
        # Stats for logging (Raw residuals)
        self.last_residual_stats = {
            'mean': torch.mean(diff).item(), 
            'std': torch.std(diff).item()
        }
        
        # Calculate and store diff statistics for logging
        # Use the diff from the best particle or overall
        overall_diff = diff.mean(dim=1)  # Average across measurements for each particle
        self.last_diff_stats = {
            'mean': overall_diff.mean().item(),
            'std': overall_diff.std().item()
        }
        
        # IMPLEMENT ENHANCED RESAMPLING STRATEGY FOR MORE PARTICLES
        # Use a more conservative resampling threshold with more particles
        terrain_variation = torch.std(z_meas_rel)
        # Lower resampling threshold for more particles (allow more particles to survive before resampling)
        adaptive_threshold = 0.3 * self.N if terrain_variation > 0.5 else 0.2 * self.N
        
        # Also resample if weights are too concentrated (potential divergence)
        weight_concentration = torch.max(self.weights) / (torch.mean(self.weights) + 1e-30)
        
        # Check for weight degeneracy with more conservative metrics for more particles
        if n_eff < adaptive_threshold or weight_concentration > 50:
            # Add diversity before resampling
            noise_before_resample = torch.normal(0.0, self.process_noise * 0.2, (self.N, 2), device=self.device)
            self.particles += noise_before_resample
            
            self.resample()
            
            # After resampling, reduce the impact of the measurement to avoid over-correction
            # This helps when we have many particles and measurements might be misleading
        else:
            # Even without resampling, maintain diversity with noise injection
            # Adjust noise injection frequency based on number of particles
            noise_freq = min(0.15, 0.05 + 0.1 * (self.N / 2000.0))  # Scale with number of particles
            if np.random.random() < noise_freq:
                # Use adaptive noise based on current weight distribution
                if weight_concentration > 30:  # High concentration
                    noise_scale = self.process_noise * 0.3
                elif weight_concentration < 5:  # Low concentration
                    noise_scale = self.process_noise * 0.05
                else:  # Moderate concentration
                    noise_scale = self.process_noise * 0.15
                noise = torch.normal(0.0, noise_scale, (self.N, 2), device=self.device)
                self.particles += noise

        # ADD ENHANCED DIVERGENCE DETECTION AND RECOVERY FOR MORE PARTICLES
        # If effective sample size is very low or weight entropy is low, we might be diverging
        if n_eff < 0.05 * self.N or entropy_ratio < 0.05:
            print(f"[GPU-PF] Critical: Low effective sample size ({n_eff:.0f}/{self.N}) or low entropy, restarting with diversity")
            # Reinitialize particles with increased spread around current estimate
            current_mean = torch.sum(self.particles * self.weights.view(-1, 1), dim=0)
            # Add significant noise to escape potential local minima
            restart_noise = torch.normal(0.0, self.process_noise * 2.0, (self.N, 2), device=self.device)
            self.particles = current_mean + restart_noise
            # Reset weights to uniform
            self.weights = torch.ones(self.N, device=self.device, dtype=torch.float32) / self.N

    def resample(self):
        # Use systematic resampling for better particle diversity
        cumulative_weights = torch.cumsum(self.weights, dim=0)
        cumulative_weights[-1] = 1.0  # Ensure sum is exactly 1
        
        # Systematic resampling
        u = torch.arange(self.N, dtype=torch.float32, device=self.device) / self.N
        offset = torch.rand(1, device=self.device) / self.N
        u = u + offset
        
        # Find indices using searchsorted
        indices = torch.searchsorted(cumulative_weights, u, right=True)
        indices = torch.clamp(indices, 0, self.N - 1)  # Ensure valid indices
        
        # Add more significant noise to resampled particles to maintain diversity
        self.particles = self.particles[indices]
        # Adjust noise level based on terrain features and particle distribution
        if hasattr(self, 'last_n_eff'):
            # If effective sample size was low, use higher noise to increase diversity
            if self.last_n_eff < 0.3 * self.N:
                noise_scale = self.process_noise * 1.0  # Higher noise when diversity is low
            else:
                noise_scale = self.process_noise * 0.5  # Lower noise for better accuracy
        else:
            noise_scale = self.process_noise * 0.5
            
        noise = torch.normal(0.0, noise_scale, (self.N, 2), device=self.device)
        self.particles += noise
        
        self.weights = torch.ones(self.N, device=self.device) / self.N

    def get_estimate(self):
        # Mean
        mean = torch.sum(self.particles * self.weights.view(-1, 1), dim=0)
        
        # Covariance
        diff = self.particles - mean
        cov_xx = torch.sum(self.weights * diff[:, 0]**2)
        cov_yy = torch.sum(self.weights * diff[:, 1]**2)
        
        return mean.cpu().numpy(), np.array([[cov_xx.item(), 0], [0, cov_yy.item()]])

class TerrainMatchingNodeGPU:
    def __init__(self):
        rospy.init_node('terrain_matching_node_gpu')
        
        asc_path = rospy.get_param('~map_path', '/home/cgz/catkin_ws/src/uuv_eskf_nav/gazebo_terrain.asc')
        self.robot_name = rospy.get_namespace().strip('/')
        self.world_frame = rospy.get_param('~world_frame', 'odom')
        self.base_frame = f"{self.robot_name}/base_link"
        
        # Check GPU
        if not torch.cuda.is_available():
            rospy.logerr("CUDA not available! Falling back to CPU or dying.")
        self.device = 'cuda'
        
        # Load Map (GPU)
        self.map_server = TerrainMapServerGPU(asc_path, device=self.device)
        
        self.tf_listener = tf.TransformListener()
        self.pf = None
        self.last_eskf_pos = None
        self.last_orientation = None
        self.current_depth = 0.0
        self.lock = threading.Lock()
        self.current_gt_pos = None

        self.mbes_data = None
        
        self.pose_pub = rospy.Publisher('terrain_nav/pose', PoseWithCovarianceStamped, queue_size=10)
        self.error_pub = rospy.Publisher('terrain_nav/error_norm', Float32, queue_size=10)
        self.eskf_error_pub = rospy.Publisher('eskf/error_norm', Float32, queue_size=10)
        
        rospy.Subscriber('pressure', FluidPressure, self.pressure_cb)
        rospy.Subscriber('/eskf/odometry/filtered', Odometry, self.eskf_cb)
        rospy.Subscriber(f'/{self.robot_name}/pose_gt', Odometry, self.gt_cb)
        
        # MBES Subscriber
        rospy.Subscriber('/eca_a9/depth/points', PointCloud2, self.mbes_cb)
        
        rospy.Timer(rospy.Duration(0.5), self.update_loop)

        rospy.loginfo(f"Terrain Matching Node (GPU - MBES) Initialized on {self.device}.")

    def pressure_cb(self, msg):
        if msg.fluid_pressure >= ATM_PRESSURE_KPA:
            depth = (msg.fluid_pressure - ATM_PRESSURE_KPA) / KPA_PER_METER
        else:
            depth = 0.0
        self.current_depth = -depth

    def gt_cb(self, msg):
        self.current_gt_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

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
        quat = msg.pose.pose.orientation
        self.last_orientation = [quat.x, quat.y, quat.z, quat.w]

        if self.pf is None:
            self.num_particles = rospy.get_param('~num_particles', 1000)
            rospy.loginfo(f"Initializing GPU PF at ({cur_x:.2f}, {cur_y:.2f}) with {self.num_particles} particles")
            self.pf = ParticleFilterGPU(self.num_particles, [cur_x, cur_y], [5.0, 5.0], self.map_server, self.device)
            self.last_eskf_pos = cur_pos
            return

        if self.last_eskf_pos is not None:
            delta = cur_pos - self.last_eskf_pos
            if np.linalg.norm(delta) > 0.1:
                with self.lock:
                    self.pf.predict(delta[0], delta[1])
                self.last_eskf_pos = cur_pos

    def mbes_cb(self, msg):
        self.mbes_data = msg

    def update_loop(self, event):
        if self.pf is None: return
        if self.mbes_data is None: return
        
        # 1. Parse MBES
        gen = pc2.read_points(self.mbes_data, field_names=("x", "y", "z"), skip_nans=True)
        pts_optical = np.array(list(gen))
        if len(pts_optical) == 0: return
        
        # Additional filtering: Remove points where z is exactly 0 (which may indicate no return)
        # This can happen when sonar beams don't hit terrain within range
        non_zero_mask = pts_optical[:, 2] != 0.0
        pts_optical = pts_optical[non_zero_mask]
        if len(pts_optical) == 0: return
        
        # Use all available MBES points for better terrain matching
        # Only downsample if the number of points is extremely large (memory concerns)
        original_count = len(pts_optical)
        if len(pts_optical) > 10000:  # Only downsample if more than 10000 points
            indices = np.random.choice(len(pts_optical), 10000, replace=False)
            pts_optical = pts_optical[indices]
            rospy.loginfo(f"Downsampled from {original_count} to {len(pts_optical)} points")
        
        # 2. Transform Optical -> Body
        try:
            (t, r) = self.tf_listener.lookupTransform(f"{self.robot_name}/base_link", self.mbes_data.header.frame_id, rospy.Time(0))
            R_ob = trans.quaternion_matrix(r)[:3, :3]
            T_ob = np.array(t)
            pts_body = np.dot(pts_optical, R_ob.T) + T_ob
        except:
            return

        try:
            # 3. Transform Body -> World (Orientation from ESKF/Last Known)
            # Using ESKF orientation (consistent with 'real' nav)
            # Note: In CPU version we used GT orientation for debugging.
            # Here we use 'last_orientation' from ESKF/Odom callback.
            if self.last_orientation is None: return
            
            R_wb = trans.quaternion_matrix(self.last_orientation)[:3, :3]
            pts_rotated = np.dot(pts_body, R_wb.T)
            
            # 4. Prepare Measurement
            z_meas = self.current_depth + pts_rotated[:, 2] # Absolute depths
            obs_offsets = pts_rotated[:, :2] # Relative XY

            with self.lock:
                self.pf.update(obs_offsets, z_meas)
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
                    
                    res_info = ""
                    res_stats = getattr(self.pf, "last_residual_stats", None)
                    if res_stats:
                        res_info = f" | ResRaw[Mean:{res_stats['mean']:.1f} Std:{res_stats['std']:.1f}]"
                    
                # Calculate Meas Stats using actual filtered count
                meas_mean = np.mean(z_meas)
                meas_std = np.std(z_meas)
                meas_info = f" | Meas[N:{len(z_meas)} Mean:{meas_mean:.1f} Std:{meas_std:.1f}]"
                
                # Add diff stats if available
                diff_info = ""
                if hasattr(self.pf, 'last_diff_stats'):
                    diff_mean = self.pf.last_diff_stats['mean']
                    diff_std = self.pf.last_diff_stats['std']
                    diff_info = f" | Diff[Mean:{diff_mean:.1f} Std:{diff_std:.1f}]"
                
                # Get ground truth position if available
                gt_info = ""
                if self.current_gt_pos is not None:
                    gt_info = f" | GT[Pos:({self.current_gt_pos[0]:.2f}, {self.current_gt_pos[1]:.2f})]"
                
                # Get current ROS time for timestamp
                current_time = rospy.get_time()
                
                print(f"\r[GPU-PF] Time:{current_time:.2f} | Error: {error:.2f} m | StdDev: {np.sqrt(cov[0,0]):.2f} m | Neff: {self.pf.last_n_eff:.0f}{res_info}{meas_info}{diff_info}{gt_info}")                
        except Exception as e:
            rospy.logerr_throttle(1.0, f"GPU PF Update Error: {e}")
            pass

if __name__ == "__main__":
    try:
        node = TerrainMatchingNodeGPU()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass