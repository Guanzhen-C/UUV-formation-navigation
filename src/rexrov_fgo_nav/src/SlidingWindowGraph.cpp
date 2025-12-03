#include "rexrov_fgo_nav/SlidingWindowGraph.h"
#include "rexrov_fgo_nav/BodyVelocityFactor.h"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace rexrov_fgo_nav {

SlidingWindowGraph::SlidingWindowGraph(ros::NodeHandle& nh) : nh_(nh) {
    // Initialize ISAM2 (incremental optimization, keeps full history)
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;  // More aggressive relinearization for better accuracy
    isam2_params.relinearizeSkip = 1;
    isam2_params.cacheLinearizedFactors = true;
    isam2_params.enableDetailedResults = false;
    isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);

    // Initialize IMU Params (Matched with URDF high precision config)
    // Gravity: 9.81 m/s^2 (Upward ENU convention)
    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(9.81);

    // Parameters from URDF/Xacro (精确匹配导航级IMU规格):
    // URDF: gyroscope_noise_density="0.00000029088" (0.29 μrad/s/√Hz)
    // URDF: accelerometer_noise_density="0.000010" (10 μg/√Hz)
    // 转换为连续时间噪声密度的平方（GTSAM使用连续时间模型）
    // [FIX] Relaxed noise models for numerical stability in ISAM2
    // The extremely small theoretical values (1e-14) cause ill-conditioned matrices.
    
    p->accelerometerCovariance = gtsam::I_3x3 * std::pow(1.0e-3, 2);   
    p->gyroscopeCovariance = gtsam::I_3x3 * std::pow(1.0e-4, 2);    
    p->integrationCovariance = gtsam::I_3x3 * 1e-5; 
    
    preintegration_params_ = p;

    prev_bias_ = gtsam::imuBias::ConstantBias(); // Zero bias initially
    current_preintegration_ = boost::make_shared<gtsam::PreintegratedImuMeasurements>(p, prev_bias_);

    // Noise Models
    // DVL noise - High precision bottom-track DVL with 3-sigma = 0.01 m/s, sigma ~= 0.0033 m/s
    // Using 0.01 m/s to account for body->world rotation uncertainty and minor timing jitter
    velocity_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.01, 0.01, 0.01).finished()); 
    
    // Depth sensor - high precision pressure sensor (sigma = 0.05m)
    depth_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 0.05).finished());
    
    // Bias Random Walk Noise - 匹配ESKF参数
    // ESKF: gyro_bias_std = 8.1e-11, accel_bias_std = 1.6e-4
    // 注意：GTSAM需要乘以sqrt(dt)来转换为离散时间噪声
    // 对于dt=0.1s, 连续时间噪声密度 = 离散噪声 / sqrt(dt)
    // ESKF的8.1e-11是连续时间，FGO需要: 8.1e-11 * sqrt(0.1) ≈ 2.56e-11
    // 但GTSAM数值稳定性要求不能太小，使用1e-9作为折中
    bias_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) <<
        1.0e-3, 1.0e-3, 1.0e-3,   // Acc bias random walk (Relaxed for stability)
        1.0e-5, 1.0e-5, 1.0e-5    // Gyro bias random walk (Relaxed for numerical stability 1e-14 vs 1e-16)
    ).finished());

    // IMU orientation noise (from Gazebo sensor fusion - fairly accurate)
    // Roll, Pitch, Yaw uncertainty in radians
    orientation_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.02, 0.02, 0.05).finished());
}

void SlidingWindowGraph::initialize(double timestamp) {
    // Initial Pose: Use ground truth if available, otherwise fallback to depth + IMU orientation
    gtsam::Pose3 priorPose;
    if (has_init_pose_) {
        priorPose = init_pose_;
        ROS_INFO("Initializing FGO with ground truth pose: [%.2f, %.2f, %.2f]",
                 init_pose_.x(), init_pose_.y(), init_pose_.z());
    } else {
        gtsam::Point3 initPosition(0.0, 0.0, init_depth_);
        priorPose = gtsam::Pose3(init_orientation_, initPosition);
        ROS_INFO("Initializing FGO with depth: %.2f m", init_depth_);
    }
    // Initial Velocity: Use ground truth if available, otherwise zero
    gtsam::Vector3 priorVelocity;
    if (has_init_velocity_) {
        priorVelocity = init_velocity_;
        ROS_INFO("Initializing FGO with ground truth velocity: [%.2f, %.2f, %.2f]",
                 init_velocity_.x(), init_velocity_.y(), init_velocity_.z());
    } else {
        priorVelocity = gtsam::Vector3::Zero();
        ROS_WARN("No initial velocity from ground truth, using zero velocity");
    }
    gtsam::imuBias::ConstantBias priorBias = gtsam::imuBias::ConstantBias();

    gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5).finished());
    gtsam::noiseModel::Diagonal::shared_ptr velNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.1, 0.1, 0.1).finished());
    
    // Initial Bias uncertainty
    gtsam::noiseModel::Diagonal::shared_ptr initBiasNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 
        0.1, 0.1, 0.1,   // Acc init bias (Relaxed)
        0.01, 0.01, 0.01 // Gyro init bias (Relaxed)
    ).finished());

    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), priorPose, poseNoise));
    graph_.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), priorVelocity, velNoise));
    graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), priorBias, initBiasNoise));

    initial_estimates_.insert(X(0), priorPose);
    initial_estimates_.insert(V(0), priorVelocity);
    initial_estimates_.insert(B(0), priorBias);

    current_pose_ = priorPose;
    current_velocity_ = priorVelocity;
    current_time_ = timestamp;

    // ISAM2 update
    isam2_->update(graph_, initial_estimates_);
    graph_.resize(0);
    initial_estimates_.clear();

    last_key_time_ = timestamp;
    initialized_ = true;
    ROS_INFO("FGO ISAM2 Initialized at t=%.3f", timestamp);
}

void SlidingWindowGraph::addImuMeasurement(const gtsam::Vector3& acc, const gtsam::Vector3& gyro, double dt) {
    if (!initialized_) {
        // Wait for first update() or manual init?
        // Usually init happens on first stable measurement set.
        return;
    }
    current_preintegration_->integrateMeasurement(acc, gyro, dt);
}

void SlidingWindowGraph::addImuOrientation(const gtsam::Rot3& orientation) {
    // Capture initial orientation for initialization
    if (!has_init_orientation_) {
        init_orientation_ = orientation;
        has_init_orientation_ = true;
        ROS_INFO("Captured initial IMU orientation");
    }

    // Store latest orientation for attitude constraint (每次都更新！)
    // 这是修复航向漂移的关键：持续使用IMU姿态来约束因子图
    latest_imu_orientation_ = orientation;
    has_new_imu_orientation_ = true;
}

void SlidingWindowGraph::update() {
    std::lock_guard<std::mutex> lock(graph_mutex_);

    if (!initialized_) {
        // Prefer ground truth for initialization, fallback to depth + IMU
        if (has_init_pose_) {
            initialize(ros::Time::now().toSec());
            return;
        }
        // Fallback: Wait for initial depth and orientation
        if (!has_init_depth_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for initial depth or ground truth...");
            return;
        }
        if (!has_init_orientation_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for initial IMU orientation...");
            return;
        }
        initialize(ros::Time::now().toSec());
        return;
    }

    // Ensure we have integrated enough data to form a valid factor
    if (current_preintegration_->deltaTij() < 1e-3) {
        // ROS_WARN("Skipping FGO update: deltaT too small (%.4f)", current_preintegration_->deltaTij());
        return;
    }

    int prev_idx = key_index_;
    key_index_++;
    int curr_idx = key_index_;

    // 更新时间戳 (每次update间隔约0.1秒)
    double dt = current_preintegration_->deltaTij();
    current_time_ += dt;

    // 1. Add IMU Factor connecting prev_idx and curr_idx
    gtsam::ImuFactor imuFactor(X(prev_idx), V(prev_idx), X(curr_idx), V(curr_idx), B(prev_idx), *current_preintegration_);
    graph_.add(imuFactor);

    // 2. Add Bias Random Walk Factor
    graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(prev_idx), B(curr_idx), gtsam::imuBias::ConstantBias(), bias_noise_));

    // 3. Predict Initial Estimate for new node
    gtsam::NavState prevState(current_pose_, current_velocity_);
    gtsam::NavState propState = current_preintegration_->predict(prevState, prev_bias_);

    initial_estimates_.insert(X(curr_idx), propState.pose());
    initial_estimates_.insert(V(curr_idx), propState.velocity());
    initial_estimates_.insert(B(curr_idx), prev_bias_);

    // --- Add Async Factors ---

    // DVL Factor - 使用机体坐标系速度约束（与ESKF相同原理）
    // 不再将DVL速度转换到世界坐标系，而是在机体坐标系中进行约束
    // 这样避免了姿态误差导致的DVL观测错误
    if (has_new_dvl_) {
        // 使用BodyVelocityFactor：在机体坐标系中约束速度
        // error = v_body_measured - R_bw * v_world
        graph_.add(BodyVelocityFactor(X(curr_idx), V(curr_idx), latest_dvl_, velocity_noise_));
        has_new_dvl_ = false;
    }

    // 注意：不使用IMU提供的姿态四元数
    // 只使用IMU的原始加速度和角速度数据（通过IMU预积分因子）
    // 姿态估计完全由IMU预积分 + DVL速度约束来完成
    has_new_imu_orientation_ = false;

    // Depth Factor
    if (has_new_depth_) {
        gtsam::Point3 gps_meas(0, 0, latest_depth_);
        gtsam::SharedNoiseModel gps_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1000.0, 1000.0, 0.05).finished());
        graph_.add(gtsam::GPSFactor(X(curr_idx), gps_meas, gps_noise));
        has_new_depth_ = false;
    }

    // Sonar Factor - 相对位姿约束修正偏航漂移
    // ISAM2 keeps full history, so we can always add the factor
    if (has_new_sonar_) {
        if (sonar_from_node_idx_ >= 0 && sonar_to_node_idx_ > sonar_from_node_idx_) {
            gtsam::noiseModel::Gaussian::shared_ptr sonarNoise = gtsam::noiseModel::Gaussian::Covariance(latest_sonar_cov_);
            graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(sonar_from_node_idx_), X(sonar_to_node_idx_), latest_sonar_pose_, sonarNoise));
            ROS_INFO("Added sonar factor: X(%d) -> X(%d), dt=%.1fs",
                     sonar_from_node_idx_, sonar_to_node_idx_,
                     (sonar_to_node_idx_ - sonar_from_node_idx_) * 0.1);
        }
        has_new_sonar_ = false;
    }

    // 4. ISAM2 incremental update
    try {
        isam2_->update(graph_, initial_estimates_);
        // Additional iterations for better convergence
        isam2_->update();
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "ISAM2 update error: %s", e.what());
    }

    // Clear buffers
    graph_.resize(0);
    initial_estimates_.clear();
    current_preintegration_->resetIntegration();

    // Get estimates from ISAM2
    try {
        gtsam::Values result = isam2_->calculateEstimate();
        if (result.exists(B(curr_idx))) {
            prev_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(curr_idx));
        }
        if (result.exists(X(curr_idx))) {
            current_pose_ = result.at<gtsam::Pose3>(X(curr_idx));
        }
        if (result.exists(V(curr_idx))) {
            current_velocity_ = result.at<gtsam::Vector3>(V(curr_idx));
        }
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "ISAM2 estimate error: %s", e.what());
    }

    // 定期输出状态
    static int update_count = 0;
    if (++update_count % 100 == 0) {
        ROS_INFO("ISAM2: key=%d, time=%.1fs, factors=%zu",
                 curr_idx, current_time_, isam2_->getFactorsUnsafe().size());
    }
}

void SlidingWindowGraph::addDvlMeasurement(const gtsam::Vector3& velocity, double timestamp) {
    if (!initialized_) return;
    std::lock_guard<std::mutex> lock(graph_mutex_);
    latest_dvl_ = velocity;
    has_new_dvl_ = true;
}

void SlidingWindowGraph::addDepthMeasurement(double depth, double timestamp) {
    // Capture initial depth for initialization
    if (!has_init_depth_) {
        init_depth_ = depth;
        has_init_depth_ = true;
        ROS_INFO("Captured initial depth: %.2f m", depth);
    }
    latest_depth_ = depth;
    has_new_depth_ = true;
}

void SlidingWindowGraph::addSonarRelativePose(const gtsam::Pose3& relative_pose, const gtsam::Matrix6& covariance, double timestamp) {
    if (!initialized_) return;

    // Record the FGO node index at the time of this sonar measurement
    // The relative_pose is from the previous MBES keyframe to the current one
    if (first_sonar_keyframe_) {
        // First keyframe - just record the current node index as reference
        last_sonar_node_idx_ = key_index_;
        first_sonar_keyframe_ = false;
        ROS_INFO("First sonar keyframe at FGO node %d", key_index_);
        return;  // No factor to add yet
    }

    // Store the relative pose and the node indices
    sonar_from_node_idx_ = last_sonar_node_idx_;
    sonar_to_node_idx_ = key_index_;
    last_sonar_node_idx_ = key_index_;  // Update for next time

    latest_sonar_pose_ = relative_pose;
    latest_sonar_cov_ = covariance;
    has_new_sonar_ = true;

    ROS_DEBUG("Sonar factor: from node %d to node %d", sonar_from_node_idx_, sonar_to_node_idx_);
}

void SlidingWindowGraph::setInitialPose(const gtsam::Pose3& pose) {
    if (!has_init_pose_) {
        init_pose_ = pose;
        has_init_pose_ = true;
        ROS_INFO("Captured initial ground truth pose: [%.2f, %.2f, %.2f]",
                 pose.x(), pose.y(), pose.z());
    }
}

void SlidingWindowGraph::setInitialVelocity(const gtsam::Vector3& velocity) {
    if (!has_init_velocity_) {
        init_velocity_ = velocity;
        has_init_velocity_ = true;
        ROS_INFO("Captured initial ground truth velocity: [%.2f, %.2f, %.2f]",
                 velocity.x(), velocity.y(), velocity.z());
    }
}

gtsam::Pose3 SlidingWindowGraph::getCurrentPose() const {
    return current_pose_;
}

gtsam::Vector3 SlidingWindowGraph::getCurrentVelocity() const {
    return current_velocity_;
}

gtsam::imuBias::ConstantBias SlidingWindowGraph::getCurrentBias() const {
    return prev_bias_;
}

} // namespace rexrov_fgo_nav