#ifndef REXROV_FGO_NAV_SLIDING_WINDOW_GRAPH_H
#define REXROV_FGO_NAV_SLIDING_WINDOW_GRAPH_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

#include <ros/ros.h>
#include <deque>
#include <map>
#include <mutex>

namespace rexrov_fgo_nav {

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Velocity (vx,vy,vz)
using gtsam::symbol_shorthand::B; // Bias (ax,ay,az,gx,gy,gz)

class SlidingWindowGraph {
public:
    SlidingWindowGraph(ros::NodeHandle& nh);
    ~SlidingWindowGraph() = default;

    void addImuMeasurement(const gtsam::Vector3& acc, const gtsam::Vector3& gyro, double dt);
    void addImuOrientation(const gtsam::Rot3& orientation);
    void addDvlMeasurement(const gtsam::Vector3& velocity, double timestamp);
    void addDepthMeasurement(double depth, double timestamp);
    void addSonarRelativePose(const gtsam::Pose3& relative_pose, const gtsam::Matrix6& covariance, double timestamp);
    void setInitialPose(const gtsam::Pose3& pose);
    void setInitialVelocity(const gtsam::Vector3& velocity);

    // Main update loop called when a keyframe/step is finished
    void update();

    gtsam::Pose3 getCurrentPose() const;
    gtsam::Vector3 getCurrentVelocity() const;
    gtsam::imuBias::ConstantBias getCurrentBias() const;

private:
    void initialize(double timestamp);

    ros::NodeHandle nh_;

    // ISAM2 for incremental optimization (keeps full history, better accuracy)
    std::shared_ptr<gtsam::ISAM2> isam2_;

    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimates_;

    // IMU Preintegration
    boost::shared_ptr<gtsam::PreintegrationParams> preintegration_params_;
    boost::shared_ptr<gtsam::PreintegratedImuMeasurements> current_preintegration_;
    gtsam::imuBias::ConstantBias prev_bias_;

    // State tracking
    int key_index_ = 0;
    bool initialized_ = false;
    double last_key_time_ = 0.0;
    double current_time_ = 0.0;  // Current timestamp for fixed-lag smoother

    // Measurements buffer (waiting to be added to graph at next keyframe)
    // Note: FGO typically adds factors at "Keyframes" or at regular intervals.
    // Since IMU is high rate, we integrate it between keyframes.
    // DVL/Depth are lower rate, we can add them as factors on the corresponding nodes.
    
    // Latest State Estimates
    gtsam::Pose3 current_pose_;
    gtsam::Vector3 current_velocity_;

    // Buffers for async measurements
    gtsam::Vector3 latest_dvl_;
    bool has_new_dvl_ = false;
    
    double latest_depth_ = 0.0;
    bool has_new_depth_ = false;
    double init_depth_ = 0.0;
    bool has_init_depth_ = false;
    gtsam::Rot3 init_orientation_;
    bool has_init_orientation_ = false;
    gtsam::Pose3 init_pose_;
    bool has_init_pose_ = false;
    gtsam::Vector3 init_velocity_;
    bool has_init_velocity_ = false;

    // IMU orientation for attitude constraint
    gtsam::Rot3 latest_imu_orientation_;
    bool has_new_imu_orientation_ = false;

    gtsam::Pose3 latest_sonar_pose_;
    gtsam::Matrix6 latest_sonar_cov_;
    bool has_new_sonar_ = false;
    int last_sonar_node_idx_ = 0;      // FGO node index when last sonar keyframe was captured
    int sonar_from_node_idx_ = 0;      // FGO node index corresponding to the "from" keyframe
    int sonar_to_node_idx_ = 0;        // FGO node index corresponding to the "to" keyframe
    bool first_sonar_keyframe_ = true; // Flag for first keyframe

    // Noise models
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr velocity_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr depth_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr bias_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr orientation_noise_;
    
    std::mutex graph_mutex_;
};

} // namespace rexrov_fgo_nav

#endif // REXROV_FGO_NAV_SLIDING_WINDOW_GRAPH_H
