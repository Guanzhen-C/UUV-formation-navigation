// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __UUV_RPT_ROS_PLUGIN_HH__
#define __UUV_RPT_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped.h>
#include <uuv_sensor_ros_plugins_msgs/AcousticRangeOWTT.h>
#include <uuv_sensor_ros_plugins_msgs/Method3SenderState.h>
#include <uuv_sensor_ros_plugins_msgs/AcousticBroadcastMethod3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include "SensorRpt.pb.h"
#include <deque>
#include <boost/array.hpp>
#include <map>

namespace gazebo
{
  class RPTROSPlugin :  public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: RPTROSPlugin();

    /// \brief Class destructor
    public: virtual ~RPTROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Latest measured position.
    protected: ignition::math::Vector3d position;

    /// \brief Store message since many attributes do not change (cov.).
    protected: uuv_sensor_ros_plugins_msgs::PositionWithCovarianceStamped rosMessage;

    // ---------------- OWTT broadcast only ----------------
    protected: ros::Publisher owtt_pub_;
    protected: ros::Publisher broadcast_pub_;
    protected: ros::Subscriber broadcast_sub_;
    protected: ros::Subscriber method3_state_sub_;

    protected: bool advertise_enabled_ = false;
    protected: bool subscribe_enabled_ = false; // kept for backward compat, unused
    protected: bool processing_enabled_ = false; // kept for backward compat, unused
    protected: double auto_repeat_period_ = 0.0; // kept for backward compat, unused
    protected: double last_auto_tx_sim_time_ = -1.0; // kept for backward compat, unused

    protected: double sound_speed_ = 1500.0;
    protected: double max_range_ = 120000.0;
    protected: std::string target_link_name_ = "base_link";
    protected: std::string default_target_ns_;
    protected: double range_noise_sigma_ = 0.0; // optional additive noise on range
    // Proportional range noise parameters (sigma = max(min, ratio * range))
    protected: double range_noise_sigma_ratio_ = 0.0;
    protected: double range_noise_sigma_min_ = 0.0;
    protected: bool range_noise_additive_ = true;
    protected: double range_noise_clip_ratio_ = 0.0; // if > 0, clip |measured-range| <= clip_ratio * range

    protected: struct TimedPose { double t; ignition::math::Vector3d p; };
    protected: std::deque<TimedPose> self_pose_buf_;
    protected: double self_buf_horizon_ = 120.0;

    // ---------------- OWTT broadcast reception (one-way) ----------------
    protected: struct PendingOwtt { 
      std::string from_ns; 
      double t_tx = 0.0; 
      ignition::math::Vector3d p_tx_ttx; 
      // Payload to embed on Rx publish
      uuv_sensor_ros_plugins_msgs::PositionWithCovarianceStamped tx_pos;
      boost::array<double, 45> tx_cross_cov_x_p; // fixed-size array
      bool has_prev = false; double prev_t = 0.0; double prev_diff = 0.0; 
    };
    protected: std::deque<PendingOwtt> pending_owtt_;
    // Track processed keys to avoid duplicate publish on small-step sign flips
    protected: std::deque<std::pair<std::string,double>> processed_keys_;
    protected: double processed_horizon_ = 180.0; // seconds to remember keys

    // Time-proximity deduplication on receiver: suppress publishes closer than threshold per sender
    protected: std::map<std::string, double> last_rx_trx_by_sender_;
    protected: double rx_min_separation_sec_ = 1.0; // can be overridden via ROS param
    protected: double last_broadcast_sim_time_ = -1.0;

    // Remove TWTT pending queue and tx request handling
    protected: static ignition::math::Vector3d InterpPose(const std::deque<TimedPose>& buf, double t);
    protected: void onMethod3State(const uuv_sensor_ros_plugins_msgs::Method3SenderState& msg);
    protected: void onBroadcastMethod3(const uuv_sensor_ros_plugins_msgs::AcousticBroadcastMethod3& msg);

    // Gating for transmission: allow receiving while disabling broadcast when false
    protected: bool tx_enable_ = true;
    protected: ros::Subscriber tx_enable_sub_;
    protected: void onTxEnable(const std_msgs::Bool::ConstPtr& msg);

    // Single-pulse immediate broadcast upon tx_enable rising edge
    protected: bool last_tx_enable_state_ = false;
    protected: bool pending_pulse_broadcast_ = false;

    // Latest Method3 sender state provided by ESKF for embedding into OWTT broadcast
    protected: bool has_method3_state_ = false;
    protected: uuv_sensor_ros_plugins_msgs::Method3SenderState last_method3_state_;
  };
}

#endif // __UUV_RPT_ROS_PLUGIN_HH__
