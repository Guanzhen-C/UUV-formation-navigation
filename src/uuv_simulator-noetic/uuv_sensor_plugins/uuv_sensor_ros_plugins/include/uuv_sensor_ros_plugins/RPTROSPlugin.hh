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
#include <uuv_sensor_ros_plugins_msgs/AcousticTxRequest.h>
#include <uuv_sensor_ros_plugins_msgs/AcousticRangeTWTT.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "SensorRpt.pb.h"
#include <deque>

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

    // ---------------- TWTT acoustic extensions (optional) ----------------
    protected: ros::Subscriber tx_req_sub_;
    protected: ros::Publisher range_pub_;

    protected: bool advertise_enabled_ = false;
    protected: bool subscribe_enabled_ = false;
    protected: bool processing_enabled_ = false;
    protected: double auto_repeat_period_ = 0.0; // [s], 0 disables periodic auto-TX
    protected: double last_auto_tx_sim_time_ = -1.0;

    protected: double sound_speed_ = 1500.0;
    protected: double max_range_ = 12000.0;
    protected: std::string target_link_name_ = "base_link";
    protected: std::string default_target_ns_;

    protected: struct TimedPose { double t; ignition::math::Vector3d p; };
    protected: std::deque<TimedPose> self_pose_buf_;
    protected: double self_buf_horizon_ = 120.0;

    protected: enum Stage { STAGE_FWD_WAIT=0, STAGE_BWD_WAIT=1 };
    protected: struct Pending {
      std::string target_ns;
      std::string request_id;
      physics::ModelPtr target_model;
      physics::LinkPtr target_link;
      double t0 = 0.0;
      ignition::math::Vector3d p1_t0;
      double t1 = 0.0;
      ignition::math::Vector3d p2_t1;
      Stage stage = STAGE_FWD_WAIT;
      bool done = false;
      std::deque<TimedPose> target_buf;
      double target_buf_horizon = 120.0;
      bool fwd_has_prev = false; double fwd_prev_t = 0.0; double fwd_prev_diff = 0.0;
      bool bwd_has_prev = false; double bwd_prev_t = 0.0; double bwd_prev_diff = 0.0;
    };
    protected: std::deque<Pending> pending_;

    protected: void onTxRequest(const uuv_sensor_ros_plugins_msgs::AcousticTxRequest& msg);
    protected: static ignition::math::Vector3d InterpPose(const std::deque<TimedPose>& buf, double t);
  };
}

#endif // __UUV_RPT_ROS_PLUGIN_HH__
