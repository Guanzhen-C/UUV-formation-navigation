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

#include <uuv_sensor_ros_plugins/RPTROSPlugin.hh>
#include <algorithm>

namespace gazebo
{
/////////////////////////////////////////////////
RPTROSPlugin::RPTROSPlugin() : ROSBaseModelPlugin()
{ }

/////////////////////////////////////////////////
RPTROSPlugin::~RPTROSPlugin()
{ }

/////////////////////////////////////////////////
void RPTROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  double variance = this->noiseSigma * this->noiseSigma;
  for (int i = 0; i < 9; i++)
    this->rosMessage.pos.covariance[i] = 0;

  this->rosMessage.pos.covariance[0] = this->rosMessage.pos.covariance[4] =
      this->rosMessage.pos.covariance[8] = variance;

  // Initialize the default RPT output
  this->rosSensorOutputPub =
    this->rosNode->advertise<
      uuv_sensor_ros_plugins_msgs::PositionWithCovarianceStamped>(
        this->sensorOutputTopic, 1);

  if (this->gazeboMsgEnabled)
  {
    this->gazeboSensorOutputPub =
      this->gazeboNode->Advertise<sensor_msgs::msgs::Rpt>(
        this->robotNamespace + "/" + this->sensorOutputTopic, 1);
  }
  // ----- Acoustic parameters (OWTT/Method3) -----
  GetSDFParam<double>(_sdf, "sound_speed", this->sound_speed_, 1500.0);
  GetSDFParam<double>(_sdf, "max_range", this->max_range_, 12000.0);
  GetSDFParam<double>(_sdf, "range_noise_sigma", this->range_noise_sigma_, 0.0);
  // Proportional noise parameters (optional)
  GetSDFParam<double>(_sdf, "range_noise_sigma_ratio", this->range_noise_sigma_ratio_, 0.0);
  GetSDFParam<double>(_sdf, "range_noise_sigma_min", this->range_noise_sigma_min_, 0.0);
  GetSDFParam<bool>(_sdf, "range_noise_additive", this->range_noise_additive_, true);
  GetSDFParam<double>(_sdf, "range_noise_clip_ratio", this->range_noise_clip_ratio_, 0.0);
  bool advertise_enabled = false, subscribe_enabled = false, processing_enabled = false;
  GetSDFParam<bool>(_sdf, "advertise_enabled", advertise_enabled, false);
  GetSDFParam<bool>(_sdf, "subscribe_enabled", subscribe_enabled, false);
  GetSDFParam<bool>(_sdf, "processing_enabled", processing_enabled, false);
  std::string default_target_ns;
  GetSDFParam<std::string>(_sdf, "default_target_ns", default_target_ns, "");
  GetSDFParam<double>(_sdf, "auto_repeat_period", this->auto_repeat_period_, 0.0);
  // Allow runtime override via ROS param under sensor topic namespace
  if (this->rosNode)
  {
    std::string ar_key = this->sensorOutputTopic + std::string("/auto_repeat_period");
    this->rosNode->param(ar_key, this->auto_repeat_period_, this->auto_repeat_period_);
  }
  // Fallback default to 5.0s if not provided
  if (this->auto_repeat_period_ <= 0.0)
    this->auto_repeat_period_ = 5.0;
  this->advertise_enabled_ = advertise_enabled;
  this->subscribe_enabled_ = subscribe_enabled;
  this->processing_enabled_ = processing_enabled;

  if (this->advertise_enabled_)
    this->owtt_pub_ = this->rosNode->advertise<uuv_sensor_ros_plugins_msgs::AcousticRangeOWTT>(this->sensorOutputTopic + "/range_owtt", 1);
  if (this->advertise_enabled_)
    this->broadcast_pub_ = this->rosNode->advertise<uuv_sensor_ros_plugins_msgs::AcousticBroadcastMethod3>(this->sensorOutputTopic + "/broadcast_method3", 10);
  // Subscribe to Method3 sender state published by local ESKF for embedding in OWTT broadcast
  if (this->rosNode)
    this->method3_state_sub_ = this->rosNode->subscribe(this->sensorOutputTopic + "/method3_sender_state", 10, &RPTROSPlugin::onMethod3State, this);
  // Subscribe to OWTT Method3 broadcast from peers
  if (this->rosNode && !default_target_ns.empty())
    this->broadcast_sub_ = this->rosNode->subscribe(std::string("/") + default_target_ns + "/" + this->sensorOutputTopic + "/broadcast_method3", 50, &RPTROSPlugin::onBroadcastMethod3, this);

  // Store default target namespace for subscribing to peer broadcasts
  this->default_target_ns_ = default_target_ns;

  // TX enable gating: default true, override by param and topic
  if (this->rosNode)
  {
    std::string tx_key = this->sensorOutputTopic + std::string("/tx_enable");
    this->rosNode->param(tx_key, this->tx_enable_, true);
    this->last_tx_enable_state_ = this->tx_enable_;
    this->tx_enable_sub_ = this->rosNode->subscribe<std_msgs::Bool>(this->sensorOutputTopic + "/tx_enable", 1, &RPTROSPlugin::onTxEnable, this);
    // Allow overriding time-proximity dedup threshold
    std::string rx_sep_key = this->sensorOutputTopic + std::string("/rx_min_separation_sec");
    this->rosNode->param(rx_sep_key, this->rx_min_separation_sec_, this->rx_min_separation_sec_);
  }
}

/////////////////////////////////////////////////
bool RPTROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Publish sensor state
  this->PublishState();

  // (TWTT processing removed)

  // Rate gate only for broadcast and position publishing; keep RX processing continuous
  bool meas_allowed = this->EnableMeasurement(_info);

  // (TWTT trigger removed)

  // Broadcast-only: publish Method3 payload at update rate; receiver will compute arrival by zero-crossing and publish range_owtt.
  bool allow_rate_broadcast = (meas_allowed && this->advertise_enabled_ && this->IsOn() && this->has_method3_state_ && this->tx_enable_);
  bool allow_pulse_broadcast = (this->pending_pulse_broadcast_ && this->advertise_enabled_ && this->IsOn() && this->has_method3_state_ && this->tx_enable_);
  if (allow_rate_broadcast || allow_pulse_broadcast)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    double sim_t = _info.simTime.Double();
#else
    double sim_t = _info.simTime.Double();
#endif
    // If pulse requested, bypass rate gate once; otherwise respect update rate
    if (allow_pulse_broadcast || this->last_broadcast_sim_time_ < 0.0 || sim_t > this->last_broadcast_sim_time_)
    {
      uuv_sensor_ros_plugins_msgs::AcousticBroadcastMethod3 b;
      b.from_ns = this->robotNamespace;
      b.t_tx.fromSec(sim_t);
      b.tx_pos = this->last_method3_state_.pos;
      b.tx_cross_cov_x_p = this->last_method3_state_.cross_cov_x_p;
      if (this->broadcast_pub_)
        this->broadcast_pub_.publish(b);
      // Consume rate budget as if a regular broadcast happened
      this->last_broadcast_sim_time_ = sim_t;
      this->pending_pulse_broadcast_ = false;
      // Also advance measurement timer so next meas_allowed respects update_rate
      this->lastMeasurementTime = _info.simTime;
    }
  }

  // True position
  // TODO This is a temporary implementation, next step includes making
  // plugins for acoustic channels and beacons
#if GAZEBO_MAJOR_VERSION >= 8
  this->position = this->link->WorldPose().Pos();
#else
  this->position = this->link->GetWorldPose().Ign().Pos();
#endif

  this->UpdateReferenceFramePose();
  if (this->referenceLink)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    this->referenceFrame = this->referenceLink->WorldPose();
#else
    this->referenceFrame = this->referenceLink->GetWorldPose().Ign();
#endif
  }

  this->position = this->position - this->referenceFrame.Pos();
  this->position = this->referenceFrame.Rot().RotateVectorReverse(
    this->position);

  this->position.X() += this->GetGaussianNoise(this->noiseAmp);
  this->position.Y() += this->GetGaussianNoise(this->noiseAmp);
  this->position.Z() += this->GetGaussianNoise(this->noiseAmp);

  this->rosMessage.header.stamp = ros::Time::now();
  this->rosMessage.header.frame_id = this->referenceFrameID;
  this->rosMessage.pos.pos.x = this->position.X();
  this->rosMessage.pos.pos.y = this->position.Y();
  this->rosMessage.pos.pos.z = this->position.Z();

  if (meas_allowed)
    this->rosSensorOutputPub.publish(this->rosMessage);

  // (TWTT gated processing removed)

  // ---------- OWTT reception processing (zero-crossing timing) ----------
  if (this->IsOn() && !this->pending_owtt_.empty())
  {
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = this->link->WorldPose();
#else
    ignition::math::Pose3d pose = this->link->GetWorldPose().Ign();
#endif
    double t = _info.simTime.Double();
    // Maintain self pose buffer
    this->self_pose_buf_.push_back({t, pose.Pos()});
    while (!this->self_pose_buf_.empty() && t - this->self_pose_buf_.front().t > this->self_buf_horizon_)
      this->self_pose_buf_.pop_front();

    // Clean processed_keys_ by horizon
    while (!this->processed_keys_.empty() && t - this->processed_keys_.front().second > this->processed_horizon_)
      this->processed_keys_.pop_front();

    for (auto &q : this->pending_owtt_)
    {
      // Skip if this (from_ns,t_tx) was already processed
      bool already = false;
      for (const auto &k : this->processed_keys_)
      {
        if (k.first == q.from_ns && std::abs(k.second - q.t_tx) < 1e-6) { already = true; break; }
      }
      if (already) continue;
      // zero-crossing on diff = c*(t - t_tx) - ||p_self(t) - p_tx(t_tx)||
      double cdt = this->sound_speed_ * (t - q.t_tx);
      double rng = (pose.Pos() - q.p_tx_ttx).Length();
      double diff = cdt - rng;
      if (!q.has_prev) { q.has_prev = true; q.prev_t = t; q.prev_diff = diff; continue; }
      bool crossed = (q.prev_diff <= 0.0 && diff >= 0.0) || (q.prev_diff >= 0.0 && diff <= 0.0);
      if (crossed)
      {
        double denom = (diff - q.prev_diff);
        double alpha = (std::abs(denom) > 1e-9) ? (-q.prev_diff) / denom : 0.0;
        alpha = std::min(std::max(alpha, 0.0), 1.0);
        double t_rx = q.prev_t + alpha * (t - q.prev_t);
        ignition::math::Vector3d p_self_trx = InterpPose(this->self_pose_buf_, t_rx);
        double geom_range = (p_self_trx - q.p_tx_ttx).Length();

        // Build measurement with optional proportional Gaussian noise
        double sigma = this->range_noise_sigma_;
        if (this->range_noise_sigma_ratio_ > 0.0)
          sigma = this->range_noise_sigma_ratio_ * geom_range;
        if (this->range_noise_sigma_min_ > 0.0)
          sigma = std::max(sigma, this->range_noise_sigma_min_);

        double measured_range = geom_range;
        if (this->range_noise_additive_ && sigma > 0.0)
        {
          std::normal_distribution<double> dist(0.0, sigma);
          measured_range += dist(this->rndGen);
        }
        if (this->range_noise_clip_ratio_ > 0.0)
        {
          double clip_abs = this->range_noise_clip_ratio_ * geom_range;
          double min_allowed = std::max(0.0, geom_range - clip_abs);
          double max_allowed = geom_range + clip_abs;
          if (measured_range < min_allowed) measured_range = min_allowed;
          if (measured_range > max_allowed) measured_range = max_allowed;
        }
        if (measured_range < 0.0) measured_range = 0.0;

        // Receiver-side time proximity dedup: suppress if too close to last publish from same sender
        double last_trx = -1.0;
        auto it_last = this->last_rx_trx_by_sender_.find(q.from_ns);
        if (it_last != this->last_rx_trx_by_sender_.end()) last_trx = it_last->second;
        bool separated_ok = (last_trx < 0.0) || ((t_rx - last_trx) >= this->rx_min_separation_sec_);

        if (separated_ok && this->owtt_pub_)
        {
          uuv_sensor_ros_plugins_msgs::AcousticRangeOWTT owtt;
          owtt.from_ns = q.from_ns;
          owtt.to_ns = this->robotNamespace;
          owtt.t_tx.fromSec(q.t_tx);
          owtt.t_rx.fromSec(t_rx);
          owtt.range = measured_range;
          owtt.variance = sigma * sigma;
          owtt.tx_pos = q.tx_pos;
          owtt.tx_cross_cov_x_p = q.tx_cross_cov_x_p;
          this->owtt_pub_.publish(owtt);
          // Mark processed to avoid duplicates
          this->processed_keys_.push_back({q.from_ns, q.t_tx});
          this->last_rx_trx_by_sender_[q.from_ns] = t_rx;
        }
      }
      q.prev_t = t; q.prev_diff = diff;
    }
    // Remove processed items whose t_tx is older than current time minus horizon
    while (!this->pending_owtt_.empty() && (t - this->pending_owtt_.front().t_tx) > this->processed_horizon_)
      this->pending_owtt_.pop_front();
  }

  if (this->gazeboMsgEnabled)
  {
    sensor_msgs::msgs::Rpt gazeboMessage;
    double variance = this->noiseSigma * this->noiseSigma;
    // Prepare constant covariance part of message
    for (int i = 0 ; i < 9; i++)
    {
      if (i == 0 || i == 4 || i == 8)
        gazeboMessage.add_position_covariance(variance);
      else
        gazeboMessage.add_position_covariance(0.0);
    }
    // Publish simulated measurement
    gazebo::msgs::Vector3d * p = new gazebo::msgs::Vector3d();
    p->set_x(this->position.X());
    p->set_y(this->position.Y());
    p->set_z(this->position.Z());

    gazeboMessage.set_allocated_position(p);
    this->gazeboSensorOutputPub->Publish(gazeboMessage);
  }

  // Update measurement time only when throttled outputs are produced
#if GAZEBO_MAJOR_VERSION >= 8
  if (meas_allowed) this->lastMeasurementTime = _info.simTime;
#else
  if (meas_allowed) this->lastMeasurementTime = _info.simTime;
#endif
  return true;
}

// (TWTT tx request handler removed)

/////////////////////////////////////////////////
void RPTROSPlugin::onMethod3State(const uuv_sensor_ros_plugins_msgs::Method3SenderState& _msg)
{
  this->last_method3_state_ = _msg;
  this->has_method3_state_ = true;
}

/////////////////////////////////////////////////
void RPTROSPlugin::onBroadcastMethod3(const uuv_sensor_ros_plugins_msgs::AcousticBroadcastMethod3& _msg)
{
  // Enqueue an OWTT pending item with sender's t_tx and position at t_tx
  PendingOwtt q;
  q.from_ns = _msg.from_ns;
  q.t_tx = _msg.t_tx.toSec();
  q.tx_pos = _msg.tx_pos;
  q.tx_cross_cov_x_p = _msg.tx_cross_cov_x_p; // boost::array gets copied
  // Note: tx_pos is a PositionWithCovarianceStamped; use its position now (assumed at t_tx)
  q.p_tx_ttx = ignition::math::Vector3d(_msg.tx_pos.pos.pos.x, _msg.tx_pos.pos.pos.y, _msg.tx_pos.pos.pos.z);
  q.has_prev = false; q.prev_t = 0.0; q.prev_diff = 0.0;
  this->pending_owtt_.push_back(q);
}

/////////////////////////////////////////////////
void RPTROSPlugin::onTxEnable(const std_msgs::Bool::ConstPtr& _msg)
{
  bool new_state = _msg->data;
  // Rising edge triggers one immediate broadcast pulse
  if (!this->last_tx_enable_state_ && new_state)
    this->pending_pulse_broadcast_ = true;
  this->tx_enable_ = new_state;
  this->last_tx_enable_state_ = new_state;
}

/////////////////////////////////////////////////
ignition::math::Vector3d RPTROSPlugin::InterpPose(const std::deque<TimedPose>& _buf, double _t)
{
  if (_buf.empty()) return ignition::math::Vector3d::Zero;
  if (_t <= _buf.front().t) return _buf.front().p;
  if (_t >= _buf.back().t) return _buf.back().p;
  for (size_t i = 1; i < _buf.size(); ++i)
  {
    if (_buf[i].t >= _t)
    {
      const auto& a = _buf[i-1]; const auto& b = _buf[i];
      double alpha = (_t - a.t) / (b.t - a.t);
      return a.p + (b.p - a.p) * alpha;
    }
  }
  return _buf.back().p;
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(RPTROSPlugin)
}
