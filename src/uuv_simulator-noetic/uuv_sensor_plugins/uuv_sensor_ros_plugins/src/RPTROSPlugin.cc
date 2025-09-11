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
  // ----- Optional TWTT acoustic extensions (disabled by default) -----
  GetSDFParam<double>(_sdf, "sound_speed", this->sound_speed_, 1500.0);
  GetSDFParam<double>(_sdf, "max_range", this->max_range_, 12000.0);
  bool advertise_enabled = false, subscribe_enabled = false, processing_enabled = false;
  GetSDFParam<bool>(_sdf, "advertise_enabled", advertise_enabled, false);
  GetSDFParam<bool>(_sdf, "subscribe_enabled", subscribe_enabled, false);
  GetSDFParam<bool>(_sdf, "processing_enabled", processing_enabled, false);
  std::string default_target_ns;
  GetSDFParam<std::string>(_sdf, "default_target_ns", default_target_ns, "");
  this->advertise_enabled_ = advertise_enabled;
  this->subscribe_enabled_ = subscribe_enabled;
  this->processing_enabled_ = processing_enabled;

  if (this->advertise_enabled_)
    this->range_pub_ = this->rosNode->advertise<uuv_sensor_ros_plugins_msgs::AcousticRangeTWTT>(this->sensorOutputTopic + "/range_twtt", 1);
  if (this->subscribe_enabled_)
    this->tx_req_sub_ = this->rosNode->subscribe(this->sensorOutputTopic + "/tx_request", 10, &RPTROSPlugin::onTxRequest, this);

  // If default target provided, schedule a pending exchange automatically
  if (!default_target_ns.empty())
  {
    Pending p; p.target_ns = default_target_ns; p.request_id = "auto";
#if GAZEBO_MAJOR_VERSION >= 8
    p.t0 = this->model->GetWorld()->SimTime().Double();
    p.p1_t0 = this->link->WorldPose().Pos();
#else
    p.t0 = this->model->GetWorld()->GetSimTime().Double();
    p.p1_t0 = this->link->GetWorldPose().Ign().Pos();
#endif
    p.stage = STAGE_FWD_WAIT; p.done = false; p.fwd_has_prev = false; p.bwd_has_prev = false;
    this->pending_.push_back(p);
  }
}

/////////////////////////////////////////////////
bool RPTROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Publish sensor state
  this->PublishState();

  // ---------- Acoustic TWTT processing (optional, without RPT gating) ----------
  if (this->processing_enabled_ && this->IsOn())
  {
    double t = _info.simTime.Double();
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d poseSelf = this->link->WorldPose();
#else
    ignition::math::Pose3d poseSelf = this->link->GetWorldPose().Ign();
#endif
    this->self_pose_buf_.push_back({t, poseSelf.Pos()});
    while (!this->self_pose_buf_.empty() && t - this->self_pose_buf_.front().t > this->self_buf_horizon_)
      this->self_pose_buf_.pop_front();

    for (auto &p : this->pending_)
    {
      if (p.done)
        continue;
      if (!p.target_model)
      {
        physics::WorldPtr world = this->model->GetWorld();
        p.target_model = world->ModelByName(p.target_ns);
        if (p.target_model)
        {
          p.target_link = p.target_model->GetLink(this->target_link_name_);
          if (!p.target_link)
          {
            std::string fq = p.target_model->GetName() + "/" + this->target_link_name_;
            p.target_link = p.target_model->GetLink(fq);
          }
          if (!p.target_link)
          {
            p.target_link = p.target_model->GetLink();
          }
        }
      }
      if (!p.target_model || !p.target_link)
        continue;

      ignition::math::Pose3d p2_pose = p.target_link->WorldPose();
      p.target_buf.push_back({t, p2_pose.Pos()});
      while (!p.target_buf.empty() && t - p.target_buf.front().t > p.target_buf_horizon)
        p.target_buf.pop_front();

      if (p.stage == STAGE_FWD_WAIT)
      {
        double cdt = this->sound_speed_ * (t - p.t0);
        double rng = (p2_pose.Pos() - p.p1_t0).Length();
        double diff = cdt - rng;
        if (!p.fwd_has_prev)
        { p.fwd_has_prev = true; p.fwd_prev_t = t; p.fwd_prev_diff = diff; }
        else
        {
          if ((p.fwd_prev_diff <= 0.0 && diff >= 0.0) || (p.fwd_prev_diff >= 0.0 && diff <= 0.0))
          {
            double denom = (diff - p.fwd_prev_diff);
            double alpha = (std::abs(denom) > 1e-9) ? (-p.fwd_prev_diff) / denom : 0.0;
            alpha = std::min(std::max(alpha, 0.0), 1.0);
            p.t1 = p.fwd_prev_t + alpha * (t - p.fwd_prev_t);
            p.p2_t1 = InterpPose(p.target_buf, p.t1);
            p.stage = STAGE_BWD_WAIT;
          }
          p.fwd_prev_t = t; p.fwd_prev_diff = diff;
        }
      }
      else if (p.stage == STAGE_BWD_WAIT)
      {
        double cdt = this->sound_speed_ * (t - p.t1);
        double rng = (poseSelf.Pos() - p.p2_t1).Length();
        double diff = cdt - rng;
        if (!p.bwd_has_prev)
        { p.bwd_has_prev = true; p.bwd_prev_t = t; p.bwd_prev_diff = diff; }
        else
        {
          if ((p.bwd_prev_diff <= 0.0 && diff >= 0.0) || (p.bwd_prev_diff >= 0.0 && diff <= 0.0))
          {
            double denom = (diff - p.bwd_prev_diff);
            double alpha = (std::abs(denom) > 1e-9) ? (-p.bwd_prev_diff) / denom : 0.0;
            alpha = std::min(std::max(alpha, 0.0), 1.0);
            double t2 = p.bwd_prev_t + alpha * (t - p.bwd_prev_t);
            ignition::math::Vector3d p1_t2 = InterpPose(this->self_pose_buf_, t2);
            double r1 = (p1_t2 - p.p2_t1).Length();

            uuv_sensor_ros_plugins_msgs::AcousticRangeTWTT out;
            out.from_ns = this->robotNamespace;
            out.to_ns = p.target_ns;
            out.t0.fromSec(p.t0);
            out.t1.fromSec(p.t1);
            out.t2.fromSec(t2);
            out.r1 = r1;
            out.success = (r1 <= this->max_range_);
            out.variance = 1.0;
            out.peer_pose_t1.header.stamp = out.t1;
            out.peer_pose_t1.header.frame_id = "world";
            out.peer_pose_t1.pose.position.x = p.p2_t1.X();
            out.peer_pose_t1.pose.position.y = p.p2_t1.Y();
            out.peer_pose_t1.pose.position.z = p.p2_t1.Z();
            out.peer_pose_t1.pose.orientation.w = 1.0;
            if (this->range_pub_) this->range_pub_.publish(out);
            p.done = true;
          }
          p.bwd_prev_t = t; p.bwd_prev_diff = diff;
        }
      }
    }
    this->pending_.erase(std::remove_if(this->pending_.begin(), this->pending_.end(), [](const Pending& x){return x.done;}), this->pending_.end());
  }

  if (!this->EnableMeasurement(_info))
    return false;

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

  this->rosSensorOutputPub.publish(this->rosMessage);

  // ---------- Acoustic TWTT processing (optional, gated) ----------
  if (this->processing_enabled_ && this->IsOn())
  {
    // Respect update rate gate
    // Maintain self pose buffer
    double t = _info.simTime.Double();
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = this->link->WorldPose();
#else
    ignition::math::Pose3d pose = this->link->GetWorldPose().Ign();
#endif
    this->self_pose_buf_.push_back({t, pose.Pos()});
    while (!this->self_pose_buf_.empty() && t - this->self_pose_buf_.front().t > this->self_buf_horizon_)
      this->self_pose_buf_.pop_front();

    for (auto &p : this->pending_)
    {
      if (p.done)
        continue;
      if (!p.target_model)
      {
        physics::WorldPtr world = this->model->GetWorld();
        p.target_model = world->ModelByName(p.target_ns);
        if (p.target_model)
        {
          // Try model-local link name
          p.target_link = p.target_model->GetLink(this->target_link_name_);
          if (!p.target_link)
          {
            // Try fully-qualified "<model>/<link>"
            std::string fq = p.target_model->GetName() + "/" + this->target_link_name_;
            p.target_link = p.target_model->GetLink(fq);
          }
          if (!p.target_link)
          {
            // Fallback to default link
            p.target_link = p.target_model->GetLink();
          }
        }
      }
      if (!p.target_model || !p.target_link)
        continue;

      ignition::math::Pose3d p2_pose = p.target_link->WorldPose();
      p.target_buf.push_back({t, p2_pose.Pos()});
      while (!p.target_buf.empty() && t - p.target_buf.front().t > p.target_buf_horizon)
        p.target_buf.pop_front();

      if (p.stage == STAGE_FWD_WAIT)
      {
        double cdt = this->sound_speed_ * (t - p.t0);
        double rng = (p2_pose.Pos() - p.p1_t0).Length();
        double diff = cdt - rng;
        if (!p.fwd_has_prev)
        {
          p.fwd_has_prev = true; p.fwd_prev_t = t; p.fwd_prev_diff = diff;
        }
        else
        {
          if ((p.fwd_prev_diff <= 0.0 && diff >= 0.0) || (p.fwd_prev_diff >= 0.0 && diff <= 0.0))
          {
            double denom = (diff - p.fwd_prev_diff);
            double alpha = (std::abs(denom) > 1e-9) ? (-p.fwd_prev_diff) / denom : 0.0;
            alpha = std::min(std::max(alpha, 0.0), 1.0);
            p.t1 = p.fwd_prev_t + alpha * (t - p.fwd_prev_t);
            p.p2_t1 = InterpPose(p.target_buf, p.t1);
            p.stage = STAGE_BWD_WAIT;
          }
          p.fwd_prev_t = t; p.fwd_prev_diff = diff;
        }
      }
      else if (p.stage == STAGE_BWD_WAIT)
      {
        double cdt = this->sound_speed_ * (t - p.t1);
        double rng = (pose.Pos() - p.p2_t1).Length();
        double diff = cdt - rng;
        if (!p.bwd_has_prev)
        {
          p.bwd_has_prev = true; p.bwd_prev_t = t; p.bwd_prev_diff = diff;
        }
        else
        {
          if ((p.bwd_prev_diff <= 0.0 && diff >= 0.0) || (p.bwd_prev_diff >= 0.0 && diff <= 0.0))
          {
            double denom = (diff - p.bwd_prev_diff);
            double alpha = (std::abs(denom) > 1e-9) ? (-p.bwd_prev_diff) / denom : 0.0;
            alpha = std::min(std::max(alpha, 0.0), 1.0);
            double t2 = p.bwd_prev_t + alpha * (t - p.bwd_prev_t);
            ignition::math::Vector3d p1_t2 = InterpPose(this->self_pose_buf_, t2);
            double r1 = (p1_t2 - p.p2_t1).Length();

            uuv_sensor_ros_plugins_msgs::AcousticRangeTWTT out;
            out.from_ns = this->robotNamespace;
            out.to_ns = p.target_ns;
            out.t0.fromSec(p.t0);
            out.t1.fromSec(p.t1);
            out.t2.fromSec(t2);
            out.r1 = r1;
            out.success = (r1 <= this->max_range_);
            out.variance = 1.0;
            out.peer_pose_t1.header.stamp = out.t1;
            out.peer_pose_t1.header.frame_id = "world";
            out.peer_pose_t1.pose.position.x = p.p2_t1.X();
            out.peer_pose_t1.pose.position.y = p.p2_t1.Y();
            out.peer_pose_t1.pose.position.z = p.p2_t1.Z();
            out.peer_pose_t1.pose.orientation.w = 1.0;
            if (this->range_pub_) this->range_pub_.publish(out);
            p.done = true;
          }
          p.bwd_prev_t = t; p.bwd_prev_diff = diff;
        }
      }
    }
    // Remove done
    this->pending_.erase(std::remove_if(this->pending_.begin(), this->pending_.end(), [](const Pending& x){return x.done;}), this->pending_.end());
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

  // Ensure non-void function returns a value
  return true;
}

/////////////////////////////////////////////////
void RPTROSPlugin::onTxRequest(const uuv_sensor_ros_plugins_msgs::AcousticTxRequest& _msg)
{
  if (!this->subscribe_enabled_ || !this->IsOn())
    return;
  Pending p; p.target_ns = _msg.target_ns; p.request_id = _msg.request_id;
#if GAZEBO_MAJOR_VERSION >= 8
  p.t0 = this->model->GetWorld()->SimTime().Double();
  p.p1_t0 = this->link->WorldPose().Pos();
#else
  p.t0 = this->model->GetWorld()->GetSimTime().Double();
  p.p1_t0 = this->link->GetWorldPose().Ign().Pos();
#endif
  p.stage = STAGE_FWD_WAIT; p.done = false; p.fwd_has_prev = false; p.bwd_has_prev = false;
  this->pending_.push_back(p);
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
