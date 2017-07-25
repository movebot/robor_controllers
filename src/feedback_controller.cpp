/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "robor_controllers/feedback_controller.h"

using namespace robor_controllers;
using namespace std;

FeedbackController::FeedbackController(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  timer_ = nh_.createTimer(ros::Duration(1.0), &FeedbackController::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &FeedbackController::updateParams, this);

  initialize();
}

FeedbackController::~FeedbackController() {
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("run");
  nh_local_.deleteParam("use_ff");

  nh_local_.deleteParam("loop_rate");

  nh_local_.deleteParam("gain_x");
  nh_local_.deleteParam("gain_y");
  nh_local_.deleteParam("gain_theta");

  nh_local_.deleteParam("max_u");
  nh_local_.deleteParam("max_v");
  nh_local_.deleteParam("max_w");

  nh_local_.deleteParam("robot_frame_id");
  nh_local_.deleteParam("reference_frame_id");
}

bool FeedbackController::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("run", p_run_, false);
  nh_local_.param<bool>("use_ff", p_use_ff_, true);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;

  nh_local_.param<double>("gain_x", p_gain_x_, 0.5);
  nh_local_.param<double>("gain_y", p_gain_y_, 0.5);
  nh_local_.param<double>("gain_theta", p_gain_theta_, 0.5);

  nh_local_.param<double>("max_u", p_max_u_, 0.5);
  nh_local_.param<double>("max_v", p_max_v_, 0.5);
  nh_local_.param<double>("max_w", p_max_w_, 3.0);

  nh_local_.param<string>("robot_frame_id", p_robot_frame_id_, string("robot"));
  nh_local_.param<string>("reference_frame_id", p_reference_frame_id_, string("reference"));

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_ != prev_active) {
    if (p_active_) {
      reference_twist_sub_ = nh_.subscribe("reference_twist", 5, &FeedbackController::refTwistCallback, this);
      controls_pub_ = nh_.advertise<geometry_msgs::Twist>("controls", 5);
    }
    else {
      sendZeroControls();
      reference_twist_sub_.shutdown();
      controls_pub_.shutdown();
    }
  }

  if (p_active_ && !p_run_)
    sendZeroControls();

  if (p_active_ && p_run_)
    timer_.start();
  else
    timer_.stop();

  return true;
}

void FeedbackController::timerCallback(const ros::TimerEvent& e) {
  ros::Time now = ros::Time::now();

  tf::StampedTransform error_tf;
  try {
    tf_ls_.waitForTransform(p_robot_frame_id_, p_reference_frame_id_, now, ros::Duration(0.01));
    tf_ls_.lookupTransform(p_robot_frame_id_, p_reference_frame_id_, now, error_tf);
  }
  catch (tf::TransformException ex) { error_tf.setIdentity(); }

  double e_x = error_tf.getOrigin().x();
  double e_y = error_tf.getOrigin().y();
  double e_theta = tf::getYaw(error_tf.getRotation());

  e_theta = atan2(sin(e_theta), cos(e_theta));

  double u = p_gain_x_ * e_x; // Forward linear velocity
  double v = p_gain_y_ * e_y; // Sideways linear velocity
  double w = p_gain_theta_ * e_theta; // Angular velocity

  if (p_use_ff_) {
    u += reference_twist_.linear.x * cos(e_theta);
    v += reference_twist_.linear.y * sin(e_theta);
    w += reference_twist_.angular.z;
  }

  scaleControls(u, v, w);

  geometry_msgs::TwistPtr controls_msg(new geometry_msgs::Twist);

  controls_msg->linear.x = u;
  controls_msg->linear.y = v;
  controls_msg->angular.z = w;

  controls_pub_.publish(controls_msg);
}

void FeedbackController::refTwistCallback(const geometry_msgs::Twist::ConstPtr reference_twist_msg) {
  reference_twist_ = *reference_twist_msg;
}

void FeedbackController::scaleControls(double& u, double& v, double& w) {
  double s = 1.0;

  if (fabs(u) / p_max_u_ > s)
    s = fabs(u) / p_max_u_;
  if (fabs(v) / p_max_v_ > s)
    s = fabs(v) / p_max_v_;
  if (fabs(w) / p_max_w_ > s)
    s = fabs(w) / p_max_w_;

  u /= s;
  v /= s;
  w /= s;
}
