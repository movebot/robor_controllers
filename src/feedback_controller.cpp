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

  // Initialize unit quaternions to be valid
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  ref_odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

bool FeedbackController::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, false);
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

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_ != prev_active) {
    if (p_active_) {
      odom_sub_ = nh_.subscribe("robot_state", 5, &FeedbackController::odomCallback, this);
      ref_odom_sub_ = nh_.subscribe("reference_state", 5, &FeedbackController::refOdomCallback, this);
      controls_pub_ = nh_.advertise<geometry_msgs::Twist>("controls", 5);
    }
    else {
      geometry_msgs::TwistPtr controls_msg(new geometry_msgs::Twist);
      controls_pub_.publish(controls_msg);

      odom_sub_.shutdown();
      ref_odom_sub_.shutdown();
      controls_pub_.shutdown();
    }
  }

  if (p_active_ && !p_run_) {
    geometry_msgs::TwistPtr controls_msg(new geometry_msgs::Twist);
    controls_pub_.publish(controls_msg);
  }

  if (p_active_ && p_run_)
    timer_.start();
  else
    timer_.stop();

  return true;
}

void FeedbackController::timerCallback(const ros::TimerEvent& e) {
  /*
   * The following variables are described in the world coordinate system
   */
  double x = odom_.pose.pose.position.x;
  double y = odom_.pose.pose.position.y;
  double theta = tf::getYaw(odom_.pose.pose.orientation);

  double v_x = odom_.twist.twist.linear.x;
  double v_y = odom_.twist.twist.linear.y;
  double w_z = odom_.twist.twist.angular.z;

  double x_r = ref_odom_.pose.pose.position.x;
  double y_r = ref_odom_.pose.pose.position.y;
  double theta_r = tf::getYaw(ref_odom_.pose.pose.orientation);

  double v_x_r = ref_odom_.twist.twist.linear.x;
  double v_y_r = ref_odom_.twist.twist.linear.y;
  double w_z_r = ref_odom_.twist.twist.angular.z;

  /*
   * The control signals must be described in the base coordinate system
   */
  double u = 0.0; // Forward linear velocity
  double v = 0.0; // Sideways linear velocity
  double w = 0.0; // Angular velocity

  double e_x = x_r - x;
  double e_y = y_r - y;
  double e_theta = theta_r - theta;

  e_theta = atan2(sin(e_theta), cos(e_theta));

  double x_p = p_gain_x_ * e_x;
  double y_p = p_gain_y_ * e_y;
  double theta_p = p_gain_theta_ * e_theta;

  if (p_use_ff_) {
    x_p += v_x_r;
    y_p += v_y_r;
    theta_p += w_z_r;
  }

  u =  x_p * cos(theta) + y_p * sin(theta);
  v = -x_p * sin(theta) + y_p * cos(theta);
  w = theta_p;

  scaleControls(u, v, w);

  geometry_msgs::TwistPtr controls_msg(new geometry_msgs::Twist);

  controls_msg->linear.x = u;
  controls_msg->linear.y = v;
  controls_msg->angular.z = w;

  controls_pub_.publish(controls_msg);
}

void FeedbackController::odomCallback(const nav_msgs::Odometry::ConstPtr odom_msg) {
  odom_ = *odom_msg;
}

void FeedbackController::refOdomCallback(const nav_msgs::Odometry::ConstPtr ref_odom_msg) {
  ref_odom_ = *ref_odom_msg;
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
