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

#include "robor_controllers/manual_controller.h"

using namespace robor_controllers;
using std::string;

ManualController::ManualController(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  params_srv_ = nh_local_.advertiseService("params", &ManualController::updateParams, this);
  timer_ = nh_.createTimer(ros::Duration(1.0), &ManualController::timerCallback, this, false, false);

  initialize();
}

bool ManualController::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("publish_ref_vel", p_pub_ref_vel_, false);

  nh_local_.param<bool>("use_joy", p_use_joy_, false);
  nh_local_.param<bool>("use_keys", p_use_keys_, false);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;
  nh_local_.param<double>("time_constant", p_time_constant_, 0.0);

  nh_local_.param<double>("linear_gain", p_linear_gain_, 0.3);
  nh_local_.param<double>("angular_gain", p_angular_gain_, 0.5);

  nh_local_.param<string>("parent_frame_id", p_parent_frame_id_, "odom");
  nh_local_.param<string>("child_frame_id", p_child_frame_id_, "reference");

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_ != prev_active) {
    if (p_active_) {
      joy_sub_ = nh_.subscribe("joy", 5, &ManualController::joyCallback, this, ros::TransportHints().udp());
      keys_sub_ = nh_.subscribe("keys", 5, &ManualController::keysCallback, this, ros::TransportHints().udp());
      controls_pub_ = nh_.advertise<geometry_msgs::Twist>("controls", 5);
      odom_pub_ = nh_.advertise<nav_msgs::Odometry>("reference_state", 5);
    }
    else {
      geometry_msgs::TwistPtr controls_msg(new geometry_msgs::Twist);
      controls_pub_.publish(controls_msg);

      joy_sub_.shutdown();
      keys_sub_.shutdown();
      controls_pub_.shutdown();
      odom_pub_.shutdown();
    }
  }

  if (p_active_ && (p_use_joy_ || p_use_keys_))
    timer_.start();
  else
    timer_.stop();

  return true;
}

void ManualController::joyCallback(const sensor_msgs::Joy::ConstPtr joy_msg) {
  if (p_use_joy_) {
    controls_.linear.x = p_linear_gain_ * joy_msg->axes[1];
    controls_.linear.y = p_linear_gain_ * joy_msg->axes[0];
    controls_.angular.z = p_angular_gain_ * joy_msg->axes[3];
  }
}

void ManualController::keysCallback(const geometry_msgs::Twist::ConstPtr keys_msg) {
  if (p_use_keys_) {
    controls_.linear.x = p_linear_gain_ * keys_msg->linear.x;
    controls_.linear.y = p_linear_gain_ * keys_msg->linear.y;
    controls_.angular.z = p_angular_gain_ * keys_msg->angular.z;
  }
}

void ManualController::timerCallback(const ros::TimerEvent& e) {
  static geometry_msgs::TwistPtr controls_msg(new geometry_msgs::Twist);

  controls_msg->linear.x  += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (controls_.linear.x - controls_msg->linear.x);
  controls_msg->linear.y  += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (controls_.linear.y - controls_msg->linear.y);
  controls_msg->angular.z += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (controls_.angular.z - controls_msg->angular.z);

  nav_msgs::OdometryPtr odom_msg(new nav_msgs::Odometry);

  odom_msg->header.stamp = ros::Time::now();
  odom_msg->header.frame_id = p_parent_frame_id_;
  odom_msg->child_frame_id = p_child_frame_id_;

  odom_msg->pose.pose.orientation.w = 1.0;
  odom_msg->twist.twist = *controls_msg;

  if (p_pub_ref_vel_)
    odom_pub_.publish(odom_msg);
  else
    controls_pub_.publish(controls_msg);
}
