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

ManualController::~ManualController() {
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("publish_reference_twist");

  nh_local_.deleteParam("use_joy");
  nh_local_.deleteParam("use_keys");

  nh_local_.deleteParam("loop_rate");
  nh_local_.deleteParam("time_constant");

  nh_local_.deleteParam("linear_gain");
  nh_local_.deleteParam("angular_gain");
}

bool ManualController::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("publish_reference_twist", p_pub_reference_twist_, false);

  p_pub_topic_ = (p_pub_reference_twist_) ? (string("reference_twist")) : (string("controls"));

  nh_local_.param<bool>("use_joy", p_use_joy_, false);
  nh_local_.param<bool>("use_keys", p_use_keys_, false);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;
  nh_local_.param<double>("time_constant", p_time_constant_, 0.0);

  nh_local_.param<double>("linear_gain", p_linear_gain_, 0.3);
  nh_local_.param<double>("angular_gain", p_angular_gain_, 0.5);

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_ != prev_active) {
    if (p_active_) {
      joy_sub_ = nh_.subscribe("joy", 5, &ManualController::joyCallback, this, ros::TransportHints().udp());
      keys_sub_ = nh_.subscribe("keys", 5, &ManualController::keysCallback, this, ros::TransportHints().udp());
      twist_pub_ = nh_.advertise<geometry_msgs::Twist>(p_pub_topic_, 10);
    }
    else {
      geometry_msgs::TwistPtr twist_msg(new geometry_msgs::Twist);
      twist_pub_.publish(twist_msg);

      joy_sub_.shutdown();
      keys_sub_.shutdown();
      twist_pub_.shutdown();
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
    twist_.linear.x = p_linear_gain_ * joy_msg->axes[1];
    twist_.linear.y = p_linear_gain_ * joy_msg->axes[0];
    twist_.angular.z = p_angular_gain_ * joy_msg->axes[3];
  }
}

void ManualController::keysCallback(const geometry_msgs::Twist::ConstPtr keys_msg) {
  if (p_use_keys_) {
    twist_.linear.x = p_linear_gain_ * keys_msg->linear.x;
    twist_.linear.y = p_linear_gain_ * keys_msg->linear.y;
    twist_.angular.z = p_angular_gain_ * keys_msg->angular.z;
  }
}

void ManualController::timerCallback(const ros::TimerEvent& e) {
  static geometry_msgs::TwistPtr twist_msg(new geometry_msgs::Twist);

  twist_msg->linear.x  += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (twist_.linear.x - twist_msg->linear.x);
  twist_msg->linear.y  += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (twist_.linear.y - twist_msg->linear.y);
  twist_msg->angular.z += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (twist_.angular.z - twist_msg->angular.z);

  twist_pub_.publish(twist_msg);
}
