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

#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

namespace robor_controllers
{

class FeedbackController
{
public:
  FeedbackController(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~FeedbackController();

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void timerCallback(const ros::TimerEvent& e);
  void refTwistCallback(const geometry_msgs::Twist::ConstPtr reference_twist_msg);

  void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }
  void scaleControls(double& u, double& v, double& w);
  void sendZeroControls() { static geometry_msgs::Twist::ConstPtr zero_controls(new geometry_msgs::Twist); controls_pub_.publish(zero_controls); }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::ServiceServer params_srv_;

  ros::Timer timer_;

  tf::TransformListener tf_ls_;

  ros::Subscriber reference_twist_sub_;

  ros::Publisher controls_pub_;

  geometry_msgs::Twist reference_twist_;

  // Parameters
  bool p_active_;
  bool p_run_;
  bool p_use_ff_;

  double p_loop_rate_;
  double p_sampling_time_;

  double p_gain_x_;
  double p_gain_y_;
  double p_gain_theta_;

  double p_max_u_;
  double p_max_v_;
  double p_max_w_;

  std::string p_robot_frame_id_;
  std::string p_reference_frame_id_;
};

} // namespace robor_controllers
