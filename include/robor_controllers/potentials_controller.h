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

#include <armadillo>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <obstacle_detector/Obstacles.h>

#include "robor_controllers/obstacle.h"

namespace robor_controllers
{

inline double signum(double x) { return (x < 0.0) ? -1.0 : 1.0; }
inline double abs(double x) { return (x < 0.0) ? -x : x; }
inline double max(double x, double y) { return (x > y) ? x : y; }
inline double squared(double x) { return pow(x, 2.0); }
inline double cubed(double x) { return pow(x, 3.0); }
inline double quaded(double x) { return pow(x, 4.0); }

class PotentialsController
{
public:
  PotentialsController(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~PotentialsController();

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool collectData(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void timerCallback(const ros::TimerEvent& e);

  void referenceTwistCallback(const geometry_msgs::Twist::ConstPtr reference_twist_msg);
  void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr obstacles_msg);

  void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }
  void sendZeroControls() { static geometry_msgs::Twist::ConstPtr zero_controls(new geometry_msgs::Twist); controls_pub_.publish(zero_controls); }

  int nearestObstacleIndex();
  void prepareObstacles();
  void setWarpVectors(Obstacle& o1, Obstacle& o2);
  void setReferenceWarp(Obstacle& o);
  void scaleControls(double& u, double& v, double& w);  

  double attractingPotential();
  arma::vec2 attractingGradient();
  arma::mat22 attractingHessian();
  arma::vec2 saddleAvoidanceFactor(const Obstacle& o);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::ServiceServer params_srv_;
  ros::ServiceServer collector_srv_;
  ros::Timer timer_;

  tf::TransformListener tf_ls_;

  ros::Subscriber reference_twist_sub_;
  ros::Subscriber obstacles_sub_;

  ros::Publisher controls_pub_;

  arma::vec2 pose_;
  arma::vec2 ref_pose_;
  arma::vec2 ref_velocity_;

  double theta_;
  double ref_theta_;
  double ref_omega_;

  std::vector<Obstacle> obstacles_;

  // Parameters
  bool p_active_;
  bool p_run_;
  bool p_assisted_control_;

  double p_loop_rate_;
  double p_sampling_time_;

  double p_R_;
  double p_eta_;
  double p_delta_;
  double p_ko_velocity_;

  double p_gain_pose_;
  double p_gain_theta_;

  double p_max_u_;
  double p_max_v_;
  double p_max_w_;

  std::string p_fixed_frame_id_;
  std::string p_robot_frame_id_;
  std::string p_reference_frame_id_;
};

} // namespace robor_controllers
