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
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <obstacle_detector/Obstacles.h>

namespace robor_controllers
{

inline double signum(double x) { return (x < 0.0) ? -1.0 : 1.0; }
inline double abs(double x) { return (x < 0.0) ? -x : x; }
inline double max(double x, double y) { return (x > y) ? x : y; }
inline double squared(double x) { return pow(x, 2.0); }
inline double cubed(double x) { return pow(x, 3.0); }
inline double quaded(double x) { return pow(x, 4.0); }


class Obstacle {
public:
  Obstacle(arma::vec2 point, arma::vec2 vel, double r, double U_s, double U_d) :
    center_(point), velocity_(vel), r_(r), U_s_(U_s), U_d_(U_d) {}

  double rho(const arma::vec2& p) const;
  arma::vec2 rhoGradient(const arma::vec2& p) const;
  arma::mat22 rhoHessian(const arma::vec2& p) const;

  double rawRepellingPotential(const double ro) const;
  double rawRepellingPotential(const arma::vec2& p) const;
  arma::vec2 rawRepellingGradient(const arma::vec2& p) const;
  arma::mat22 rawRepellingHessian(const arma::vec2& p) const;

  double warpDistance(const arma::vec2& warp_vec, const arma::vec2& p, double& range) const;
  arma::vec2 warpDistanceGradient(const arma::vec2& warp_vec) const;
  arma::mat22 warpDistanceHessian() const;

  double warp(const double d, const double D) const;
  double warp(const arma::vec2& warp_vec, const arma::vec2& p) const;
  arma::vec2 warpGradient(const arma::vec2& warp_vec, const arma::vec2& p) const;
  arma::mat22 warpHessian(const arma::vec2& warp_vec, const arma::vec2& p) const;

  double totalWarp(const arma::vec2& p) const;
  double repellingPotential(const arma::vec2& p) const;
  arma::vec2 repellingGradient(const arma::vec2& p) const;
  arma::mat22 repellingHessian(const arma::vec2& p) const;

public:
  static void setParams(double eta, double R) { eta_ = eta; R_ = R; }

  arma::vec2 center_;
  arma::vec2 velocity_;
  double r_;     // Radius of obstacle

  double U_s_;   // Static factor of potential value at obstacle border
  double U_d_;   // Dynamic factor of potential value at obstacle border

  std::vector<arma::vec2> warp_vectors_;

  static double eta_;   // Border value parameter
  static double R_;     // Range constant of potential
};


class PotentialsController
{
public:
  PotentialsController(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool collectData(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void timerCallback(const ros::TimerEvent& e);

  void odomCallback(const nav_msgs::Odometry::ConstPtr odom_msg);
  void refOdomCallback(const nav_msgs::Odometry::ConstPtr ref_odom_msg);
  void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr obstacles_msg);

  void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }

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

  ros::Subscriber odom_sub_;
  ros::Subscriber ref_odom_sub_;
  ros::Subscriber obstacles_sub_;
  ros::Publisher controls_pub_;

  arma::vec2 pose_;
  arma::vec2 velocity_;
  arma::vec2 ref_pose_;
  arma::vec2 ref_velocity_;

  double theta_;
  double omega_;
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
};

} // namespace robor_controllers
