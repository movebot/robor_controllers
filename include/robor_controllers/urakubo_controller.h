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
 * Authors: Mateusz Przybyla and Wojciech Kowalczyk and Xi Zhang
 */

#pragma once

#include <armadillo>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <obstacle_detector/Obstacles.h>

namespace robor_controllers
{

struct Ubstacle {
  Ubstacle() : x(0.0), y(0.0), r(0.0) {}
  double x, y, r;
};

class UrakuboController
{
public:
  UrakuboController(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void timerCallback(const ros::TimerEvent& e);
  void odomCallback(const nav_msgs::Odometry::ConstPtr odom_msg);
  void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr obstacles_msg);

  void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }
  void computeControls();

  arma::vec saddleAvoidance(arma::vec min_eigen_vec, const arma::vec eigen_vals);
  bool detectSaddle(arma::vec& min_eigen_vec, arma::vec& eig_vals);

  double squaredNormR();

  double betaWorld();                       
  double betaObstacle(const Ubstacle& o);
  double beta();                            

  arma::vec gradBetaWorld();                      
  arma::vec gradBetaObstacle(const Ubstacle& o);
  arma::vec gradBeta();                           

  arma::mat hessBetaWorld();
  arma::mat hessBetaObstacle(const Ubstacle& o);
  arma::mat hessBeta();

  double C();
  arma::vec gradC();
  arma::mat hessC();

  double V();            // Navigation function
  arma::vec gradV(); 
  arma::mat hessV();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::ServiceServer params_srv_;
  ros::Timer timer_;

  ros::Subscriber odom_sub_;
  ros::Subscriber obstacles_sub_;

  ros::Publisher controls_pub_;
  ros::Publisher potential_pub_;
  ros::Publisher grad_norm_pub_;
  ros::Publisher is_saddle_pub_;

  geometry_msgs::Pose2D pose_;
  std::vector<Ubstacle> obstacles_;

  std::vector<double> beta_list_;
  std::vector<arma::vec> grad_beta_list_;
  std::vector<arma::mat> hess_beta_list_;

  double t_;

  // Parameters
  bool p_active_;
  bool p_run_;
  bool p_normalize_gradient_;

  double p_loop_rate_;
  double p_sampling_time_;

  double p_a_;
  double p_b_dash_;
  double p_k_w_;
  double p_epsilon_;
  double p_kappa_;
  double p_inv_kappa_;

  double p_epsilon_e_;
  double p_epsilon_d_;

  double p_min_saddle_gradient_;
  double p_min_normalizing_potential_;
  double p_min_normalizing_gradient_;

  double p_max_u_;
  double p_max_v_;
  double p_max_w_;

  Ubstacle p_world_obstacle_;
};

} // namespace robor_controllers
