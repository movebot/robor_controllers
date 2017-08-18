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

#include "robor_controllers/urakubo_controller.h"

using namespace robor_controllers;
using namespace arma;
using namespace std;

UrakuboController::UrakuboController(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  timer_ = nh_.createTimer(ros::Duration(1.0), &UrakuboController::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &UrakuboController::updateParams, this);

  initialize();
}

UrakuboController::~UrakuboController() {
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("run");
  nh_local_.deleteParam("normalize_gradient");

  nh_local_.deleteParam("loop_rate");

  nh_local_.deleteParam("a");
  nh_local_.deleteParam("b_");
  nh_local_.deleteParam("k_w");
  nh_local_.deleteParam("epsilon");
  nh_local_.deleteParam("kappa");

  nh_local_.deleteParam("epsilon_e");
  nh_local_.deleteParam("epsilon_p");
  nh_local_.deleteParam("epsilon_d");

  nh_local_.deleteParam("min_saddle_gradient");
  nh_local_.deleteParam("min_normalizing_potential");
  nh_local_.deleteParam("min_normalizing_gradient");

  nh_local_.deleteParam("max_u");
  nh_local_.deleteParam("max_v");
  nh_local_.deleteParam("max_w");

  nh_local_.deleteParam("world_x");
  nh_local_.deleteParam("world_y");
  nh_local_.deleteParam("world_radius");

  nh_local_.deleteParam("robot_frame_id");
  nh_local_.deleteParam("reference_frame_id");
}

bool UrakuboController::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("run", p_run_, false);
  nh_local_.param<bool>("normalize_gradient", p_normalize_gradient_, true);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;

  nh_local_.param<double>("a", p_a_, 0.5);
  nh_local_.param<double>("b_", p_b_dash_, 2.5);
  nh_local_.param<double>("k_w", p_k_w_, 0.1);
  nh_local_.param<double>("epsilon", p_epsilon_, 0.0001);
  nh_local_.param<double>("kappa", p_kappa_, 3.0);

  nh_local_.param<double>("epsilon_e", p_epsilon_n_, 0.1);
  nh_local_.param<double>("epsilon_p", p_epsilon_n_, 0.01);
  nh_local_.param<double>("epsilon_d", p_epsilon_d_, 0.01);

  nh_local_.param<double>("min_saddle_gradient", p_min_saddle_gradient_, 0.05);
  nh_local_.param<double>("min_normalizing_potential", p_min_normalizing_potential_, 0.07);
  nh_local_.param<double>("min_normalizing_gradient", p_min_normalizing_gradient_, 0.05);

  nh_local_.param<double>("max_u", p_max_u_, 0.4);
  nh_local_.param<double>("max_v", p_max_v_, 0.4);
  nh_local_.param<double>("max_w", p_max_w_, 2.7);

  nh_local_.param<double>("world_x", p_world_obstacle_.x, 0.0);
  nh_local_.param<double>("world_y", p_world_obstacle_.y, 0.0);
  nh_local_.param<double>("world_radius", p_world_obstacle_.r, 5.0);

  nh_local_.param<string>("robot_frame_id", p_robot_frame_id_, string("robot"));
  nh_local_.param<string>("reference_frame_id", p_reference_frame_id_, string("reference"));

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_kappa_ != 0.0)
    p_inv_kappa_ = 1.0 / p_kappa_;
  else
    p_inv_kappa_ = 0.0;

  if (p_active_ != prev_active) {
    if (p_active_) {
      obstacles_sub_ = nh_.subscribe("obstacles", 5, &UrakuboController::obstaclesCallback, this);
      controls_pub_  = nh_.advertise<geometry_msgs::Twist>("controls", 5);
    }
    else {
      geometry_msgs::TwistPtr controls_msg(new geometry_msgs::Twist);
      controls_pub_.publish(controls_msg);

      obstacles_sub_.shutdown();
      controls_pub_.shutdown();
    }
  }

  if (p_active_ && !p_run_) {
    geometry_msgs::TwistPtr controls_msg(new geometry_msgs::Twist);
    controls_pub_.publish(controls_msg);
  }

  if (p_active_ && p_run_) {
    t_ = 0;
    timer_.start();
  }
  else
    timer_.stop();

  return true;
}

void UrakuboController::timerCallback(const ros::TimerEvent& e) {
  ros::Time now = ros::Time::now();

  tf::StampedTransform pose_tf;
  try {
    tf_ls_.waitForTransform(p_reference_frame_id_, p_robot_frame_id_, now, ros::Duration(0.05));
    tf_ls_.lookupTransform(p_reference_frame_id_, p_robot_frame_id_, now, pose_tf);
  }
  catch (tf::TransformException ex) { pose_tf.setIdentity(); std::cout << "ERROR" << std::endl; }

  pose_.x = pose_tf.getOrigin().x();
  pose_.y = pose_tf.getOrigin().y();
  pose_.theta = tf::getYaw(pose_tf.getRotation());



  double dt = 1.0 / p_loop_rate_; // (e.current_real - e.last_real).toSec();
  t_ += dt;

  computeControls();
}

void UrakuboController::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr obstacles_msg) {
  obstacles_.clear();

  Ubstacle o;
  for (const auto& obstacle : obstacles_msg->circles) {
    o.x = obstacle.center.x;
    o.y = obstacle.center.y;
    o.r = obstacle.radius;

    obstacles_.push_back(o);
  }
}

void UrakuboController::computeControls() {
  geometry_msgs::TwistPtr controls_msg(new geometry_msgs::Twist);

//  double b, h, g;               // Variable parameters

//  mat B = mat(3, 2).zeros();    // Input matrix
//  vec L = vec(3).zeros();       // [sin(fi) -cos(fi) 0]^T
  vec graadV = vec(3).zeros();  // Gradient of navigation function
  vec u = vec(2).zeros();       // Control signals

  vec pose = { pose_.x, pose_.y, pose_.theta };

//  // Auxiliary matrices
//  mat I = eye<mat>(2, 2);
//  mat J = mat(2, 2).zeros();
//  J <<  0.0 << 1.0 << endr
//    << -1.0 << 0.0 << endr;

//  // Update input matrix
//  B(0, 0) = cos(pose_.theta);
//  B(1, 0) = sin(pose_.theta);
//  B(2, 1) = 1.0;

//  L(0) =  sin(pose_.theta);
//  L(1) = -cos(pose_.theta);

  beta_list_.clear();
  grad_beta_list_.clear();
  hess_beta_list_.clear();

  beta_list_.push_back(betaWorld());
  for (auto& o : obstacles_)
    beta_list_.push_back(betaObstacle(o));

  grad_beta_list_.push_back(gradBetaWorld());
  for (auto& o : obstacles_)
    grad_beta_list_.push_back(gradBetaObstacle(o));

  hess_beta_list_.push_back(hessBetaWorld());
  for (auto& o : obstacles_)
    hess_beta_list_.push_back(hessBetaObstacle(o));

  // Recalculate gradient of navigation function
  graadV = gradV();

//  // Recalculate parameter b
//  g = norm(trans(B) * graadV);
//  h = pow(g, 2.0) + p_epsilon_ * sqrt(g);

//  if (h != 0.0)
//    b = -p_b_dash_ * dot(L, graadV) / h;
//  else
//    b = p_b_dash_;
//  // Calculate control signals
//  vec eigen_vals = vec(3).zeros();
//  vec min_eigen_vec = vec(3).zeros();

//  if (detectSaddle(min_eigen_vec, eigen_vals))
//    u = saddleAvoidance(min_eigen_vec, eigen_vals);
//  else {
//   if (p_normalize_gradient_ && norm(graadV) > p_min_normalizing_gradient_ && V() > p_min_normalizing_potential_)
//      graadV /= norm(graadV);
//  }
// u = -(p_a_ * I + b * J) * trans(B) * graadV / norm(graadV);
// u = -(p_a_ * I + b * J) * trans(B) * graadV * (norm(pose) + p_epsilon_e_) / (norm(trans(B) * graadV) + p_epsilon_d_);
// && norm(graadV) > p_min_normalizing_gradient_ && V() > p_min_normalizing_potential_

  double v_x, v_y, w;
  vec graadV2 = { graadV(0), graadV(1) };

//  if (p_normalize_gradient_)
//    u = -graadV2 * (norm(pose) + p_epsilon_n_) / (pow(norm(graadV2), 1.1 + p_epsilon_p_) + p_epsilon_d_);
//  else
    u = -graadV2;

  v_x = u(0);
  v_y = u(1);
  w = -1.0 * pose_.theta;

  // Scale the control signals
  double s = 1.0;

  if (fabs(v_x) / p_max_u_ > s)
    s = fabs(v_x) / p_max_u_;
  if (fabs(v_y) / p_max_v_ > s)
    s = fabs(v_y) / p_max_v_;
  if (fabs(w) / p_max_w_ > s)
    s = fabs(w) / p_max_w_;

  if (s > 1.0) {
    v_x /= s;
    v_y /= s;
    w /= s;
  }

//  if (sqrt(pose_.x * pose_.x + pose_.y * pose_.y + pose_.theta * pose_.theta / 9.0) < 0.01) {
//    u(0) = 0.0;
//    u(1) = 0.0;
//  }

  controls_msg->linear.x = v_x;
  controls_msg->linear.y = v_y;
  controls_msg->angular.z = w;

  controls_pub_.publish(controls_msg);
}

vec UrakuboController::saddleAvoidance(const vec min_eigen_vec, const vec eigen_vals) {
  vec u = vec(2).zeros();

  double e1 = min_eigen_vec(0);
  double e2 = min_eigen_vec(1);

  double w_ = 2.5;
  double bi = e1;

  double a = 0.7;

  u(0) = a * sin(w_ * t_) + bi;
  u(1) = a * cos(w_ * t_);

  return u;
}

bool UrakuboController::detectSaddle(vec& min_eigen_vec, vec& eig_vals) {
  if (norm(gradV()) < 0.0) {//p_min_saddle_gradient_) {
    mat hess = hessV();	// In hessian inf values may occur!

    cx_vec cx_eig_val;
    cx_mat cx_eig_vec;
    eig_gen(cx_eig_val, cx_eig_vec, hess);

    vec eig_val = real(cx_eig_val);
    mat eig_vec = real(cx_eig_vec);

    if (!(min(eig_val) < 0.0 && max(eig_val) > 0.0))
      return false;

    min_eigen_vec = eig_vec.col(index_min(eig_val));
    eig_vals = eig_val;

    return true;
  }

  return false;
}

double UrakuboController::squaredNormR() {
  return pow(pose_.x, 2.0) + pow(pose_.y, 2.0);
}

double UrakuboController::betaWorld() {
  return pow(p_world_obstacle_.r, 2.0) - pow(pose_.x - p_world_obstacle_.x, 2.0) - pow(pose_.y - p_world_obstacle_.y, 2.0);
}

double UrakuboController::betaObstacle(const Ubstacle& o) {
  return pow(pose_.x - o.x, 2.0) + pow(pose_.y - o.y, 2.0) - pow(o.r, 2.0);
}

double UrakuboController::beta() {
  double beta = betaWorld();

  for (auto& o : obstacles_) {
    beta *= betaObstacle(o);
  }

  return beta;
}

vec UrakuboController::gradBetaWorld() {
  vec gradB_0 = vec(3);

  gradB_0(0) = -2.0 * (pose_.x - p_world_obstacle_.x);
  gradB_0(1) = -2.0 * (pose_.y - p_world_obstacle_.y);
  gradB_0(2) =  0.0;

  return gradB_0;
}

vec UrakuboController::gradBetaObstacle(const Ubstacle& o) {
  vec gradBetaObstacle = vec(3);

  gradBetaObstacle(0) = 2.0 * (pose_.x - o.x);
  gradBetaObstacle(1) = 2.0 * (pose_.y - o.y);
  gradBetaObstacle(2) = 0.0;

  return gradBetaObstacle;
}

vec UrakuboController::gradBeta() {
  // gradB = gradB0 * B1 * B2 * ... * Bn + B0 * gradB1 * B2 * ... * Bn + ... + B0 * B1 * B2 * ... * gradBn
  vec gradB = vec(3).zeros();
  vec part_grad = vec(3).zeros();

  for (int i = 0; i < beta_list_.size(); ++i) {
    part_grad = grad_beta_list_[i];

    for (int j = 0; j < beta_list_.size(); ++j)
      if (i != j)
        part_grad *= beta_list_[j];

    gradB += part_grad;
  }

  return gradB;
}

mat UrakuboController::hessBetaWorld() {
  mat hessBetaWorld = mat(3,3).zeros();
  hessBetaWorld(0,0) = -2.0;
  hessBetaWorld(1,1) = -2.0;

  return hessBetaWorld;
}

mat UrakuboController::hessBetaObstacle(const Ubstacle& o) {
  mat hessBetaObstacle = mat(3,3).zeros();
  hessBetaObstacle(0,0) = 2.0;
  hessBetaObstacle(1,1) = 2.0;

  return hessBetaObstacle;
}

mat UrakuboController::hessBeta() {
  int N = beta_list_.size();

  mat hess_beta = mat(3,3).zeros();

  for (int i = 0; i < N; ++i) {
    mat part_hess = mat(3,3).zeros();
    part_hess = hess_beta_list_[i];

    for (int j = 0; j < N; ++j)
      if (i != j)
        part_hess *= beta_list_[j];

    vec part_grad = vec(3).zeros();
    for (int j = 0; j < N; ++j) {
      double part_beta = 1.0;

      if (i != j) {
        for (int k = 0; k < N; ++k) {
          if (k != i && k != j)
            part_beta *= beta_list_[k];
        }

        part_grad += part_beta * grad_beta_list_[j];
      }
    }

    hess_beta += part_hess + part_grad * trans(grad_beta_list_[i]);
  }

  return hess_beta;
}

double UrakuboController::C() {
  return squaredNormR() + pow(pose_.theta, 2.0) * (p_k_w_) / (p_k_w_ + squaredNormR());
}

vec UrakuboController::gradC() {
  vec gradC = vec(3).zeros();

  gradC(0) = 2.0 * pose_.x;// * (1.0 - p_k_w_ * pow(pose_.theta, 2.0) / pow(p_k_w_ + squaredNormR(), 2.0));
  gradC(1) = 2.0 * pose_.y;// * (1.0 - p_k_w_ * pow(pose_.theta, 2.0) / pow(p_k_w_ + squaredNormR(), 2.0));
  gradC(2) = 0.0;// 2.0 * pose_.theta * p_k_w_ / (p_k_w_ + squaredNormR());

  return gradC;
}

mat UrakuboController::hessC() {
  mat hessC = mat(3,3).zeros();

  hessC(0,0) = 8.0 * p_k_w_ * pow(pose_.theta, 2.0) * pow(pose_.x, 2.0) / pow(squaredNormR() + p_k_w_, 3.0) -
               2.0 * p_k_w_ * pow(pose_.theta, 2.0) / pow(squaredNormR() + p_k_w_, 2.0) + 2.0;

  hessC(1,1) = 8.0 * p_k_w_ * pow(pose_.theta, 2.0) * pow(pose_.y, 2.0) / pow(squaredNormR() + p_k_w_, 3.0) -
               2.0 * p_k_w_ * pow(pose_.theta, 2.0) / pow(squaredNormR() + p_k_w_, 2.0) + 2.0;

  hessC(2,2) = 2.0 * p_k_w_ / (squaredNormR() + p_k_w_);

  hessC(0,1) = (hessC(1,0) = 8.0 * p_k_w_ * pow(pose_.theta, 2.0) * pose_.x * pose_.y / pow(squaredNormR() + p_k_w_, 3.0));

  hessC(0,2) = (hessC(2,0) = -4.0 * p_k_w_ * pose_.theta * pose_.x / pow(squaredNormR() + p_k_w_, 2.0));

  hessC(1,2) = (hessC(2,1) = -4.0 * p_k_w_ * pose_.theta * pose_.y / pow(squaredNormR() + p_k_w_, 2.0));

  return hessC;
}

double UrakuboController::V() {
  double V = 0.0;

  double D = pow(pow(C(), p_kappa_) + beta(), p_inv_kappa_);

  if (D != 0.0)
    V = C() / D;

  return V;
}

vec UrakuboController::gradV() {
  vec gradV = vec(3).zeros();

  vec N = - (C() * gradBeta() - p_kappa_ * beta() * gradC());
  double D = p_kappa_ * pow(beta() + pow(C(), p_kappa_), p_inv_kappa_ + 1.0);

  if (D != 0.0)
    gradV = N / D;

  return gradV;
}

mat UrakuboController::hessV() {
  mat hessV = mat(3,3).zeros();

  hessV = hessC() / pow(beta() + pow(C(), p_kappa_), p_inv_kappa_) -
          C() * (hessBeta() + p_kappa_ * pow(C(), p_kappa_ - 1.0) * hessC() + p_kappa_ * pow(C(), p_kappa_ - 2.0) * gradC() * trans(gradC()) * (p_kappa_ - 1.0)) / (p_kappa_ * pow(beta() + pow(C(), p_kappa_), p_inv_kappa_ + 1.0)) -
          2.0 * (gradBeta() + p_kappa_ * pow(C(), p_kappa_ - 1.0) * gradC()) * trans(gradC()) / (p_kappa_ * pow(beta() + pow(C(), p_kappa_), p_inv_kappa_ + 1.0)) +
          C() * (p_inv_kappa_ + 1.0) * (gradBeta() + p_kappa_ * pow(C(), p_kappa_ - 1.0) * gradC()) * trans(gradBeta() + p_kappa_ * pow(C(), p_kappa_ - 1.0) * gradC()) / (p_kappa_ * pow(beta() + pow(C(), p_kappa_), p_inv_kappa_ + 2.0));

  return hessV;
}
