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

#include "robor_controllers/potentials_controller.h"

using namespace robor_controllers;
using namespace arma;
using namespace std;

PotentialsController::PotentialsController(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  pose_ = { 0.0, 0.0 };
  ref_pose_ = { 0.0, 0.0 };
  ref_velocity_ = { 0.0, 0.0 };

  theta_ = 0.0;
  ref_theta_ = 0.0;
  ref_omega_ = 0.0;

  timer_ = nh_.createTimer(ros::Duration(1.0), &PotentialsController::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &PotentialsController::updateParams, this);
  collector_srv_ = nh_local_.advertiseService("collector", &PotentialsController::collectData, this);

  initialize();
}

bool PotentialsController::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("run", p_run_, false);
  nh_local_.param<bool>("assisted_control", p_assisted_control_, false);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;

  nh_local_.param<double>("R", p_R_, 0.5);
  nh_local_.param<double>("eta", p_eta_, 1.2);
  nh_local_.param<double>("delta", p_delta_, 0.2);
  nh_local_.param<double>("ko_velocity", p_ko_velocity_, 0.15);

  nh_local_.param<double>("gain_pose", p_gain_pose_, 0.5);
  nh_local_.param<double>("gain_theta", p_gain_theta_, 4.0);

  nh_local_.param<double>("max_u", p_max_u_, 0.4);
  nh_local_.param<double>("max_v", p_max_v_, 0.4);
  nh_local_.param<double>("max_w", p_max_w_, 2.5);

  Obstacle::setParams(p_eta_, p_R_);

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_ != prev_active) {
    if (p_active_) {
      odom_sub_ = nh_.subscribe("robot_state", 5, &PotentialsController::odomCallback, this);
      ref_odom_sub_ = nh_.subscribe("reference_state", 5, &PotentialsController::refOdomCallback, this);
      obstacles_sub_ = nh_.subscribe("obstacles", 5, &PotentialsController::obstaclesCallback, this);
      controls_pub_ = nh_.advertise<geometry_msgs::Twist>("controls", 5);
    }
    else {
      geometry_msgs::TwistPtr controls_msg(new geometry_msgs::Twist);
      controls_pub_.publish(controls_msg);

      odom_sub_.shutdown();
      ref_odom_sub_.shutdown();
      obstacles_sub_.shutdown();
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

bool PotentialsController::collectData(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  odom_sub_.shutdown();
  ref_odom_sub_.shutdown();
  obstacles_sub_.shutdown();
  controls_pub_.shutdown();

  vec2 saved_pose = pose_;
  prepareObstacles();

  double ds = 0.05;

  double x_min = -5.0;
  double x_max = 5.0;

  double y_min = -5.0;
  double y_max = 5.0;

  int N_x = (x_max - x_min) / ds;
  int N_y = (y_max - y_min) / ds;

  vec x = vec(N_x + 1);
  for (int i = 0; i <= N_x; ++i)
    x(i) = x_min + i * ds;

  vec y = vec(N_y + 1);
  for (int j = 0; j <= N_y; ++j)
    y(j) = y_min + j * ds;

  mat potential = mat(N_y + 1, N_x + 1);
  imat in_obstacle = imat(N_y + 1, N_x + 1).zeros();
  imat negative_hess = imat(N_y + 1, N_x + 1).zeros();

  for (int i = 0; i <= N_x; ++i) {
    for (int j = 0; j <= N_y; ++j) {
      pose_ = { x(i), y(j) };

      potential(j, i) = attractingPotential();

      int idx = nearestObstacleIndex();
      if (idx < 0)
        continue;

      potential(j, i) += obstacles_[idx].repellingPotential(pose_);

      if (norm(pose_ - obstacles_[idx].center_) < obstacles_[idx].r_)
        in_obstacle(j, i) = 1;

      mat22 H = attractingHessian() + obstacles_[idx].repellingHessian(pose_);
      if (det(H) < -0.0001)
        negative_hess(j, i) = 1;
    }
  }

  string home_path = getenv("HOME");
  string folder_name = home_path + "/Robor/records/";

  boost::filesystem::create_directories(folder_name);
  potential.save(folder_name + "U.txt", csv_ascii);
  in_obstacle.save(folder_name + "in_obstacle.txt", csv_ascii);
  negative_hess.save(folder_name + "negative_hess.txt", csv_ascii);
  x.save(folder_name + "x.txt", csv_ascii);
  y.save(folder_name + "y.txt", csv_ascii);

  std::ofstream file(folder_name + "obstacles.txt");
  file << "n,x,y,vx,vy,r,R,U_s,U_d" << endl;

  int n = 1;
  for (const Obstacle& o : obstacles_) {
    file << n << "," << o.center_(0) << "," << o.center_(1) << "," << o.velocity_(0) << "," << o.velocity_(1) << "," << o.r_ << "," << o.R_ << "," << o.U_s_ << "," << o.U_d_ << endl;
    n++;
  }

  file.close();

  pose_ = saved_pose;

  odom_sub_ = nh_.subscribe("robot_state", 5, &PotentialsController::odomCallback, this);
  ref_odom_sub_ = nh_.subscribe("reference_state", 5, &PotentialsController::refOdomCallback, this);
  obstacles_sub_ = nh_.subscribe("obstacles", 5, &PotentialsController::obstaclesCallback, this);
  controls_pub_ = nh_.advertise<geometry_msgs::Twist>("controls", 5);
}

void PotentialsController::timerCallback(const ros::TimerEvent& e) {
  if (p_assisted_control_) {
    ref_pose_ = pose_;
    p_gain_pose_ = 0.0;
  }

  prepareObstacles();

  vec2 velocity = -attractingGradient();
  mat22 H = attractingHessian();

  if (obstacles_.size() > 0) {
    int nearest_obstacle_idx = nearestObstacleIndex();
    velocity -= obstacles_[nearest_obstacle_idx].repellingGradient(pose_);
    H += obstacles_[nearest_obstacle_idx].repellingHessian(pose_);

    if (det(H) < -0.0001) // Cannot be zero bec. of numerical zeros with negative sign
      velocity += saddleAvoidanceFactor(obstacles_[nearest_obstacle_idx]);
  }

  //  // Reduce orientation error to +/- pi to avoid unwinding effect
  //  double ref_theta_aux_ = atan2(velocity(1), velocity(0));

  //  double e_theta_aux;
  //  if (cos(theta_ - ref_theta_aux_) != 0.0)
  //    e_theta_aux = atan(sin(theta_ - ref_theta_aux_) / cos(theta_ - ref_theta_aux_));
  //  else
  //    e_theta_aux = M_PI / 2.0;

  double e_theta = atan2(sin(theta_ - ref_theta_), cos(theta_ - ref_theta_));
  double omega = -p_gain_theta_ * e_theta + ref_omega_;
  // double omega = -p_gain_theta_ * ((1.0 - exp(-norm(velocity) / 0.01)) * e_theta_aux + exp(-norm(velocity) / 0.01) * e_theta) + ref_omega_;

  // The control signals must be described in the base coordinate system
  double u, v;
  u =  velocity(0) * cos(theta_) + velocity(1) * sin(theta_);
  v = -velocity(0) * sin(theta_) + velocity(1) * cos(theta_);

  scaleControls(u, v, omega);

  geometry_msgs::TwistPtr controls_msg(new geometry_msgs::Twist);
  controls_msg->linear.x = u;
  controls_msg->linear.y = v;
  controls_msg->angular.z = omega;

  controls_pub_.publish(controls_msg);
}

void PotentialsController::odomCallback(const nav_msgs::Odometry::ConstPtr odom_msg) {
  pose_ = { odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y };
  velocity_ = { odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y };

  theta_ = tf::getYaw(odom_msg->pose.pose.orientation);
  omega_ = odom_msg->twist.twist.angular.z;
}

void PotentialsController::refOdomCallback(const nav_msgs::Odometry::ConstPtr ref_odom_msg) {
  ref_pose_ = { ref_odom_msg->pose.pose.position.x, ref_odom_msg->pose.pose.position.y };
  ref_velocity_ = { ref_odom_msg->twist.twist.linear.x, ref_odom_msg->twist.twist.linear.y };

  ref_theta_ = tf::getYaw(ref_odom_msg->pose.pose.orientation);
  ref_omega_ = ref_odom_msg->twist.twist.angular.z;
}

void PotentialsController::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr obstacles_msg) {
  obstacles_.clear();

  for (const auto& obstacle : obstacles_msg->circles) {
    vec2 center = { obstacle.center.x, obstacle.center.y };
    vec2 velocity = { obstacle.velocity.x, obstacle.velocity.y };

    obstacles_.push_back(Obstacle(center, velocity, obstacle.radius, 0.0, 0.0));
  }
}

int PotentialsController::nearestObstacleIndex() {
  double distance = 100000.0;
  int idx = -1;

  for (int i = 0; i < obstacles_.size(); ++i) {
    double d = norm(obstacles_[i].center_ - pose_) - obstacles_[i].r_;

    if (d < distance) {
      distance = d;
      idx = i;
    }
  }

  return idx;
}

void PotentialsController::prepareObstacles() {
  for (Obstacle& obstacle : obstacles_) {
    obstacle.U_s_ = p_eta_ * obstacle.R_ * (p_gain_pose_ * (norm(obstacle.center_ - ref_pose_) + obstacle.r_) + abs(dot(ref_velocity_, normalise(obstacle.center_ - ref_pose_))));
    obstacle.U_d_ = 0.5 * p_eta_ * obstacle.R_ * (norm(obstacle.velocity_) + dot(obstacle.rhoGradient(pose_), obstacle.velocity_));

    setReferenceWarp(obstacle);
  }

  for (int i = 0; i < obstacles_.size(); ++i)
    for (int j = i + 1; j < obstacles_.size(); ++j)
      setWarpVectors(obstacles_[i], obstacles_[j]);
}

void PotentialsController::setReferenceWarp(Obstacle& o) {
  double d = norm(ref_pose_ - o.center_);
  vec2 n = (ref_pose_ - o.center_);

  if (d == 0.0)
    return;
  else if (d > o.r_ + p_delta_)
    o.warp_vectors_.push_back(n);
  else //if (d > o.r_ && d <= o.r_ + p_delta_)
    o.warp_vectors_.push_back((o.r_ + p_delta_) * n / d);
//  else
//    o.warp_vectors_.push_back((o.r_ + p_delta_) * n * o.r_ / squared(d));
}

void PotentialsController::setWarpVectors(Obstacle& o1, Obstacle& o2) {
  double d = norm(o2.center_ - o1.center_);
  vec2 n = (o2.center_ - o1.center_);

  if (d > o1.r_ + o2.r_) {
    double d1 = 0.5 * (d + o1.r_ - o2.r_);
    double d2 = 0.5 * (d + o2.r_ - o1.r_);

    vec2 w1 =  n * d1 / d;
    vec2 w2 = -n * d2 / d;

    bool use_w1 = true;
    bool use_w2 = true;

    for (int i = 0; i < o1.warp_vectors_.size(); ++i) {
      vec2 w = o1.warp_vectors_[i];

      // Remove vector from list if current vector covers it
      if (dot(w1, w) >= squared(d1)) {
        o1.warp_vectors_.erase(o1.warp_vectors_.begin() + i);
        continue;
      }

      // Discard current vector if the on in the list covers it
      if (dot(w1, w) > squared(norm(w))) {
        use_w1 = false;
        break;
      }
    }

    for (int i = 0; i < o2.warp_vectors_.size(); ++i) {
      vec2 w = o2.warp_vectors_[i];

      // Remove vector from list if current vector covers it
      if (dot(w2, w) >= squared(d2)) {
        o2.warp_vectors_.erase(o2.warp_vectors_.begin() + i);
        continue;
      }

      // Discard current vector if the on in the list covers it
      if (dot(w2, w) > squared(norm(w))) {
        use_w2 = false;
        break;
      }
    }

    if (use_w1)
      o1.warp_vectors_.push_back(w1);

    if (use_w2)
      o2.warp_vectors_.push_back(w2);
  }
}

void PotentialsController::scaleControls(double& u, double& v, double& w) {
//  double s = 1.0;

//  if (fabs(u) / p_max_u_ > s)
//    s = fabs(u) / p_max_u_;
//  if (fabs(v) / p_max_v_ > s)
//    s = fabs(v) / p_max_v_;
//  if (fabs(w) / p_max_w_ > s)
//    s = fabs(w) / p_max_w_;

//  u /= s;  v /= s;  w /= s;
  double s_u = 1.0;
  double s_v = 1.0;
  double s_w = 1.0;

  if (fabs(u) / p_max_u_ > s_u)
    s_u = fabs(u) / p_max_u_;

  if (fabs(v) / p_max_v_ > s_v)
    s_v = fabs(v) / p_max_v_;

  if (fabs(w) / p_max_w_ > s_w)
    s_w = fabs(w) / p_max_w_;

  u /= s_u;  v /= s_v;  w /= s_w;
}

double PotentialsController::attractingPotential() {
  return 0.5 * p_gain_pose_ * squared(norm(pose_ - ref_pose_)) - dot(pose_, ref_velocity_);
}

vec2 PotentialsController::attractingGradient() {
  return p_gain_pose_ * (pose_ - ref_pose_) - ref_velocity_;
}

mat22 PotentialsController::attractingHessian() {
  return p_gain_pose_ * mat(2,2).eye();
}

vec2 PotentialsController::saddleAvoidanceFactor(const Obstacle& o) {
  vec2 attr_grad = attractingGradient();
  vec2 rep_grad = o.repellingGradient(pose_);

  // No repelling gradient = no problem
  if (norm(rep_grad) == 0.0)
    return vec(2).zeros();

  double cross, cos_grad;
  vec2 sv = o.velocity_ + attr_grad;
  if (norm(sv) == 0.0) {
    cross = 1.0;
    cos_grad = 1.0;
  }
  else {
    cross = -sv(0) * rep_grad(1) + sv(1) * rep_grad(0);
    cos_grad = dot(rep_grad, sv) / (norm(rep_grad) * norm(sv));
  }

  vec2 anti_saddle = { rep_grad(1), -rep_grad(0) };
  anti_saddle *= p_ko_velocity_ * 0.5 * (1.0 - cos_grad) * signum(cross) / norm(rep_grad);

  return anti_saddle;
}

//
// Obstacle Methods
//
double Obstacle::rho(const vec2& p) const {
  return norm(p - center_);
}

vec2 Obstacle::rhoGradient(const vec2& p) const {
  return (p - center_) / rho(p);
}

mat22 Obstacle::rhoHessian(const vec2& p) const {
  vec2 ro_grad = rhoGradient(p);
  vec2 perp_grad = { ro_grad(1), -ro_grad(0) };

  return perp_grad * trans(perp_grad) / rho(p);
}


double Obstacle::rawRepellingPotential(const double ro) const {
  return (U_s_ + U_d_) * exp(-(ro - r_) / R_);
}

double Obstacle::rawRepellingPotential(const vec2& p) const {
  double ro = rho(p);
  return (ro > r_) ? rawRepellingPotential(ro) : (U_s_ + U_d_);
}

vec2 Obstacle::rawRepellingGradient(const vec2& p) const {
  double ro = rho(p);
  vec2 grad_U_d = 0.5 * eta_ * R_ * rhoHessian(p) * velocity_;
  return (ro > r_) ? (grad_U_d - (U_s_ + U_d_) * rhoGradient(p) / R_) * exp(-(ro - r_) / R_) : vec(2).zeros();
}

mat22 Obstacle::rawRepellingHessian(const vec2& p) const {
  double ro = rho(p);
  vec2 ro_grad = rhoGradient(p);
  mat22 ro_hess = rhoHessian(p);

  double x_ = p(0) - center_(0);
  double y_ = p(1) - center_(1);

  mat22 aux1 = { {-y_ * velocity_(1), 2.0 * x_ * velocity_(1) - y_ * velocity_(0)}, {2.0 * y_ * velocity_(0) - x_ * velocity_(1), -x_ * velocity_(0)} };
  mat22 aux2 = -3.0 * ro_grad * trans(ro_hess * velocity_) / quaded(ro) + aux1;

  mat22 A = (0.5 * eta_ * (R_ * aux2 - (ro_hess * velocity_) * trans(ro_grad)) - (U_s_ + U_d_) * ro_hess / R_) * exp(-(ro - r_) / R_);
  mat22 B = -rawRepellingGradient(p) * trans(ro_grad) / R_;

  return (ro > r_) ? A + B : mat(2,2).zeros();
}


double Obstacle::warpDistance(const vec2& warp_vec, const vec2& p, double& range) const {
  range = norm(warp_vec) - r_;
  return dot(p - center_, warp_vec) / (range + r_) - r_;
}

vec2 Obstacle::warpDistanceGradient(const vec2& warp_vec) const {
  return normalise(warp_vec);
}

mat22 Obstacle::warpDistanceHessian() const {
  return mat(2,2).zeros();
}


double Obstacle::warp(const double d, const double D) const {
  return 2.0 * cubed(d / D) - 3.0 * squared(d / D) + 1.0;
}

double Obstacle::warp(const vec2& warp_vec, const vec2& p) const {
  double warp_range;
  double warp_distance = warpDistance(warp_vec, p, warp_range);

  if (warp_distance >= warp_range)
    return 0.0;
  else if (warp_distance <= 0.0)
    return 1.0;
  else
    return warp(warp_distance, warp_range);
}

vec2 Obstacle::warpGradient(const vec2& warp_vec, const vec2& p) const {
  double warp_range;
  double warp_distance = warpDistance(warp_vec, p, warp_range);

  if (warp_distance > 0.0 && warp_distance < warp_range)
    return 6.0 * warp_distance * (warp_distance - warp_range) * warpDistanceGradient(warp_vec) / cubed(warp_range);
  else
    return vec(2).zeros();
}

mat22 Obstacle::warpHessian(const vec2& warp_vec, const vec2& p) const {
  double warp_range;
  double warp_distance = warpDistance(warp_vec, p, warp_range);
  vec2 warp_grad = warpDistanceGradient(warp_vec);

  if (warp_distance > 0.0 && warp_distance < warp_range)
    return 6.0 * (2.0 * warp_distance - warp_range) * warp_grad * trans(warp_grad) / cubed(warp_range); // + 6.0 * warp_distance * (warp_distance - warp_range) * warpRhoHessian() / cubed(warp_range); //<-zero!
  else
    return mat(2,2).zeros();
}

double Obstacle::totalWarp(const vec2& p) const {
  double s = 1.0;

  for (const vec2& warp_vec : warp_vectors_)
    s *= warp(warp_vec, p);

  return s;
}

double Obstacle::repellingPotential(const vec2& p) const {
  return rawRepellingPotential(p) * totalWarp(p);
}

vec2 Obstacle::repellingGradient(const vec2& p) const {
  vec2 grad_P = rawRepellingGradient(p) * totalWarp(p);
  vec2 grad_warp = {0, 0};

  for (int j = 0; j < warp_vectors_.size(); ++j) {
    vec2 dP = warpGradient(warp_vectors_[j], p);

    for (int k = 0; k < warp_vectors_.size(); ++k)
      if (k != j) dP *= warp(warp_vectors_[k], p);

    grad_warp += dP;
  }

  grad_P += rawRepellingPotential(p) * grad_warp;

  return grad_P;
}

mat22 Obstacle::repellingHessian(const vec2& p) const {
  mat22 H1 = rawRepellingHessian(p) * totalWarp(p);

  vec2 warp_grad = vec(2).zeros();
  for (int k = 0; k < warp_vectors_.size(); ++k) {
    vec2 grad_s = warpGradient(warp_vectors_[k], p);

    for (int j = 0; j < warp_vectors_.size(); ++j)
      if (j != k) grad_s *= warp(warp_vectors_[j], p);

    warp_grad += grad_s;
  }
  mat22 H2 = 2.0 * rawRepellingGradient(p) * trans(warp_grad);

  mat22 H3 = mat(2,2).zeros();
  for (int k = 0; k < warp_vectors_.size(); ++k) {
    mat22 warp_hess = warpHessian(warp_vectors_[k], p);
    vec2 warp_grad_2 = vec(2).zeros();

    for (int j = 0; j < warp_vectors_.size(); ++j) {
      if (j != k) {
        warp_hess *= warp(warp_vectors_[j], p);

        vec2 grad_s_j = warpGradient(warp_vectors_[j], p);
        for (int l = 0; l < warp_vectors_.size(); ++l)
          if (l != j && l != k)
            grad_s_j *= warp(warp_vectors_[l], p);

        warp_grad_2 += grad_s_j;
      }
    }

    H3 += warp_hess + warpGradient(warp_vectors_[k], p) * trans(warp_grad_2);
  }

  mat22 H = H1 + H2 + rawRepellingPotential(p) * H3;

  return H;
}

double Obstacle::eta_ = 1.0;
double Obstacle::R_ = 1.0;
