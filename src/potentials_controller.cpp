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

PotentialsController::~PotentialsController() {
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("run");
  nh_local_.deleteParam("assisted_control");

  nh_local_.deleteParam("loop_rate");

  nh_local_.deleteParam("R");
  nh_local_.deleteParam("eta");
  nh_local_.deleteParam("delta");
  nh_local_.deleteParam("ko_velocity");

  nh_local_.deleteParam("gain_pose");
  nh_local_.deleteParam("gain_theta");

  nh_local_.deleteParam("max_u");
  nh_local_.deleteParam("max_v");
  nh_local_.deleteParam("max_w");

  nh_local_.deleteParam("fixed_frame_id");
  nh_local_.deleteParam("robot_frame_id");
  nh_local_.deleteParam("reference_frame_id");
}

bool PotentialsController::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("run", p_run_, false);
  nh_local_.param<bool>("assisted_control", p_assisted_control_, false);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;

  nh_local_.param<double>("R", p_R_, 0.5);
  nh_local_.param<double>("eta", p_eta_, 1.2);
  nh_local_.param<double>("delta", p_delta_, 0.5);
  nh_local_.param<double>("ko_velocity", p_ko_velocity_, 0.15);

  nh_local_.param<double>("gain_pose", p_gain_pose_, 0.5);
  nh_local_.param<double>("gain_theta", p_gain_theta_, 0.5);

  nh_local_.param<double>("max_u", p_max_u_, 0.5);
  nh_local_.param<double>("max_v", p_max_v_, 0.5);
  nh_local_.param<double>("max_w", p_max_w_, 3.0);

  nh_local_.param<string>("fixed_frame_id", p_fixed_frame_id_, string("map"));
  nh_local_.param<string>("robot_frame_id", p_robot_frame_id_, string("robot"));
  nh_local_.param<string>("reference_frame_id", p_reference_frame_id_, string("reference"));

  Obstacle::setParams(p_eta_, p_R_);

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_ != prev_active) {
    if (p_active_) {
      reference_twist_sub_ = nh_.subscribe("reference_twist", 10, &PotentialsController::referenceTwistCallback, this);
      obstacles_sub_ = nh_.subscribe("obstacles", 10, &PotentialsController::obstaclesCallback, this);
      controls_pub_ = nh_.advertise<geometry_msgs::Twist>("controls", 10);
    }
    else {
      sendZeroControls();

      reference_twist_sub_.shutdown();
      obstacles_sub_.shutdown();
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

bool PotentialsController::collectData(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  reference_twist_sub_.shutdown();
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
  string folder_name = home_path + "/potential_records/";

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

  reference_twist_sub_ = nh_.subscribe("reference_twist", 5, &PotentialsController::referenceTwistCallback, this);
  obstacles_sub_ = nh_.subscribe("obstacles", 5, &PotentialsController::obstaclesCallback, this);
  controls_pub_ = nh_.advertise<geometry_msgs::Twist>("controls", 5);
}

void PotentialsController::timerCallback(const ros::TimerEvent& e) {
  ros::Time now = ros::Time::now();

  tf::StampedTransform robot_tf;
  try {
    tf_ls_.waitForTransform(p_fixed_frame_id_, p_robot_frame_id_, now, ros::Duration(0.1));
    tf_ls_.lookupTransform(p_fixed_frame_id_, p_robot_frame_id_, now, robot_tf);

    pose_ = { robot_tf.getOrigin().x(), robot_tf.getOrigin().y() };
    theta_ = tf::getYaw(robot_tf.getRotation());
  }
  catch (tf::TransformException ex) { sendZeroControls(); return; }

  if (!p_assisted_control_) {
    tf::StampedTransform reference_tf;
    try {
      tf_ls_.waitForTransform(p_fixed_frame_id_, p_reference_frame_id_, now, ros::Duration(0.1));
      tf_ls_.lookupTransform(p_fixed_frame_id_, p_reference_frame_id_, now, reference_tf);

      ref_pose_ = { reference_tf.getOrigin().x(), reference_tf.getOrigin().y() };
      ref_theta_ = tf::getYaw(reference_tf.getRotation());
    }
    catch (tf::TransformException ex) { sendZeroControls(); return; }
  }

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

//  double e_theta = atan2(sin(theta_ - ref_theta_), cos(theta_ - ref_theta_));
//  double omega = -p_gain_theta_ * e_theta + ref_omega_;

  double omega = 0.0;

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

void PotentialsController::referenceTwistCallback(const geometry_msgs::Twist::ConstPtr reference_twist_msg) {
  ref_velocity_ = { reference_twist_msg->linear.x, reference_twist_msg->linear.y };
  ref_omega_ = reference_twist_msg->angular.z;
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
  double s = 1.0;

  if (fabs(u) / p_max_u_ > s)
    s = fabs(u) / p_max_u_;
  if (fabs(v) / p_max_v_ > s)
    s = fabs(v) / p_max_v_;
  if (fabs(w) / p_max_w_ > s)
    s = fabs(w) / p_max_w_;

  u /= s;  v /= s;  w /= s;
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
