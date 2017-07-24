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
