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

namespace robor_controllers
{

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

} // namespace robor_controllers
