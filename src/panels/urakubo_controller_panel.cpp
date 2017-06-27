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

#include "robor_controllers/panels/urakubo_controller_panel.h"

using namespace robor_controllers;

UrakuboControllerPanel::UrakuboControllerPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("urakubo_controller") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  getParams();

  activate_checkbox_  = new QCheckBox("On/Off");

  a_input_            = new QLineEdit(QString::number(a_));
  b_dash_input_       = new QLineEdit(QString::number(b_dash_));
  k_w_input_          = new QLineEdit(QString::number(k_w_));
  epsilon_input_      = new QLineEdit(QString::number(epsilon_));
  kappa_input_        = new QLineEdit(QString::number(kappa_));
  world_radius_input_ = new QLineEdit(QString::number(world_radius_));

  QString home_path = getenv("HOME");
  QString play_icon = home_path + QString("/.local/share/icons/robor/play.png");
  QString stop_icon = home_path + QString("/.local/share/icons/robor/stop.png");

  start_button_ = new QPushButton;
  start_button_->setMinimumSize(50, 50);
  start_button_->setMaximumSize(50, 50);
  start_button_->setIcon(QIcon(play_icon));
  start_button_->setIconSize(QSize(25, 25));
  start_button_->setCheckable(true);

  stop_button_ = new QPushButton;
  stop_button_->setMinimumSize(50, 50);
  stop_button_->setMaximumSize(50, 50);
  stop_button_->setIcon(QIcon(stop_icon));
  stop_button_->setIconSize(QSize(25, 25));
  stop_button_->setCheckable(true);

  QFrame* lines[2];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QHBoxLayout* buttons_layout = new QHBoxLayout;
  buttons_layout->addItem(margin);
  buttons_layout->addWidget(stop_button_);
  buttons_layout->addWidget(start_button_);
  buttons_layout->addItem(margin);

  QGridLayout* input_layout = new QGridLayout;
  input_layout->addItem(margin, 0, 0, 3, 1);

  input_layout->addWidget(new QLabel("a:"), 0, 1);
  input_layout->addWidget(a_input_, 0, 2);
  input_layout->addWidget(new QLabel("-, "), 0, 3);

  input_layout->addWidget(new QLabel("b_:"), 0, 4);
  input_layout->addWidget(b_dash_input_, 0, 5);
  input_layout->addWidget(new QLabel("-"), 0, 6);

  input_layout->addWidget(new QLabel("k<sub>w</sub>:"), 0, 7);
  input_layout->addWidget(k_w_input_, 0, 8);
  input_layout->addWidget(new QLabel("-"), 0, 9);

  input_layout->addItem(margin, 0, 10, 2, 1);

  input_layout->addWidget(new QLabel("e:"), 1, 1);
  input_layout->addWidget(epsilon_input_, 1, 2);
  input_layout->addWidget(new QLabel("1/s, "), 1, 3);

  input_layout->addWidget(new QLabel("k:"), 1, 4);
  input_layout->addWidget(kappa_input_, 1, 5);
  input_layout->addWidget(new QLabel("-"), 1, 6);

  input_layout->addWidget(new QLabel("R:"), 1, 7);
  input_layout->addWidget(world_radius_input_, 1, 8);
  input_layout->addWidget(new QLabel("m"), 1, 9);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(lines[0]);
  layout->addLayout(buttons_layout);
  layout->addWidget(lines[1]);
  layout->addLayout(input_layout);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(start_button_, SIGNAL(clicked()), this, SLOT(start()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop()));

  evaluateParams();
}

void UrakuboControllerPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void UrakuboControllerPanel::start() {
  p_run_ = true;
  processInputs();
}

void UrakuboControllerPanel::stop() {
  p_run_ = false;
  processInputs();
}

void UrakuboControllerPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();
  p_run_ = p_run_ && p_active_;

  try { a_ = boost::lexical_cast<double>(a_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { a_ = 0.0; a_input_->setText("0.0"); }

  try { b_dash_ = boost::lexical_cast<double>(b_dash_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { b_dash_ = 0.0; b_dash_input_->setText("0.0"); }

  try { k_w_ = boost::lexical_cast<double>(k_w_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { k_w_ = 0.0; k_w_input_->setText("0.0"); }

  try { epsilon_ = boost::lexical_cast<double>(epsilon_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { epsilon_ = 0.0; epsilon_input_->setText("0.0"); }

  try { kappa_ = boost::lexical_cast<double>(kappa_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { kappa_ = 0.0; kappa_input_->setText("0.0"); }

  try { world_radius_ = boost::lexical_cast<double>(world_radius_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { world_radius_ = 0.0; world_radius_input_->setText("0.0"); }
}

void UrakuboControllerPanel::setParams() {
  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("run", p_run_);

  nh_local_.setParam("world_radius", world_radius_);
  nh_local_.setParam("kappa", kappa_);
  nh_local_.setParam("epsilon", epsilon_);
  nh_local_.setParam("k_w", k_w_);
  nh_local_.setParam("b_", b_dash_);
  nh_local_.setParam("a", a_);
}

void UrakuboControllerPanel::getParams() {
  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("run", p_run_, false);

  nh_local_.param<double>("world_radius", world_radius_, 3.0);
  nh_local_.param<double>("kappa", kappa_, 3.0);
  nh_local_.param<double>("epsilon", epsilon_, 0.0001);
  nh_local_.param<double>("k_w", k_w_, 0.1);
  nh_local_.param<double>("b_", b_dash_, 2.5);
  nh_local_.param<double>("a", a_, 0.5);
}

void UrakuboControllerPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);

  if (!p_active_)
    p_run_ = false;

  start_button_->setEnabled(p_active_ && !p_run_);
  start_button_->setChecked(p_run_);

  stop_button_->setEnabled(p_active_ && p_run_);
  stop_button_->setChecked(!p_run_);

  a_input_->setEnabled(p_active_ && !p_run_);
  b_dash_input_->setEnabled(p_active_ && !p_run_);
  k_w_input_->setEnabled(p_active_ && !p_run_);
  epsilon_input_->setEnabled(p_active_ && !p_run_);
  kappa_input_->setEnabled(p_active_ && !p_run_);
  world_radius_input_->setEnabled(p_active_ && !p_run_);
}

void UrakuboControllerPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void UrakuboControllerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void UrakuboControllerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robor_controllers::UrakuboControllerPanel, rviz::Panel)
