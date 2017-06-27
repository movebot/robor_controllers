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

#include "robor_controllers/panels/potentials_controller_panel.h"

using namespace robor_controllers;

PotentialsControllerPanel::PotentialsControllerPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("potentials_controller") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  collect_cli_ = nh_local_.serviceClient<std_srvs::Empty>("collector");
  getParams();

  activate_checkbox_ = new QCheckBox("On/Off");
  assisted_checkbox_ = new QCheckBox("Manual assistance");

  R_input_           = new QLineEdit(QString::number(p_R_));
  eta_input_         = new QLineEdit(QString::number(p_eta_));
  delta_input_       = new QLineEdit(QString::number(p_delta_));

  gain_pose_input_   = new QLineEdit(QString::number(p_gain_pose_));
  gain_theta_input_  = new QLineEdit(QString::number(p_gain_theta_));
  gain_saddle_input_ = new QLineEdit(QString::number(p_gain_saddle_));

  max_u_input_       = new QLineEdit(QString::number(p_max_u_));
  max_v_input_       = new QLineEdit(QString::number(p_max_v_));
  max_w_input_       = new QLineEdit(QString::number(p_max_w_));

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

  collect_button_ = new QPushButton("PF snapshot");
  collect_button_->setMinimumSize(100, 30);
  collect_button_->setMaximumSize(100, 30);
  collect_button_->setToolTip("Saves 5x5 m grid around origin with resolution of 1 cm");

  QFrame* lines[4];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QString eta_char = QChar(0x03B7);
  QString delta_char = QChar(0x03B4);
  QString theta_char = QChar(0x03B8);
  QString omega_char = QChar(0x03C9);
  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  //
  QHBoxLayout* buttons_layout = new QHBoxLayout;
  buttons_layout->addItem(margin);
  buttons_layout->addWidget(stop_button_);
  buttons_layout->addWidget(start_button_);
  buttons_layout->addItem(margin);

  //
  QHBoxLayout* options_layout = new QHBoxLayout;
  options_layout->addItem(margin);
  options_layout->addWidget(assisted_checkbox_);
  options_layout->addWidget(collect_button_);
  options_layout->addItem(margin);

  //
  QGridLayout* input_layout = new QGridLayout;
  input_layout->addItem(margin, 0, 0, 3, 1);

  input_layout->addWidget(new QLabel("R:"), 0, 1, Qt::AlignRight);
  input_layout->addWidget(R_input_, 0, 2);
  input_layout->addWidget(new QLabel("m, "), 0, 3, Qt::AlignLeft);

  input_layout->addWidget(new QLabel(eta_char + ":"), 0, 4, Qt::AlignRight);
  input_layout->addWidget(eta_input_, 0, 5);
  input_layout->addWidget(new QLabel(" "), 0, 6, Qt::AlignLeft);

  input_layout->addWidget(new QLabel(delta_char + ":"), 0, 7, Qt::AlignRight);
  input_layout->addWidget(delta_input_, 0, 8);
  input_layout->addWidget(new QLabel("m"), 0, 9, Qt::AlignLeft);


  input_layout->addWidget(new QLabel("k<sub>P</sub>:"), 1, 1, Qt::AlignRight);
  input_layout->addWidget(gain_pose_input_, 1, 2);
  input_layout->addWidget(new QLabel("1/s, "), 1, 3, Qt::AlignLeft);

  input_layout->addWidget(new QLabel("k<sub>"+ theta_char +"</sub>:"), 1, 4, Qt::AlignRight);
  input_layout->addWidget(gain_theta_input_, 1, 5);
  input_layout->addWidget(new QLabel("1/s"), 1, 6, Qt::AlignLeft);

  input_layout->addWidget(new QLabel("k<sub>S</sub>:"), 1, 7, Qt::AlignRight);
  input_layout->addWidget(gain_saddle_input_, 1, 8);
  input_layout->addWidget(new QLabel("1/s"), 1, 9, Qt::AlignLeft);


  input_layout->addWidget(new QLabel("u<sub>max</sub>:"), 2, 1, Qt::AlignRight);
  input_layout->addWidget(max_u_input_, 2, 2);
  input_layout->addWidget(new QLabel("m/s, "), 2, 3, Qt::AlignLeft);

  input_layout->addWidget(new QLabel("v<sub>max</sub>:"), 2, 4, Qt::AlignRight);
  input_layout->addWidget(max_v_input_, 2, 5);
  input_layout->addWidget(new QLabel("m/s, "), 2, 6, Qt::AlignLeft);

  input_layout->addWidget(new QLabel(omega_char + "<sub>max</sub>:"), 2, 7, Qt::AlignRight);
  input_layout->addWidget(max_w_input_, 2, 8);
  input_layout->addWidget(new QLabel("rad/s"), 2, 9, Qt::AlignLeft);

  //
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget(activate_checkbox_);
  main_layout->addWidget(lines[0]);
  main_layout->addLayout(buttons_layout);
  main_layout->addWidget(lines[1]);
  main_layout->addLayout(options_layout);
  main_layout->addWidget(lines[2]);
  main_layout->addLayout(input_layout);
  main_layout->setAlignment(main_layout, Qt::AlignCenter);
  setLayout(main_layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(assisted_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(start_button_, SIGNAL(clicked()), this, SLOT(start()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop()));
  connect(collect_button_, SIGNAL(clicked()), this, SLOT(collectPF()));

  evaluateParams();
}

void PotentialsControllerPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void PotentialsControllerPanel::start() {
  p_run_ = true;
  processInputs();
}

void PotentialsControllerPanel::stop() {
  p_run_ = false;
  processInputs();
}

void PotentialsControllerPanel::collectPF() {
  std_srvs::Empty empty;
  collect_cli_.call(empty);
}

void PotentialsControllerPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();
  p_assisted_control_ = assisted_checkbox_->isChecked();

  p_run_ = p_run_ && p_active_;

  try { p_R_ = boost::lexical_cast<double>(R_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_R_ = 0.0; R_input_->setText("0.0"); }

  try { p_delta_ = boost::lexical_cast<double>(delta_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_delta_ = 0.0; delta_input_->setText("0.0"); }

  try { p_eta_ = boost::lexical_cast<double>(eta_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_eta_ = 0.0; eta_input_->setText("0.0"); }

  try { p_gain_pose_ = boost::lexical_cast<double>(gain_pose_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_gain_pose_ = 0.0; gain_pose_input_->setText("0.0"); }

  try { p_gain_theta_ = boost::lexical_cast<double>(gain_theta_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_gain_theta_ = 0.0; gain_theta_input_->setText("0.0"); }

  try { p_gain_saddle_ = boost::lexical_cast<double>(gain_saddle_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_gain_saddle_ = 0.0; gain_saddle_input_->setText("0.0"); }

  try { p_max_u_ = boost::lexical_cast<double>(max_u_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_u_ = 0.0; max_u_input_->setText("0.0"); }

  try { p_max_v_ = boost::lexical_cast<double>(max_v_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_v_ = 0.0; max_v_input_->setText("0.0"); }

  try { p_max_w_ = boost::lexical_cast<double>(max_w_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_w_ = 0.0; max_w_input_->setText("0.0"); }
}

void PotentialsControllerPanel::setParams() {
  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("run", p_run_);
  nh_local_.setParam("assisted_control", p_assisted_control_);

  nh_local_.setParam("R", p_R_);
  nh_local_.setParam("eta", p_eta_);
  nh_local_.setParam("delta", p_delta_);

  nh_local_.setParam("gain_pose", p_gain_pose_);
  nh_local_.setParam("gain_theta", p_gain_theta_);
  nh_local_.setParam("gain_saddle", p_gain_saddle_);

  nh_local_.setParam("max_u", p_max_u_);
  nh_local_.setParam("max_v", p_max_v_);
  nh_local_.setParam("max_w", p_max_w_);
}

void PotentialsControllerPanel::getParams() {
  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("run", p_run_, false);
  nh_local_.param<bool>("assisted_control", p_assisted_control_, false);

  nh_local_.param<double>("R", p_R_, 0.3);
  nh_local_.param<double>("eta", p_eta_, 1.5);
  nh_local_.param<double>("delta", p_delta_, 0.2);

  nh_local_.param<double>("gain_pose", p_gain_pose_, 0.5);
  nh_local_.param<double>("gain_theta", p_gain_theta_, 0.5);
  nh_local_.param<double>("gain_saddle", p_gain_saddle_, 0.2);

  nh_local_.param<double>("max_u", p_max_u_, 0.4);
  nh_local_.param<double>("max_v", p_max_v_, 0.4);
  nh_local_.param<double>("max_w", p_max_w_, 2.5);
}

void PotentialsControllerPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);

  if (!p_active_)
    p_run_ = false;

  assisted_checkbox_->setEnabled(p_active_ && !p_run_);
  assisted_checkbox_->setChecked(p_assisted_control_);

  start_button_->setEnabled(p_active_ && !p_run_);
  start_button_->setChecked(p_run_);

  stop_button_->setEnabled(p_active_ && p_run_);
  stop_button_->setChecked(!p_run_);

  collect_button_->setEnabled(p_active_);

  R_input_->setEnabled(p_active_ && !p_run_);
  eta_input_->setEnabled(p_active_ && !p_run_);
  delta_input_->setEnabled(p_active_ && !p_run_);

  gain_pose_input_->setEnabled(p_active_ && !p_run_ && !p_assisted_control_);
  gain_theta_input_->setEnabled(p_active_ && !p_run_ && !p_assisted_control_);
  gain_saddle_input_->setEnabled(p_active_ && !p_run_);

  max_u_input_->setEnabled(p_active_ && !p_run_);
  max_v_input_->setEnabled(p_active_ && !p_run_);
  max_w_input_->setEnabled(p_active_ && !p_run_);
}

void PotentialsControllerPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void PotentialsControllerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void PotentialsControllerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robor_controllers::PotentialsControllerPanel, rviz::Panel)
