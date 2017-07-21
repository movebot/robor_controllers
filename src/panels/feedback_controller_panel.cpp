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

#include "robor_controllers/panels/feedback_controller_panel.h"

using namespace robor_controllers;

FeedbackControllerPanel::FeedbackControllerPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("feedback_controller") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  getParams();

  activate_checkbox_ = new QCheckBox("On/Off");
  ff_checkbox_       = new QCheckBox("Feedforward");
  gain_x_input_      = new QLineEdit(QString::number(p_gain_x_));
  gain_y_input_      = new QLineEdit(QString::number(p_gain_y_));
  gain_theta_input_  = new QLineEdit(QString::number(p_gain_theta_));
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

  QFrame* lines[5];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QString theta = QChar(0x03B8);
  QString omega = QChar(0x03C9);
  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QHBoxLayout* buttons_layout = new QHBoxLayout;
  buttons_layout->addItem(margin);
  buttons_layout->addWidget(stop_button_);
  buttons_layout->addWidget(start_button_);
  buttons_layout->addItem(margin);

  QGridLayout* input_layout = new QGridLayout;
  input_layout->addItem(margin, 0, 0, 3, 1);

  input_layout->addWidget(new QLabel("k<sub>x</sub>:"), 0, 1, Qt::AlignRight);
  input_layout->addWidget(gain_x_input_, 0, 2);
  input_layout->addWidget(new QLabel("1/s, "), 0, 3, Qt::AlignLeft);

  input_layout->addWidget(new QLabel("k<sub>y</sub>:"), 0, 4, Qt::AlignRight);
  input_layout->addWidget(gain_y_input_, 0, 5);
  input_layout->addWidget(new QLabel("1/s, "), 0, 6, Qt::AlignLeft);

  input_layout->addWidget(new QLabel("k<sub>"+ theta +"</sub>:"), 0, 7, Qt::AlignRight);
  input_layout->addWidget(gain_theta_input_, 0, 8);
  input_layout->addWidget(new QLabel("1/s"), 0, 9, Qt::AlignLeft);

  input_layout->addWidget(new QLabel("u<sub>max</sub>:"), 1, 1, Qt::AlignRight);
  input_layout->addWidget(max_u_input_, 1, 2);
  input_layout->addWidget(new QLabel("m/s, "), 1, 3, Qt::AlignLeft);

  input_layout->addWidget(new QLabel("v<sub>max</sub>:"), 1, 4, Qt::AlignRight);
  input_layout->addWidget(max_v_input_, 1, 5);
  input_layout->addWidget(new QLabel("m/s, "), 1, 6, Qt::AlignLeft);

  input_layout->addWidget(new QLabel(omega + "<sub>max</sub>:"), 1, 7, Qt::AlignRight);
  input_layout->addWidget(max_w_input_, 1, 8);
  input_layout->addWidget(new QLabel("rad/s"), 1, 9, Qt::AlignLeft);

  input_layout->addItem(margin, 0, 10, 3, 1);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(lines[0]);
  layout->addLayout(buttons_layout);
  layout->addWidget(lines[1]);
  layout->addWidget(ff_checkbox_);
  layout->addWidget(lines[2]);
  layout->addLayout(input_layout);
  layout->addWidget(lines[3]);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(ff_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(start_button_, SIGNAL(clicked()), this, SLOT(start()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop()));

  evaluateParams();
}

void FeedbackControllerPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void FeedbackControllerPanel::start() {
  p_run_ = true;
  processInputs();
}

void FeedbackControllerPanel::stop() {
  p_run_ = false;
  processInputs();
}

void FeedbackControllerPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();
  p_ff_active_ = ff_checkbox_->isChecked();
  p_run_ = p_run_ && p_active_;

  try { p_gain_x_ = boost::lexical_cast<double>(gain_x_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_gain_x_ = 0.0; gain_x_input_->setText("0.0"); }

  try { p_gain_y_ = boost::lexical_cast<double>(gain_y_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_gain_y_ = 0.0; gain_y_input_->setText("0.0"); }

  try { p_gain_theta_ = boost::lexical_cast<double>(gain_theta_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_gain_theta_ = 0.0; gain_theta_input_->setText("0.0"); }

  try { p_max_u_ = boost::lexical_cast<double>(max_u_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_u_ = 0.0; max_u_input_->setText("0.0"); }

  try { p_max_v_ = boost::lexical_cast<double>(max_v_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_v_ = 0.0; max_v_input_->setText("0.0"); }

  try { p_max_w_ = boost::lexical_cast<double>(max_w_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_w_ = 0.0; max_w_input_->setText("0.0"); }
}

void FeedbackControllerPanel::setParams() {
  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("run", p_run_);
  nh_local_.setParam("use_ff", p_ff_active_);

  nh_local_.setParam("gain_x", p_gain_x_);
  nh_local_.setParam("gain_y", p_gain_y_);
  nh_local_.setParam("gain_theta", p_gain_theta_);

  nh_local_.setParam("max_u", p_max_u_);
  nh_local_.setParam("max_v", p_max_v_);
  nh_local_.setParam("max_w", p_max_w_);
}

void FeedbackControllerPanel::getParams() {
  p_active_ = nh_local_.param("active", false);
  p_run_ = nh_local_.param("run", false);
  p_ff_active_ = nh_local_.param("use_ff", false);

  p_gain_x_ = nh_local_.param("gain_x", 0.0);
  p_gain_y_ = nh_local_.param("gain_y", 0.0);
  p_gain_theta_ = nh_local_.param("gain_theta", 0.0);

  p_max_u_ = nh_local_.param("max_u", 0.0);
  p_max_v_ = nh_local_.param("max_v", 0.0);
  p_max_w_ = nh_local_.param("max_w", 0.0);
}

void FeedbackControllerPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);

  if (!p_active_)
    p_run_ = false;

  start_button_->setEnabled(p_active_ && !p_run_);
  start_button_->setChecked(p_run_);

  stop_button_->setEnabled(p_active_ && p_run_);
  stop_button_->setChecked(!p_run_);

  ff_checkbox_->setEnabled(p_active_ && !p_run_);
  ff_checkbox_->setChecked(p_ff_active_);

  gain_x_input_->setEnabled(p_active_ && !p_run_);
  gain_y_input_->setEnabled(p_active_ && !p_run_);
  gain_theta_input_->setEnabled(p_active_ && !p_run_);

  max_u_input_->setEnabled(p_active_ && !p_run_);
  max_v_input_->setEnabled(p_active_ && !p_run_);
  max_w_input_->setEnabled(p_active_ && !p_run_);
}

void FeedbackControllerPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void FeedbackControllerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void FeedbackControllerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robor_controllers::FeedbackControllerPanel, rviz::Panel)
