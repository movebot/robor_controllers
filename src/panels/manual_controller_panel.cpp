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

#include "robor_controllers/panels/manual_controller_panel.h"

using namespace robor_controllers;

ManualControllerPanel::ManualControllerPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("manual_controller") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  keys_pub_ = nh_.advertise<geometry_msgs::Twist>("keys", 10);
  getParams();

  std::fill_n(keys_, 6, 0);

  activate_checkbox_ = new QCheckBox("On/Off");
  pub_ref_vel_checkbox_ = new QCheckBox("Publish reference velocity (need restart)");

  joy_button_        = new QPushButton("Joy");
  keys_button_       = new QPushButton("Keys");
  left_button_       = new QPushButton("A");
  right_button_      = new QPushButton("D");
  up_button_         = new QPushButton("W");
  down_button_       = new QPushButton("S");
  rot_left_button_   = new QPushButton("Q");
  rot_right_button_  = new QPushButton("E");

  joy_button_->setCheckable(true);
  joy_button_->setMinimumSize(50, 50);
  joy_button_->setMaximumSize(50, 50);

  keys_button_->setCheckable(true);
  keys_button_->setMinimumSize(50, 50);
  keys_button_->setMaximumSize(50, 50);

  up_button_->setMinimumSize(50, 50);
  up_button_->setMaximumSize(50, 50);

  down_button_->setMinimumSize(50, 50);
  down_button_->setMaximumSize(50, 50);

  left_button_->setMinimumSize(50, 50);
  left_button_->setMaximumSize(50, 50);

  right_button_->setMinimumSize(50, 50);
  right_button_->setMaximumSize(50, 50);

  rot_left_button_->setMinimumSize(50, 50);
  rot_left_button_->setMaximumSize(50, 50);

  rot_right_button_->setMinimumSize(50, 50);
  rot_right_button_->setMaximumSize(50, 50);

  k_v_input_ = new QLineEdit(QString::number(p_linear_gain_));
  k_w_input_ = new QLineEdit(QString::number(p_angular_gain_));
  time_constant_input_ = new QLineEdit(QString::number(p_time_constant_));

  QFrame* lines[4];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QString omega = QChar(0x03C9);
  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QGridLayout* buttons_layout = new QGridLayout;
  buttons_layout->addItem(margin, 0, 0, 2, 1);
  buttons_layout->addWidget(joy_button_, 0, 1, Qt::AlignLeft);
  buttons_layout->addWidget(rot_left_button_, 0, 2, Qt::AlignRight);
  buttons_layout->addWidget(up_button_, 0, 3, Qt::AlignCenter);
  buttons_layout->addWidget(rot_right_button_, 0, 4, Qt::AlignLeft);
  buttons_layout->addWidget(keys_button_, 0, 5, Qt::AlignRight);
  buttons_layout->addItem(margin, 0, 6, 2, 1);
  buttons_layout->addWidget(left_button_, 1, 2, Qt::AlignRight);
  buttons_layout->addWidget(down_button_, 1, 3, Qt::AlignCenter);
  buttons_layout->addWidget(right_button_, 1, 4, Qt::AlignLeft);

  QGridLayout* gains_layout = new QGridLayout;
  gains_layout->addItem(margin, 0, 0, 2, 1);
  gains_layout->addWidget(new QLabel("k<sub>v</sub>:"), 0, 1, Qt::AlignRight);
  gains_layout->addWidget(k_v_input_, 0, 2);
  gains_layout->addWidget(new QLabel("m/s"), 0, 3, Qt::AlignLeft);
  gains_layout->addItem(new QSpacerItem(10, 1), 0, 4);
  gains_layout->addWidget(new QLabel("k<sub>" + omega + "</sub>:"), 0, 5, Qt::AlignRight);
  gains_layout->addWidget(k_w_input_, 0, 6);
  gains_layout->addWidget(new QLabel("rad/s"), 0, 7, Qt::AlignLeft);
  gains_layout->addWidget(new QLabel("T<sub>f</sub>:"), 0, 8, Qt::AlignRight);
  gains_layout->addWidget(time_constant_input_, 0, 9);
  gains_layout->addWidget(new QLabel("s"), 0, 10, Qt::AlignLeft);
  gains_layout->addItem(margin, 0, 11, 2, 1);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(lines[0]);
  layout->addLayout(buttons_layout);
  layout->addWidget(lines[1]);
  layout->addWidget(pub_ref_vel_checkbox_);
  layout->addWidget(lines[2]);
  layout->addLayout(gains_layout);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(pub_ref_vel_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));

  connect(joy_button_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(keys_button_, SIGNAL(clicked()), this, SLOT(processInputs()));

  connect(k_v_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(k_w_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(time_constant_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));

  evaluateParams();
}

void ManualControllerPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void ManualControllerPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();
  p_pub_ref_vel_ = pub_ref_vel_checkbox_->isChecked();

  p_use_joy_ = joy_button_->isChecked();
  p_use_keys_ = keys_button_->isChecked();

  try { p_linear_gain_ = boost::lexical_cast<double>(k_v_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_linear_gain_ = 0.0; k_v_input_->setText("0.0"); }

  try { p_angular_gain_ = boost::lexical_cast<double>(k_w_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_angular_gain_ = 0.0; k_w_input_->setText("0.0"); }

  try { p_time_constant_ = boost::lexical_cast<double>(time_constant_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_time_constant_ = 0.0; time_constant_input_->setText("0.0"); }
}

void ManualControllerPanel::setParams() {
  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("publish_ref_vel", p_pub_ref_vel_);

  nh_local_.setParam("use_joy", p_use_joy_);
  nh_local_.setParam("use_keys", p_use_keys_);

  nh_local_.setParam("linear_gain", p_linear_gain_);
  nh_local_.setParam("angular_gain", p_angular_gain_);
  nh_local_.setParam("time_constant", p_time_constant_);
}

void ManualControllerPanel::getParams() {
  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("publish_ref_vel", p_pub_ref_vel_, false);

  nh_local_.param<bool>("use_joy", p_use_joy_, false);
  nh_local_.param<bool>("use_keys", p_use_keys_, false);

  nh_local_.param<double>("linear_gain", p_linear_gain_, 0.3);
  nh_local_.param<double>("angular_gain", p_angular_gain_, 0.5);
  nh_local_.param<double>("time_constant", p_time_constant_, 0.0);
}

void ManualControllerPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);

  pub_ref_vel_checkbox_->setEnabled(p_active_);
  pub_ref_vel_checkbox_->setChecked(p_pub_ref_vel_);

  joy_button_->setEnabled(p_active_);
  joy_button_->setChecked(p_use_joy_);

  keys_button_->setEnabled(p_active_);
  keys_button_->setChecked(p_use_keys_);

  up_button_->setEnabled(p_active_ && p_use_keys_);
  down_button_->setEnabled(p_active_ && p_use_keys_);
  left_button_->setEnabled(p_active_ && p_use_keys_);
  right_button_->setEnabled(p_active_ && p_use_keys_);
  rot_left_button_->setEnabled(p_active_ && p_use_keys_);
  rot_right_button_->setEnabled(p_active_ && p_use_keys_);

  k_v_input_->setEnabled(p_active_);
  k_w_input_->setEnabled(p_active_);
  time_constant_input_->setEnabled(p_active_);
}

void ManualControllerPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void ManualControllerPanel::keyPressEvent(QKeyEvent * e) {
  if (p_active_ && p_use_keys_) {
    geometry_msgs::Twist keys;

    if (e->key() == Qt::Key_W || e->key() == Qt::Key_Up) {
      up_button_->setDown(true);
      keys_[0] = 1;
    }
    if (e->key() == Qt::Key_S || e->key() == Qt::Key_Down) {
      down_button_->setDown(true);
      keys_[1] = 1;
    }
    if (e->key() == Qt::Key_A || e->key() == Qt::Key_Left) {
      left_button_->setDown(true);
      keys_[2] = 1;
    }
    if (e->key() == Qt::Key_D || e->key() == Qt::Key_Right) {
      right_button_->setDown(true);
      keys_[3] = 1;
    }
    if (e->key() == Qt::Key_Q || e->key() == Qt::Key_Delete) {
      rot_left_button_->setDown(true);
      keys_[4] = 1;
    }
    if (e->key() == Qt::Key_E || e->key() == Qt::Key_PageDown) {
      rot_right_button_->setDown(true);
      keys_[5] = 1;
    }

    keys.linear.x = keys_[0] - keys_[1];
    keys.linear.y = keys_[2] - keys_[3];
    keys.angular.z = keys_[4] - keys_[5];

    keys_pub_.publish(keys);
  }
}

void ManualControllerPanel::keyReleaseEvent(QKeyEvent * e) {
  if (p_active_ && p_use_keys_ && !e->isAutoRepeat()) {
    geometry_msgs::Twist keys;

    if (e->key() == Qt::Key_W || e->key() == Qt::Key_Up) {
      up_button_->setDown(false);
      keys_[0] = 0;
    }
    if (e->key() == Qt::Key_S || e->key() == Qt::Key_Down) {
      down_button_->setDown(false);
      keys_[1] = 0;
    }
    if (e->key() == Qt::Key_A || e->key() == Qt::Key_Left) {
      left_button_->setDown(false);
      keys_[2] = 0;
    }
    if (e->key() == Qt::Key_D || e->key() == Qt::Key_Right) {
      right_button_->setDown(false);
      keys_[3] = 0;
    }
    if (e->key() == Qt::Key_Q || e->key() == Qt::Key_Delete) {
      rot_left_button_->setDown(false);
      keys_[4] = 0;
    }
    if (e->key() == Qt::Key_E || e->key() == Qt::Key_PageDown) {
      rot_right_button_->setDown(false);
      keys_[5] = 0;
    }

    keys.linear.x = keys_[0] - keys_[1];
    keys.linear.y = keys_[2] - keys_[3];
    keys.angular.z = keys_[4] - keys_[5];

    keys_pub_.publish(keys);
  }
}

void ManualControllerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ManualControllerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robor_controllers::ManualControllerPanel, rviz::Panel)
