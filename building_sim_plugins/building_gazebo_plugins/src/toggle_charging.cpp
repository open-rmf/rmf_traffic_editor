/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>

// GUI Plugin that creates buttons for enabling/disabling slotcar charging
// and publishes any change in state
class ToggleCharging : public gazebo::GUIPlugin
{
  Q_OBJECT
  gazebo::transport::NodePtr _node;
  gazebo::transport::PublisherPtr _charge_state_pub;

  bool _enable_charge = true;
  bool _enable_instant_charge = false;
  bool _enable_drain = true;
  const std::string _enable_charge_str = "_enable_charge";
  const std::string _enable_instant_charge_str = "_enable_instant_charge";
  const std::string _enable_drain_str = "_enable_drain";

public:
  ToggleCharging()
  {
    printf("ToggleCharging::ToggleCharging()\n");
    _node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    _node->Init();
    _charge_state_pub = _node->Advertise<gazebo::msgs::Selection>(
      "/charge_state");

    QVBoxLayout* vbox = new QVBoxLayout;

    QPushButton* charge_button = create_button(
      "Allow Charging", _enable_charge);
    connect(
      charge_button,
      &QAbstractButton::clicked,
      [this]()
      {
        this->_enable_charge = !this->_enable_charge;
        this->button_clicked(_enable_charge_str, this->_enable_charge);
      });
    vbox->addWidget(charge_button);

    QPushButton* instant_charge_button = create_button(
      "Instant Charging", _enable_instant_charge);
    connect(
      instant_charge_button,
      &QAbstractButton::clicked,
      [this]()
      {
        this->_enable_instant_charge = !this->_enable_instant_charge;
        this->button_clicked(_enable_instant_charge_str,
        this->_enable_instant_charge);
      });
    vbox->addWidget(instant_charge_button);

    QPushButton* drain_button = create_button(
      "Allow Battery Drain", _enable_drain);
    connect(
      drain_button,
      &QAbstractButton::clicked,
      [this]()
      {
        this->_enable_drain = !this->_enable_drain;
        this->button_clicked(_enable_drain_str, this->_enable_drain);
      });
    vbox->addWidget(drain_button);

    setLayout(vbox);
    this->move(0, 80);
  }

  virtual ~ToggleCharging()
  {
  }

  void button_clicked(const std::string& name, bool selected)
  {
    gazebo::msgs::Selection charge_state_msg;
    charge_state_msg.set_name(name);
    charge_state_msg.set_selected(selected);
    charge_state_msg.set_id(1); // id not necessary for current use case
    _charge_state_pub->Publish(charge_state_msg);
  }

private:
  QPushButton* create_button(const std::string& button_nm, bool init_val)
  {
    auto new_btn = new QPushButton(QString::fromStdString(button_nm));
    new_btn->setCheckable(true);
    new_btn->setChecked(init_val);
    return new_btn;
  }
};

#include "toggle_charging.moc"

GZ_REGISTER_GUI_PLUGIN(ToggleCharging)
