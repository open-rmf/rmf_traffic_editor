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

#include <string>
#include <iostream>

#include <ignition/plugin/Register.hh>
#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

using namespace ignition;
using namespace gui;

class toggle_charging : public Plugin
{
  Q_OBJECT

private:
  bool _enable_charge = true;
  bool _enable_instant_charge = false;
  bool _enable_drain = true;
  const std::string _enable_charge_str = "_enable_charge";
  const std::string _enable_instant_charge_str = "_enable_instant_charge";
  const std::string _enable_drain_str = "_enable_drain";

  ignition::transport::Node _node;
  ignition::transport::Node::Publisher _charge_state_pub;

  void checkbox_checked(const std::string& name, bool checked);

protected slots:
  void OnEnableCharge(bool);
  void OnEnableInstantCharge(bool);
  void OnEnableDrain(bool);

public:
  toggle_charging();

  virtual void LoadConfig(const tinyxml2::XMLElement* _pluginElem)
  override;
};

toggle_charging::toggle_charging()
{
  _charge_state_pub = _node.Advertise<ignition::msgs::Selection>(
    "/charge_state");
  if (!_charge_state_pub)
  {
    std::cerr << "Error advertising topic [/charge_state]" << std::endl;
  }
}

void toggle_charging::LoadConfig(const tinyxml2::XMLElement* _pluginElem)
{
  if (!_pluginElem)
    return;

  if (this->title.empty())
    this->title = "Toggle Charging";
}

void toggle_charging::OnEnableCharge(bool checked)
{
  _enable_charge = checked;
  checkbox_checked(_enable_charge_str, checked);
}

void toggle_charging::OnEnableInstantCharge(bool checked)
{
  _enable_instant_charge = checked;
  checkbox_checked(_enable_instant_charge_str, _enable_instant_charge);
}

void toggle_charging::OnEnableDrain(bool checked)
{
  _enable_drain = checked;
  checkbox_checked(_enable_drain_str, _enable_drain);
}

void toggle_charging::checkbox_checked(
  const std::string& name, bool checked)
{
  ignition::msgs::Selection selection;
  selection.set_name(name);
  selection.set_selected(checked);
  selection.set_id(1); // Id not necessary for current use case
  _charge_state_pub.Publish(selection);
}

// Register this plugin
IGNITION_ADD_PLUGIN(toggle_charging,
  ignition::gui::Plugin)


#include "toggle_charging.moc"