
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <charge_msgs/msg/charge_state.hpp>

#include <ignition/plugin/Register.hh>
#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

using namespace ignition;
using namespace gui;

using ChargeState = charge_msgs::msg::ChargeState;

class toggle_charging : public Plugin
{
  Q_OBJECT

  private:
    rclcpp::Node::SharedPtr _ros_node;
    bool _enable_charge = false;
    bool _enable_instant_charge = false;
    bool _enable_drain = false;
    rclcpp::Publisher<ChargeState>::SharedPtr _charge_state_pub;

    void checkbox_checked();

  protected slots:
    void OnEnableCharge(bool);
    void OnEnableInstantCharge(bool);
    void OnEnableDrain(bool);

  public:
    toggle_charging();

    virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem)
        override;
};

toggle_charging::toggle_charging()
{
  if (!rclcpp::ok())
    rclcpp::init(0, 0);
  _ros_node = std::make_shared<rclcpp::Node>("toggle_charging");

  _charge_state_pub = _ros_node->create_publisher<ChargeState>(
    "/charge_state", rclcpp::SystemDefaultsQoS());
}

void toggle_charging::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (!_pluginElem)
    return;
  
  if (this->title.empty())
    this->title = "Toggle Charging";
}

void toggle_charging::OnEnableCharge(bool checked)
{
  _enable_charge = checked;
  checkbox_checked();
}

void toggle_charging::OnEnableInstantCharge(bool checked)
{
  _enable_instant_charge = checked;
  checkbox_checked();
}

void toggle_charging::OnEnableDrain(bool checked)
{
  _enable_drain = checked;
  checkbox_checked();
}

void toggle_charging::checkbox_checked()
{
  ChargeState _state;
  _state.time = rclcpp::Time {0, 0, RCL_ROS_TIME};
  _state.enable_charge = _enable_charge;
  _state.enable_drain = _enable_drain;
  _state.enable_instant_charge = _enable_instant_charge;
  _charge_state_pub->publish(_state);
}

// Register this plugin
IGNITION_ADD_PLUGIN(toggle_charging,
                    ignition::gui::Plugin)


#include "toggle_charging.moc"