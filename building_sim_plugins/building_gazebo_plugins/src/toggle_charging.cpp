#include <functional>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <charge_msgs/msg/charge_state.hpp>

using std::string;
using ChargeState = charge_msgs::msg::ChargeState;

class ToggleCharging : public gazebo::GUIPlugin
{
  Q_OBJECT
  gazebo_ros::Node::SharedPtr _ros_node;

  bool _enable_charge = false;
  bool _enable_instant_charge = false;
  bool _enable_drain = false;
  rclcpp::Publisher<ChargeState>::SharedPtr _charge_state_pub;

public:
  virtual ~ToggleCharging()
  {
  }

  void Load(sdf::ElementPtr sdf)
  {
    printf("ToggleCharging::Load()\n");
    _ros_node = gazebo_ros::Node::Get(sdf);

    _charge_state_pub = _ros_node->create_publisher<ChargeState>(
      "/charge_state", rclcpp::SystemDefaultsQoS());

    QHBoxLayout* hbox = new QHBoxLayout;

    QPushButton* charge_button =
      new QPushButton(QString::fromStdString("Allow Charging"));
    charge_button->setCheckable(true);
    charge_button->setChecked(false);
    connect(
      charge_button,
      &QAbstractButton::clicked,
      [this]()
      {
        this->_enable_charge = !this->_enable_charge;
        this->button_clicked();
      });
    hbox->addWidget(charge_button);

    QPushButton* instant_charge_button =
      new QPushButton(QString::fromStdString("Instant Charging"));
    instant_charge_button->setCheckable(true);
    instant_charge_button->setChecked(false);
    connect(
      instant_charge_button,
      &QAbstractButton::clicked,
      [this]()
      {
        this->_enable_instant_charge = !this->_enable_instant_charge;
        this->button_clicked();
      });
    hbox->addWidget(instant_charge_button);

    QPushButton* drain_button =
      new QPushButton(QString::fromStdString("Allow Battery Drain"));
    drain_button->setCheckable(true);
    drain_button->setChecked(false);
    connect(
      drain_button,
      &QAbstractButton::clicked,
      [this]()
      {
        this->_enable_drain = !this->_enable_drain;
        this->button_clicked();
      });
    hbox->addWidget(drain_button);

    setLayout(hbox);
    this->move(0, 50);
  }

  void button_clicked()
  {
    ChargeState _state;
    _state.time = rclcpp::Time {0, 0, RCL_ROS_TIME};
    _state.enable_charge = _enable_charge;
    _state.enable_drain = _enable_drain;
    _state.enable_instant_charge = _enable_instant_charge;
    _charge_state_pub->publish(_state);
  }
};

#include "toggle_charging.moc"

GZ_REGISTER_GUI_PLUGIN(ToggleCharging)
