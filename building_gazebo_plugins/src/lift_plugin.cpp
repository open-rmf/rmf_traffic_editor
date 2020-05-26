// gazebo & ROS
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

// msgs
#include <rmf_lift_msgs/msg/lift_state.hpp>
#include <rmf_lift_msgs/msg/lift_request.hpp>

#include "door.hpp"
#include "lift.hpp"
#include "utils.hpp"

using namespace gazebo;

namespace building_gazebo_plugins {

class LiftPlugin : public gazebo::ModelPlugin
{
private:
  // Gazebo items
  event::ConnectionPtr _update_connection;
  physics::ModelPtr _model;
  std::unique_ptr<Lift> _lift;

  // ROS items
  using LiftState = rmf_lift_msgs::msg::LiftState;
  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
  gazebo_ros::Node::SharedPtr _ros_node;
  rclcpp::Publisher<LiftState>::SharedPtr _lift_state_pub;
  rclcpp::Subscription<LiftRequest>::SharedPtr _lift_request_sub;

  LiftState _state;
  LiftRequest _request;

  // Runtime items
  bool _load_complete;
  double _last_update_time;
  double _last_pub_time;

public:
  LiftPlugin()
  : _last_update_time(0),
    _last_pub_time(0)
  {
    _load_complete = false;
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    _ros_node = gazebo_ros::Node::Get(sdf);
    _model = model;

    RCLCPP_INFO(
          _ros_node->get_logger(),
          "Loading LiftPlugin for [%s]",
          _model->GetName().c_str());

    // load Lift object
    _lift = std::make_unique<Lift>(_model, sdf);
    _load_complete = _lift->load_successful();

    // initialize pub & sub
    _lift_state_pub = _ros_node->create_publisher<LiftState>(
          "/lift_states", rclcpp::SystemDefaultsQoS());

    _lift_request_sub = _ros_node->create_subscription<LiftRequest>(
          "/lift_requests", rclcpp::SystemDefaultsQoS(),
          [&](LiftRequest::UniquePtr msg)
    {
      if (msg->lift_name == _state.lift_name)
        _request = *msg;
    });

    // initialize _state
    _state.lift_name = model->GetName();
    _state.available_floors.clear();
    for (const std::string& floor_name : _lift->get_floor_names())
    {
      _state.available_floors.push_back(floor_name);
    }

    _last_update_time = _model->GetWorld()->SimTime().Double();
    
    // link to Gazebo world if everything is completely loaded
    if (!_load_complete)
    {
      RCLCPP_ERROR(
            _ros_node->get_logger(),
            "Failed when loading [%s]",
            _model->GetName().c_str());
    }
    else
    {
      _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&LiftPlugin::on_update, this));
    
      RCLCPP_INFO(
            _ros_node->get_logger(),
            "Finished loading [%s]",
            _model->GetName().c_str());
    }
  }

private:
  void on_update()
  {
    if (!_load_complete)
      return;
    
    const double t = _model->GetWorld()->SimTime().Double();
    const double dt = t - _last_update_time;
    _last_update_time = t;

    if (_request.lift_name == _lift->get_name())
    {
      _lift->set_lift_request(_request.destination_floor, static_cast<DoorStateEnum>(_request.door_state));
    }
    // at 1 Hz, publish lift state
    if (t - _last_pub_time > 1)
    {
      _last_pub_time = t;

      _state.lift_time = rclcpp::Time(t);
      _state.current_floor = _lift->get_current_floor();
      _state.destination_floor = _lift->get_destination_floor();
      _state.door_state = static_cast<uint8_t>(_lift->get_overall_door_state());
      _state.motion_state = static_cast<uint8_t>(_lift->get_cabin_state());
      //TODO: add lift mode
      _state.current_mode = 1;

      _lift_state_pub->publish(_state);
    }

    // at 1000 Hz, update lift controllers
    _lift->update(dt);
  }

};

GZ_REGISTER_MODEL_PLUGIN(LiftPlugin)
} // namespace building_gazebo_plugins