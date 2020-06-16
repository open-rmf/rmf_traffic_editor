#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>

#include "utils.hpp"

namespace building_gazebo_plugins {

//==============================================================================
class Door
{
public:

  bool _debuggable;

  Door(const bool debuggable,
      const gazebo::physics::JointPtr& joint,
       const MotionParams& params,
       const bool flip_direction = false)
  : _debuggable(debuggable),
    _joint(joint),
    _params(params)
  {
    if (flip_direction)
    {
      _closed_position = _joint->LowerLimit(0);
      _open_position = _joint->UpperLimit(0);
    }
    else
    {
      _closed_position = _joint->UpperLimit(0);
      _open_position = _joint->LowerLimit(0);
    }
  }

  bool is_open() const
  {
    return std::abs(_open_position - _joint->Position(0)) <= _params.dx_min;
  }

  bool is_closed() const
  {
    return std::abs(_closed_position - _joint->Position(0)) <= _params.dx_min;
  }

  void open(double dt)
  {
    _set_door_command(_open_position, dt);
  }

  void close(double dt)
  {
    _set_door_command(_closed_position, dt);
  }


private:

  void _set_door_command(const double target, const double dt)
  {
    double dx = target - _joint->Position(0);

    if (std::abs(dx) < _params.dx_min/2.0)
      dx = 0.0;

    const double door_v = compute_desired_rate_of_change(
      dx, _joint->GetVelocity(0), _params, dt);

    _joint->SetParam("vel", 0, door_v);
    _joint->SetParam("fmax", 0, _params.f_max);
  }

  gazebo::physics::JointPtr _joint;
  MotionParams _params;

  double _open_position;
  double _closed_position;

};

class DoorPlugin : public gazebo::ModelPlugin
{
private:
  gazebo::event::ConnectionPtr _update_connection;
  gazebo_ros::Node::SharedPtr _ros_node;
  gazebo::physics::ModelPtr _model;

  using DoorMode = rmf_door_msgs::msg::DoorMode;
  using DoorState = rmf_door_msgs::msg::DoorState;
  rclcpp::Publisher<DoorState>::SharedPtr _door_state_pub;

  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  rclcpp::Subscription<DoorRequest>::SharedPtr _door_request_sub;

  std::vector<Door> _doors;

  DoorState _state;
  DoorRequest _request;

  double _last_update_time;
  double _last_pub_time;

  bool _initialized = false;

public:
  DoorPlugin()
  : _last_update_time(0),
    _last_pub_time(0)
  {
    // Do nothing
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    _ros_node = gazebo_ros::Node::Get(sdf);
    _model = model;

    RCLCPP_INFO(
          _ros_node->get_logger(),
          "Loading DoorPlugin for [%s]",
          _model->GetName().c_str());

    MotionParams params;
    get_sdf_param_if_available<double>(sdf, "v_max_door", params.v_max);
    get_sdf_param_if_available<double>(sdf, "a_max_door", params.a_max);
    get_sdf_param_if_available<double>(sdf, "a_nom_door", params.a_nom);
    get_sdf_param_if_available<double>(sdf, "dx_min_door", params.dx_min);
    get_sdf_param_if_available<double>(sdf, "f_max_door", params.f_max);

    sdf::ElementPtr door_element;
    std::string left_door_joint_name;
    std::string right_door_joint_name;
    std::string door_type;
    if (!get_element_required(sdf, "door", door_element) ||
        !get_sdf_attribute_required<std::string>(
          door_element, "left_joint_name", left_door_joint_name) ||
        !get_sdf_attribute_required<std::string>(
          door_element, "right_joint_name", right_door_joint_name) ||
        !get_sdf_attribute_required<std::string>(
          door_element, "type", door_type))
    {
      RCLCPP_ERROR(
            _ros_node->get_logger(),
            " -- Missing required parameters for [%s] plugin",
            _model->GetName().c_str());
      return;
    }

    if (left_door_joint_name == "empty_joint" &&
        right_door_joint_name == "empty_joint")
    {
      RCLCPP_ERROR(
          _ros_node->get_logger(),
          " -- Both door joint names are missing for [%s] plugin, at least one"
          " is required", _model->GetName().c_str());
      return;
    }

    if (left_door_joint_name != "empty_joint")
    {
      const auto left_door_joint = _model->GetJoint(left_door_joint_name);
      if (!left_door_joint)
      {
        RCLCPP_ERROR(
              _ros_node->get_logger(),
              " -- Model is missing the left door joint [%s]",
              left_door_joint_name.c_str());
        return;
      }
      _doors.emplace_back(_model->GetName() == "chart_lift_door",
                          left_door_joint, params);
    }

    if (right_door_joint_name != "empty_joint")
    {
      const auto right_door_joint = _model->GetJoint(right_door_joint_name);
      if (!right_door_joint)
      {
        RCLCPP_ERROR(
              _ros_node->get_logger(),
              " -- Model is missing the right door joint [%s]",
              right_door_joint_name.c_str());
        return;
      }
      _doors.emplace_back(_model->GetName() == "chart_lift_door",
                          right_door_joint, params, true);
    }

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&DoorPlugin::on_update, this));

    _door_state_pub = _ros_node->create_publisher<DoorState>(
          "/door_states", rclcpp::SystemDefaultsQoS());

    _door_request_sub = _ros_node->create_subscription<DoorRequest>(
          "/door_requests", rclcpp::SystemDefaultsQoS(),
          [&](DoorRequest::UniquePtr msg)
    {
      if (msg->door_name == _state.door_name)
        _request = *msg;
    });

    _state.door_name = model->GetName();

    // Set the mode to closed by default
    _request.requested_mode.value = DoorMode::MODE_CLOSED;

    // random start time offset to prevent state message crossfire
    _last_pub_time = ((double) std::rand()) / ((double) (RAND_MAX));

    _initialized = true;

    RCLCPP_INFO(
          _ros_node->get_logger(),
          "Finished loading [%s]",
          _model->GetName().c_str());
  }

private:

  bool all_doors_open() const
  {
    for (const auto& door : _doors)
    {
      if (!door.is_open())
        return false;
    }

    return true;
  }

  bool all_doors_closed() const
  {
    for (const auto& door : _doors)
    {
      if (!door.is_closed())
        return false;
    }

    return true;
  }

  void on_update()
  {
    if (!_initialized)
      return;

    const double t = _model->GetWorld()->SimTime().Double();
    const double dt = t - _last_update_time;
    _last_update_time = t;

    if (_request.requested_mode.value == DoorMode::MODE_OPEN)
    {
      for (auto& door : _doors)
        door.open(dt);
    }
    else
    {
      for (auto& door : _doors)
        door.close(dt);
    }

    if (t - _last_pub_time >= 1.0)
    {
      _last_pub_time = t;

      _state.door_time = _ros_node->now();
      if (all_doors_open())
      {
        _state.current_mode.value = DoorMode::MODE_OPEN;
      }
      else if (all_doors_closed())
      {
        _state.current_mode.value = DoorMode::MODE_CLOSED;
      }
      else
      {
        _state.current_mode.value = DoorMode::MODE_MOVING;
      }

      _door_state_pub->publish(_state);
    }
  }
 
};

GZ_REGISTER_MODEL_PLUGIN(DoorPlugin)
} // namespace building_gazebo_plugins
