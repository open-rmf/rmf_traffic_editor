#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/door_common.hpp>

using namespace building_sim_common;

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
  gazebo::physics::ModelPtr _model;
  std::shared_ptr<DoorCommon> _door_common = nullptr;
  
  std::vector<Door> _doors;

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
    auto _ros_node = gazebo_ros::Node::Get(sdf);
    _model = model;

    RCLCPP_INFO(
      _ros_node->get_logger(),
      "Loading DoorPlugin for [%s]",
      _model->GetName().c_str());

    _door_common = DoorCommon::make(
         _model->GetName(),
        _ros_node,
        sdf);

    if (!_door_common)
      return;

    const auto left_door_joint_name = _door_common->left_door_joint_name();
    const auto right_door_joint_name = _door_common->right_door_joint_name();

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
        left_door_joint, _door_common->params());
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
        right_door_joint, _door_common->params(), true);
    }

    _initialized = true;

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&DoorPlugin::on_update, this));

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

    if (_door_common->requested_mode().value == DoorMode::MODE_OPEN)
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
      const int32_t t_sec = static_cast<int32_t>(t);
      const uint32_t t_nsec =
        static_cast<uint32_t>((t-static_cast<double>(t_sec)) *1e9);
      const rclcpp::Time now{t_sec, t_nsec, RCL_ROS_TIME};

      if (all_doors_open())
      {
        _door_common->publish_state(DoorMode::MODE_OPEN, now);
      }
      else if (all_doors_closed())
      {
        _door_common->publish_state(DoorMode::MODE_CLOSED, now);
      }
      else
      {
        _door_common->publish_state(DoorMode::MODE_MOVING, now);
      }
    }
  }

};

GZ_REGISTER_MODEL_PLUGIN(DoorPlugin)
