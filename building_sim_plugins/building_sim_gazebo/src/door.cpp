#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/door_common.hpp>

using namespace building_sim_common;

namespace building_sim_gazebo {
//==============================================================================

class DoorPlugin : public gazebo::ModelPlugin
{
private:
  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
  gazebo::physics::JointPtr _left_door_joint = nullptr;
  gazebo::physics::JointPtr _right_door_joint = nullptr;

  std::shared_ptr<DoorCommon> _door_common = nullptr;

  bool _initialized = false;

public:
  DoorPlugin()
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
      _left_door_joint = _model->GetJoint(left_door_joint_name);
      if (!_left_door_joint)
      {
        RCLCPP_ERROR(
          _ros_node->get_logger(),
          " -- Model is missing the left door joint [%s]",
          left_door_joint_name.c_str());
        return;
      }
      _door_common->add_left_door(_left_door_joint->UpperLimit(0),
        _left_door_joint->LowerLimit(0));
    }

    if (right_door_joint_name != "empty_joint")
    {
      _right_door_joint = _model->GetJoint(right_door_joint_name);
      if (!_right_door_joint)
      {
        RCLCPP_ERROR(
          _ros_node->get_logger(),
          " -- Model is missing the right door joint [%s]",
          right_door_joint_name.c_str());
        return;
      }
      _door_common->add_right_door(_right_door_joint->UpperLimit(0),
        _right_door_joint->LowerLimit(0));
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

  void on_update()
  {
    if (!_initialized)
      return;

    const double t = _model->GetWorld()->SimTime().Double();

    DoorCommon::DoorUpdateRequest request;
    if (_left_door_joint)
    {
      request.left_position = std::make_shared<double>(
        _left_door_joint->Position(0));
      request.left_velocity = std::make_shared<double>(
        _left_door_joint->GetVelocity(0));
    }
    if (_right_door_joint)
    {
      request.right_position = std::make_shared<double>(
        _right_door_joint->Position(0));
      request.right_velocity = std::make_shared<double>(
        _right_door_joint->GetVelocity(0));
    }

    auto result = _door_common->update(t, request);

    // Apply motions to the joints
    if (_left_door_joint)
    {
      _left_door_joint->SetParam("vel", 0, *result.left_velocity);
      _left_door_joint->SetParam("fmax", 0, *result.fmax);
    }
    if (_right_door_joint)
    {
      _right_door_joint->SetParam("vel", 0, *result.right_velocity);
      _right_door_joint->SetParam("fmax", 0, *result.fmax);
    }
  }

};

GZ_REGISTER_MODEL_PLUGIN(DoorPlugin)

} // namespace building_sim_gazebo
