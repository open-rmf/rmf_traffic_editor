#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/lift_common.hpp>

using namespace building_sim_common;

namespace building_sim_gazebo {
//==============================================================================

class LiftPlugin : public gazebo::ModelPlugin
{
private:
  // Gazebo items
  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
  gazebo::physics::JointPtr _cabin_joint_ptr;
  gazebo_ros::Node::SharedPtr _ros_node;

  std::unique_ptr<LiftCommon> _lift_common = nullptr;

  bool _initialized;

public:
  LiftPlugin()
  {
    _initialized = false;
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    _ros_node = gazebo_ros::Node::Get(sdf);
    _model = model;

    RCLCPP_INFO(_ros_node->get_logger(),
      "Loading LiftPlugin for [%s]",
      _model->GetName().c_str());

    // load Lift object
    _lift_common = LiftCommon::make(_model->GetName(), _ros_node, sdf);
    if (!_lift_common)
    {
      RCLCPP_ERROR(_ros_node->get_logger(),
        "Failed when loading [%s]",
        _model->GetName().c_str());
      return;
    }

    _cabin_joint_ptr = _model->GetJoint(_lift_common->get_joint_name());
    if (!_cabin_joint_ptr)
    {
      RCLCPP_ERROR(_ros_node->get_logger(),
        " -- Model is missing the joint [%s]",
        _lift_common->get_joint_name().c_str());
      return;
    }

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&LiftPlugin::on_update, this));

    _cabin_joint_ptr->SetPosition(0, _lift_common->get_elevation());

    RCLCPP_INFO(_ros_node->get_logger(),
      "Finished loading [%s]",
      _model->GetName().c_str());

    _initialized = true;
  }

private:
  void on_update()
  {
    if (!_initialized)
      return;

    const double t = _model->GetWorld()->SimTime().Double();
    const double position = _cabin_joint_ptr->Position(0);
    const double velocity = _cabin_joint_ptr->GetVelocity(0);

    // Send update request
    auto result = _lift_common->update(t, position, velocity);

    _cabin_joint_ptr->SetParam("vel", 0, result.velocity);
    _cabin_joint_ptr->SetParam("fmax", 0, result.fmax);
  }
};

GZ_REGISTER_MODEL_PLUGIN(LiftPlugin)

} // namespace building_sim_gazebo
