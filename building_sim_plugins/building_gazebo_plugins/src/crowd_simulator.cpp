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

#include <memory>
#include <unordered_set>
#include <regex>
#include <cstdlib>

#include <ignition/math/Pose3.hh>

#include "crowd_simulator.hpp"

namespace crowd_simulation_gazebo {

//============================================
//WorldPlugin
void CrowdSimulatorPlugin::Load(
  gazebo::physics::WorldPtr world,
  sdf::ElementPtr sdf)
{
  _world = world;

  if (!_crowd_sim_interface->read_sdf(sdf))
  {
    exit(EXIT_FAILURE);
  }

  if (!_crowd_sim_interface->init_crowd_sim())
  {
    RCLCPP_ERROR(
      _crowd_sim_interface->logger(),
      "Crowd simulation failed to initialize.");
    exit(EXIT_FAILURE);
  }

  if (!_spawn_agents_in_world())
  {
    RCLCPP_ERROR(
      _crowd_sim_interface->logger(),
      "Crowd simulation failed to spawn agents in the world.");
    exit(EXIT_FAILURE);
  }

  _update_connection_ptr = gazebo::event::Events::ConnectWorldUpdateBegin(
    [this](gazebo::common::UpdateInfo update_info)
    {
      _update(update_info);
    }
  );
}

//============================================
void CrowdSimulatorPlugin::_update(
  const gazebo::common::UpdateInfo& update_info)
{
  //first round do nothing, initialize time stamp
  if (_last_sim_time == gazebo::common::Time::Zero)
  {
    _last_sim_time = update_info.simTime;
  }

  if (!_initialized)
  {
    // not initizalied
    _init_spawned_agents();
    return;
  }

  auto delta_sim_time = (update_info.simTime - _last_sim_time).Double();
  if (delta_sim_time > _crowd_sim_interface->get_sim_time_step())
  {
    _last_sim_time = update_info.simTime;
    _crowd_sim_interface->one_step_sim();
    _update_all_objects(delta_sim_time);
  }
}

//============================================
void CrowdSimulatorPlugin::_update_all_objects(double delta_sim_time)
{
  for (size_t id = 0; id < _objects_count; id++)
  {
    ObjectPtr obj_ptr = _crowd_sim_interface->get_object_by_id(id);
    gazebo::physics::ModelPtr model_ptr = _world->ModelByName(
      obj_ptr->model_name);

    //update external agents
    if (obj_ptr->agent_ptr->_external)
    {
      ignition::math::Pose3d pose = model_ptr->WorldPose();
      _crowd_sim_interface->update_external_agent<ignition::math::Pose3d>(
        obj_ptr->agent_ptr, pose);
      continue;
    }

    //update internal agents
    auto type_ptr = _crowd_sim_interface->_model_type_db_ptr->get(
      obj_ptr->type_name);
    _update_internal_object(delta_sim_time, obj_ptr,
      model_ptr, type_ptr);
  }
}

//============================================
void CrowdSimulatorPlugin::_update_internal_object(
  double delta_sim_time,
  const ObjectPtr object_ptr,
  const gazebo::physics::ModelPtr model_ptr,
  const crowd_simulator::ModelTypeDatabase::RecordPtr type_ptr)
{
  if (!object_ptr)
  {
    RCLCPP_ERROR(
      _crowd_sim_interface->logger(), "Null objectPtr when update Object!");
    return;
  }
  if (!model_ptr)
  {
    RCLCPP_ERROR(
      _crowd_sim_interface->logger(), "Null modelPtr when update Object!");
    return;
  }

  //update pose from menge to gazebo
  ignition::math::Pose3d pose =
    _crowd_sim_interface->get_agent_pose<ignition::math::Pose3d>(
    object_ptr->agent_ptr,
    delta_sim_time);

  gazebo::physics::ActorPtr actor_ptr =
    boost::dynamic_pointer_cast<gazebo::physics::Actor>(model_ptr);

  auto delta_dist_vector = pose.Pos() - actor_ptr->WorldPose().Pos();
  // might need future work on 3D case
  // the center of human has a z_elevation, which will make the human keep walking even if he reached the target
  delta_dist_vector.Z(0.0);
  double delta_dist = delta_dist_vector.Length();

  auto init_pose = type_ptr->pose;
  ignition::math::Pose3d anim_pose(
    init_pose.x(), init_pose.y(), init_pose.z(),
    init_pose.pitch(), init_pose.roll(), init_pose.yaw());
  //update x and y coordinates
  anim_pose.Pos().X(pose.Pos().X());
  anim_pose.Pos().Y(pose.Pos().Y());

  AnimState next_state = object_ptr->get_next_state(
    delta_dist < _crowd_sim_interface->get_switch_anim_distance_th() &&
    !type_ptr->idle_animation.empty());

  auto traj_info = actor_ptr->CustomTrajectory();
  switch (next_state)
  {
    case AnimState::WALK:
      actor_ptr->SetScriptTime(
        actor_ptr->ScriptTime() + delta_dist / type_ptr->animation_speed);
      anim_pose.Rot() = pose.Rot();
      if (object_ptr->current_state != next_state)
        traj_info->type = type_ptr->animation;
      break;

    case AnimState::IDLE:
      actor_ptr->SetScriptTime(
        actor_ptr->ScriptTime() + delta_sim_time);
      anim_pose.Rot() = actor_ptr->WorldPose().Rot();
      if (object_ptr->current_state != next_state)
        traj_info->type = type_ptr->idle_animation;
      break;
  }
  object_ptr->current_state = next_state;
  actor_ptr->SetWorldPose(anim_pose);
}

//============================================
void CrowdSimulatorPlugin::_init_spawned_agents()
{
  _objects_count = _crowd_sim_interface->get_num_objects();
  for (size_t id = 0; id < _objects_count; ++id)
  {
    ObjectPtr obj_ptr = _crowd_sim_interface->get_object_by_id(id);
    // spawned agents are not fully loaded
    if (!_world->ModelByName(obj_ptr->model_name))
    {
      _initialized = false;
      return;
    }
    // all agents are loaded, set internal actors as non-static model and set custom trajectory
    // because only non-static model can interact with slotcars
    if (!obj_ptr->is_external)
    {
      gazebo::physics::ModelPtr model_ptr = _world->ModelByName(
        obj_ptr->model_name);
      gazebo::physics::ActorPtr actor_ptr =
        boost::dynamic_pointer_cast<gazebo::physics::Actor>(model_ptr);
      gazebo::physics::TrajectoryInfoPtr trajectory_info(new gazebo::physics::
        TrajectoryInfo()); //matches the actor skeleton

      crowd_simulator::ModelTypeDatabase::RecordPtr type_ptr =
        _crowd_sim_interface->_model_type_db_ptr->get(obj_ptr->type_name);
      trajectory_info->type = type_ptr->animation;
      // set each keyframe duration as the sim_time_step
      trajectory_info->duration = _crowd_sim_interface->get_sim_time_step();
      actor_ptr->SetCustomTrajectory(trajectory_info);
      actor_ptr->SetStatic(false);

      //check actor has idle animation
      for (auto idle_anim : _crowd_sim_interface->get_switch_anim_name())
      {
        if (actor_ptr->SkeletonAnimations().find(idle_anim) !=
          actor_ptr->SkeletonAnimations().end())
        {
          type_ptr->idle_animation = idle_anim;
          break;
        }
      }
    }
  }
  _initialized = true;
  RCLCPP_INFO(
    _crowd_sim_interface->logger(),
    "Gazebo models all loaded! Start simulating...");
}

//============================================
bool CrowdSimulatorPlugin::_spawn_agents_in_world()
{
  //create model in world for each internal agents
  _objects_count = _crowd_sim_interface->get_num_objects();
  for (size_t id = 0; id < _objects_count; id++)
  {
    if (!_crowd_sim_interface->get_object_by_id(id)->is_external)
    {
      auto object_ptr = _crowd_sim_interface->get_object_by_id(id);
      assert(object_ptr);
      auto type_ptr = _crowd_sim_interface->_model_type_db_ptr->get(
        object_ptr->type_name);
      assert(type_ptr);
      if (!_create_model(object_ptr->model_name, type_ptr,
        object_ptr->agent_ptr) )
      {
        RCLCPP_INFO(_crowd_sim_interface->logger(),
          "Failed to insert model [" + object_ptr->model_name + "] in world");
        return false;
      }
    }
  }
  return true;
}

//============================================
bool CrowdSimulatorPlugin::_create_model(
  const std::string& model_name,
  const crowd_simulator::ModelTypeDatabase::RecordPtr model_type_ptr,
  const crowd_simulator::AgentPtr agent_ptr)
{
  sdf::ElementPtr model_element(new sdf::Element());
  model_element->SetName("include");

  sdf::ElementPtr name_element(new sdf::Element());
  name_element->SetName("name");
  name_element->AddValue("string", model_name, true);
  model_element->InsertElement(name_element);

  sdf::ElementPtr uri_element(new sdf::Element());
  uri_element->SetName("uri");
  uri_element->AddValue("string", model_type_ptr->file_name, true);
  model_element->InsertElement(uri_element);

  sdf::ElementPtr static_element(new sdf::Element());
  static_element->SetName("static");
  static_element->AddValue("string", "False", true);
  model_element->InsertElement(static_element);

  sdf::ElementPtr pose_element(new sdf::Element());
  pose_element->SetName("pose");
  std::ostringstream oss;
  oss << agent_ptr->_pos.x() << " " << agent_ptr->_pos.y() << " " << "0 0 0 0";
  pose_element->AddValue("pose", oss.str(), true);
  model_element->InsertElement(pose_element);

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf->Root(model_element);

  assert(sdf);
  _world->InsertModelSDF(*sdf);
  RCLCPP_INFO(_crowd_sim_interface->logger(),
    "Insert actor for crowd simulator agent: [" + model_name + "] at ["+ oss.str() +
    "].");
  return true;
}

// insert the plugin
GZ_REGISTER_WORLD_PLUGIN(CrowdSimulatorPlugin)
} //namespace crowd_simulation_gazebo