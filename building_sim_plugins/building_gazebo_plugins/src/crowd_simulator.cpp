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

  sdf::ElementPtr sdf_floor = sdf->GetElement("floor");

  while (sdf_floor)
  {
    auto crowd_sim_interface =
      std::make_shared<crowd_simulator::CrowdSimInterface>();

    if (!crowd_sim_interface->read_sdf(sdf_floor))
    {
      if (!crowd_sim_interface->_menge_disabled) // enabled but contain errors
        exit(EXIT_FAILURE);
    }

    if (!crowd_sim_interface->_menge_disabled &&
      !crowd_sim_interface->init_crowd_sim())
    {
      RCLCPP_ERROR(
        crowd_sim_interface->logger(),
        "Crowd simulation failed to initialize.");
      exit(EXIT_FAILURE);
    }

    _crowd_sim_interfaces.push_back(crowd_sim_interface);
    sdf_floor = sdf_floor->GetNextElement("floor");
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
  for (auto& crowd_sim_interface:_crowd_sim_interfaces)
  {
    if (delta_sim_time > crowd_sim_interface->get_sim_time_step())
    {
      _last_sim_time = update_info.simTime;
      if (crowd_sim_interface->_menge_disabled)
      {
        crowd_sim_interface->_objects_count =
          crowd_sim_interface->get_num_objects();
        for (size_t id = 0; id < crowd_sim_interface->_objects_count; id++)
        {
          auto name = crowd_sim_interface->get_internal_agent(id);
          gazebo::physics::ModelPtr model_ptr = _world->ModelByName(name);
          gazebo::physics::ActorPtr actor_ptr =
            boost::dynamic_pointer_cast<gazebo::physics::Actor>(model_ptr);
          actor_ptr->SetScriptTime(
            actor_ptr->ScriptTime() + delta_sim_time);
        }
      }
      else
      {
        crowd_sim_interface->one_step_sim();
        _update_all_objects(crowd_sim_interface, delta_sim_time);
      }
    }
  }
}

//============================================
void CrowdSimulatorPlugin::_update_all_objects(
  std::shared_ptr<crowd_simulator::CrowdSimInterface> crowd_sim_interface,
  double delta_sim_time)
{
  for (size_t id = 0; id < crowd_sim_interface->_objects_count; id++)
  {
    ObjectPtr obj_ptr = crowd_sim_interface->get_object_by_id(id);
    gazebo::physics::ModelPtr model_ptr = _world->ModelByName(
      obj_ptr->model_name);

    //update external agents
    if (obj_ptr->agent_ptr->_external)
    {
      ignition::math::Pose3d pose = model_ptr->WorldPose();
      crowd_sim_interface->update_external_agent<ignition::math::Pose3d>(
        obj_ptr->agent_ptr, pose);
      continue;
    }

    //update internal agents
    auto type_ptr = crowd_sim_interface->_model_type_db_ptr->get(
      obj_ptr->type_name);
    _update_internal_object(crowd_sim_interface, delta_sim_time, obj_ptr,
      model_ptr, type_ptr);
  }
}

//============================================
void CrowdSimulatorPlugin::_update_internal_object(
  std::shared_ptr<crowd_simulator::CrowdSimInterface> crowd_sim_interface,
  double delta_sim_time,
  const ObjectPtr object_ptr,
  const gazebo::physics::ModelPtr model_ptr,
  const crowd_simulator::ModelTypeDatabase::RecordPtr type_ptr)
{
  if (!object_ptr)
  {
    RCLCPP_ERROR(
      crowd_sim_interface->logger(), "Null objectPtr when update Object!");
    return;
  }
  if (!model_ptr)
  {
    RCLCPP_ERROR(
      crowd_sim_interface->logger(), "Null modelPtr when update Object!");
    return;
  }

  //update pose from menge to gazebo
  ignition::math::Pose3d pose =
    crowd_sim_interface->get_agent_pose<ignition::math::Pose3d>(
    object_ptr->agent_ptr,
    delta_sim_time);

  gazebo::physics::ActorPtr actor_ptr =
    boost::dynamic_pointer_cast<gazebo::physics::Actor>(model_ptr);

  auto pos_old = actor_ptr->WorldPose().Pos();
  auto pos_new = pose.Pos();
  auto delta_dist_vector = pos_new - pos_old;
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
    delta_dist < crowd_sim_interface->get_switch_anim_distance_th() &&
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

  anim_pose.Pos().Z(pos_old.Z());
  actor_ptr->SetWorldPose(anim_pose);
}

//============================================
void CrowdSimulatorPlugin::_init_spawned_agents()
{
  for (auto& crowd_sim_interface:_crowd_sim_interfaces)
  {
    crowd_sim_interface->_objects_count =
      crowd_sim_interface->get_num_objects();
    if (crowd_sim_interface->_menge_disabled)
    {
      for (size_t id = 0; id < crowd_sim_interface->_objects_count; id++)
      {
        auto name = crowd_sim_interface->get_internal_agent(id);
        gazebo::physics::ModelPtr model_ptr = _world->ModelByName(name);
        gazebo::physics::ActorPtr actor_ptr =
          boost::dynamic_pointer_cast<gazebo::physics::Actor>(model_ptr);
        gazebo::physics::TrajectoryInfoPtr trajectory_info(new gazebo::physics::
          TrajectoryInfo()); //matches the actor skeleton
        trajectory_info->type = "idle";
        actor_ptr->SetCustomTrajectory(trajectory_info);
        actor_ptr->SetStatic(false);
      }
      _initialized = true;
    }
    else
    {
      for (size_t id = 0; id < crowd_sim_interface->_objects_count; ++id)
      {
        ObjectPtr obj_ptr = crowd_sim_interface->get_object_by_id(id);
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
          gazebo::physics::TrajectoryInfoPtr trajectory_info(
            new gazebo::physics::TrajectoryInfo()); //matches the actor skeleton

          crowd_simulator::ModelTypeDatabase::RecordPtr type_ptr =
            crowd_sim_interface->_model_type_db_ptr->get(obj_ptr->type_name);
          trajectory_info->type = type_ptr->animation;
          // set each keyframe duration as the sim_time_step
          trajectory_info->duration = crowd_sim_interface->get_sim_time_step();
          actor_ptr->SetCustomTrajectory(trajectory_info);
          actor_ptr->SetStatic(false);

          //check actor has idle animation
          for (auto idle_anim : crowd_sim_interface->get_switch_anim_name())
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
        crowd_sim_interface->logger(),
        "Gazebo models all loaded! Start simulating...");
    }
  }
}

// insert the plugin
GZ_REGISTER_WORLD_PLUGIN(CrowdSimulatorPlugin)
} //namespace crowd_simulation_gazebo