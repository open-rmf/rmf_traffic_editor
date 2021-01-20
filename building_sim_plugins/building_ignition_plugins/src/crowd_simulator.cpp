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

#include <regex>
#include <cstdlib>

#include <sdf/Actor.hh>

#include <ignition/math/Pose3.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>

#include "crowd_simulator.hpp"

namespace crowd_simulation_ign {

//=================================================
void CrowdSimulatorPlugin::Configure(
  const ignition::gazebo::Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf,
  ignition::gazebo::EntityComponentManager& ecm,
  ignition::gazebo::EventManager& event_mgr)
{
  _world = std::make_shared<ignition::gazebo::Model>(entity);
  RCLCPP_INFO(rclcpp::get_logger("crowdsim"),
    "Initializing world plugin with name: " + _world->Name(ecm));
  _world_name = _world->Name(ecm);

  auto sdf_floor = sdf->GetElementImpl("floor");

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
        "Error loading crowd simulator plugin. Load [ Menge ] failed!");
      exit(EXIT_FAILURE);
    }

    _crowd_sim_interfaces.push_back(crowd_sim_interface);
    sdf_floor = sdf_floor->GetNextElement("floor");
  }

}

//=================================================
void CrowdSimulatorPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo& info,
  ignition::gazebo::EntityComponentManager& ecm)
{
  // wait for all the models and actors loaded in ignition rendering
  if (!_initialized)
  {
    for (auto& crowd_sim_interface:_crowd_sim_interfaces)
      _init_spawned_agents(ecm, crowd_sim_interface);
    return;
  }

  std::chrono::duration<double> delta_sim_time_tmp = info.simTime -
    _last_sim_time;
  double delta_sim_time = delta_sim_time_tmp.count();
  for (auto& crowd_sim_interface:_crowd_sim_interfaces)
  {
    if (crowd_sim_interface->get_sim_time_step() <= delta_sim_time)
    {
      _last_sim_time = info.simTime;
      if (crowd_sim_interface->_menge_disabled)
      {
        _animate_idle_objects(delta_sim_time, ecm, crowd_sim_interface);
      }
      else
      {
        crowd_sim_interface->one_step_sim();
        _update_all_objects(delta_sim_time, ecm, crowd_sim_interface);
      }
    }
  }
}

//==========================================================
void CrowdSimulatorPlugin::_init_spawned_agents(
  ignition::gazebo::EntityComponentManager& ecm,
  std::shared_ptr<crowd_simulator::CrowdSimInterface> crowd_sim_interface)
{
  // check all the models are in the world
  std::unordered_map<std::string, size_t> objects_name;
  size_t object_count = crowd_sim_interface->get_num_objects();
  if (crowd_sim_interface->_menge_disabled)
  {
    for (size_t id = 0; id < object_count; id++)
    {
      objects_name.insert({crowd_sim_interface->get_internal_agent(id), id});
    }
    // for internal agent
    ecm.Each<ignition::gazebo::components::Actor,
      ignition::gazebo::components::Name>(
      [&](const ignition::gazebo::Entity& entity,
      const ignition::gazebo::components::Actor* actor,
      const ignition::gazebo::components::Name* name) -> bool
      {
        auto it_objects_name = objects_name.find(name->Data());
        if (it_objects_name != objects_name.end())
        {
          // update in entityDic
          _entity_dic[name->Data()] = entity;
          // config internal spawned agent for custom trajectory
          _config_spawned_agents(nullptr, entity, ecm, crowd_sim_interface);
          objects_name.erase(name->Data());
        }
        return true;
      });
    if (objects_name.size() != 0)
    {
      _initialized = false;
      return;
    }
    _initialized = true;
  }
  else
  {
    for (size_t id = 0; id < object_count; id++)
    {
      auto obj = crowd_sim_interface->get_object_by_id(id);
      // already found in the Dic
      if (_entity_dic.find(obj->model_name) != _entity_dic.end())
        continue;
      objects_name.insert({obj->model_name, id});
    }
    // for external agent
    ecm.Each<ignition::gazebo::components::Model,
      ignition::gazebo::components::Name>(
      [&](const ignition::gazebo::Entity& entity,
      const ignition::gazebo::components::Model*,
      const ignition::gazebo::components::Name* name) -> bool
      {
        auto it_objects_name = objects_name.find(name->Data());
        if (it_objects_name != objects_name.end())
        {
          // update in entityDic
          _entity_dic[name->Data()] = entity;
          auto obj_ptr =
          crowd_sim_interface->get_object_by_id(it_objects_name->second);
          // config internal spawned agent for custom trajectory
          if (!obj_ptr->is_external)
          {
            _config_spawned_agents(obj_ptr, entity, ecm, crowd_sim_interface);
          }
          objects_name.erase(name->Data());
          RCLCPP_INFO(crowd_sim_interface->logger(),
          "Crowd Simulator found agent: " + name->Data() );
        }
        return true;
      }
      );
    // for internal agent
    ecm.Each<ignition::gazebo::components::Actor,
      ignition::gazebo::components::Name>(
      [&](const ignition::gazebo::Entity& entity,
      const ignition::gazebo::components::Actor*,
      const ignition::gazebo::components::Name* name) -> bool
      {
        auto it_objects_name = objects_name.find(name->Data());
        if (it_objects_name != objects_name.end())
        {
          // update in entityDic
          _entity_dic[name->Data()] = entity;
          auto obj_ptr =
          crowd_sim_interface->get_object_by_id(it_objects_name->second);
          // config internal spawned agent for custom trajectory
          if (!obj_ptr->is_external)
          {
            _config_spawned_agents(obj_ptr, entity, ecm, crowd_sim_interface);
          }
          objects_name.erase(name->Data());
          RCLCPP_INFO(crowd_sim_interface->logger(),
          "Crowd Simulator found agent: " + name->Data() );
        }
        return true;
      }
      );

    // external agents not found or not loaded yet
    if (objects_name.size() != 0)
    {
      _initialized = false;
      return;
    }
    _initialized = true;
    RCLCPP_INFO(
      crowd_sim_interface->logger(),
      "Ignition Models are all loaded! Start simulating...");
  }
}

//==================================================
void CrowdSimulatorPlugin::_config_spawned_agents(
  const crowd_simulator::CrowdSimInterface::ObjectPtr obj_ptr,
  const ignition::gazebo::Entity& entity,
  ignition::gazebo::EntityComponentManager& ecm,
  std::shared_ptr<crowd_simulator::CrowdSimInterface> crowd_sim_interface) const
{
  bool disabled = crowd_sim_interface->_menge_disabled;
  assert(disabled || obj_ptr);
  auto agent_ptr = (disabled) ? nullptr : obj_ptr->agent_ptr;
  auto model_type =
    (disabled) ? nullptr : crowd_sim_interface->_model_type_db_ptr->get(
    obj_ptr->type_name);

  // get pose component for entity
  auto pose_comp = ecm.Component<ignition::gazebo::components::Pose>(entity);
  auto original_pose = ignition::math::Pose3d(
    pose_comp->Data().Pos().X(), pose_comp->Data().Pos().Y(),
    pose_comp->Data().Pos().Z(), 0, 0, 0);
  if (!disabled && nullptr == pose_comp) // model_type nullptr for disabled
  {
    // use the initial_pose for actor type
    ignition::math::Pose3d initial_pose =
      model_type->pose.convert_to_ign_math_pose_3d<ignition::math::Pose3d>();
    ecm.CreateComponent(entity,
      ignition::gazebo::components::Pose(initial_pose));
  }
  else
  {
    // original pose in the world
    *pose_comp =
      ignition::gazebo::components::Pose(ignition::math::Pose3d(0, 0,
        pose_comp->Data().Pos().Z(), 0, 0,
        0));
  }

  // different from gazebo plugin, the pose component is the origin of the trajPose
  ignition::math::Pose3d actor_pose;
  if (!disabled)
  {
    actor_pose = ignition::math::Pose3d(
      static_cast<double>(agent_ptr->_pos.x()),
      static_cast<double>(agent_ptr->_pos.y()), 0.0,
      0, 0, 0
    );
  }
  else
  {
    actor_pose = original_pose;
  }

  if (!disabled)
  {
    // check idle animation name
    auto actor_comp =
      ecm.Component<ignition::gazebo::components::Actor>(entity);
    for (auto idle_anim : crowd_sim_interface->get_switch_anim_name())
    {
      if (actor_comp->Data().AnimationNameExists(idle_anim))
      {
        model_type->idle_animation = idle_anim;
        break;
      }
    }
  }


  // initialize agent animationName
  std::string animation_name =
    (disabled) ? "idle" : model_type->animation;
  assert(!animation_name.empty());

  auto animation_name_comp =
    ecm.Component<ignition::gazebo::components::AnimationName>(entity);
  if (nullptr == animation_name_comp)
  {
    ecm.CreateComponent(entity,
      ignition::gazebo::components::AnimationName(animation_name));
  }
  else
  {
    *animation_name_comp = ignition::gazebo::components::AnimationName(
      animation_name);
  }

  // mark as one-time-change
  ecm.SetChanged(
    entity,
    ignition::gazebo::components::AnimationName::typeId,
    ignition::gazebo::ComponentState::OneTimeChange);

  // initialize agent animationTime
  auto anim_time_comp =
    ecm.Component<ignition::gazebo::components::AnimationTime>(entity);
  if (nullptr == anim_time_comp)
  {
    ecm.CreateComponent(entity,
      ignition::gazebo::components::AnimationTime());
  }

  // having a trajectory pose prevents the actor from moving with the sdf script
  auto traj_pose_comp =
    ecm.Component<ignition::gazebo::components::TrajectoryPose>(entity);
  if (nullptr == traj_pose_comp)
  {
    ecm.CreateComponent(entity,
      ignition::gazebo::components::TrajectoryPose(actor_pose));
  }

}

void CrowdSimulatorPlugin::_animate_idle_objects(double delta_sim_time,
  ignition::gazebo::EntityComponentManager&  ecm,
  std::shared_ptr<crowd_simulator::CrowdSimInterface> crowd_sim_interface) const
{
  // for internal agent
  ecm.Each<ignition::gazebo::components::Actor,
    ignition::gazebo::components::Name>(
    [&](const ignition::gazebo::Entity& entity,
    const ignition::gazebo::components::Actor* actor,
    const ignition::gazebo::components::Name* name) -> bool
    {
      //const auto& local_entity = _entity_dic.find(name->Data());
      //if (local_entity != _entity_dic.end())
      //{

      auto anim_time_comp =
      ecm.Component<ignition::gazebo::components::AnimationTime>(entity);
      if (nullptr == anim_time_comp)
      {
        RCLCPP_ERROR(
          crowd_sim_interface->logger(),
          "[" + name->Data() +
          "] has no AnimationTime component.");
        exit(EXIT_FAILURE);
      }
      anim_time_comp->Data() +=
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(delta_sim_time));
      ecm.SetChanged(entity,
      ignition::gazebo::components::AnimationTime::typeId,
      ignition::gazebo::ComponentState::OneTimeChange);
      //}
      return true;
    });
}

//============================================================================
void CrowdSimulatorPlugin::_update_all_objects(
  double delta_sim_time,
  ignition::gazebo::EntityComponentManager& ecm,
  std::shared_ptr<crowd_simulator::CrowdSimInterface> crowd_sim_interface) const
{
  auto objects_count = crowd_sim_interface->get_num_objects();
  for (size_t id = 0; id < objects_count; id++)
  {
    auto obj_ptr = crowd_sim_interface->get_object_by_id(id);
    auto it_entity = _entity_dic.find(obj_ptr->model_name);
    if (it_entity == _entity_dic.end())   //safe check
    {
      RCLCPP_ERROR(crowd_sim_interface->logger(),
        "Didn't initialize external agent [" + obj_ptr->model_name + "]");
      exit(EXIT_FAILURE);
    }
    auto entity = it_entity->second;

    // for external agent
    if (obj_ptr->is_external)
    {
      auto model_pose =
        ecm.Component<ignition::gazebo::components::Pose>(entity)->Data();
      crowd_sim_interface->update_external_agent(obj_ptr->agent_ptr,
        model_pose);
      continue;
    }

    // for internal agent
    _update_internal_object(delta_sim_time, obj_ptr, entity, ecm,
      crowd_sim_interface);
  }
}

void CrowdSimulatorPlugin::_update_internal_object(
  double delta_sim_time,
  const crowd_simulator::CrowdSimInterface::ObjectPtr obj_ptr,
  const ignition::gazebo::Entity& entity,
  ignition::gazebo::EntityComponentManager& ecm,
  std::shared_ptr<crowd_simulator::CrowdSimInterface> crowd_sim_interface) const
{
  double animation_speed = crowd_sim_interface->_model_type_db_ptr->get(
    obj_ptr->type_name)->animation_speed;
  ignition::math::Pose3d initial_pose =
    crowd_sim_interface->_model_type_db_ptr->get(obj_ptr->type_name)->pose.
    convert_to_ign_math_pose_3d<ignition::math::Pose3d>();
  ignition::math::Pose3d agent_pose =
    crowd_sim_interface->get_agent_pose<ignition::math::Pose3d>(
    obj_ptr->agent_ptr, delta_sim_time);
  agent_pose += initial_pose;

  // get components to be updated
  auto traj_pose_comp =
    ecm.Component<ignition::gazebo::components::TrajectoryPose>(entity);
  if (nullptr == traj_pose_comp)
  {
    RCLCPP_ERROR(crowd_sim_interface->logger(),
      "Model [" + obj_ptr->model_name + "] has no TrajectoryPose component.");
    exit(EXIT_FAILURE);
  }
  auto anim_name_comp =
    ecm.Component<ignition::gazebo::components::AnimationName>(entity);
  if (nullptr == anim_name_comp)
  {
    RCLCPP_ERROR(crowd_sim_interface->logger(),
      "Model [" + obj_ptr->model_name + "] has no AnimationName component.");
    exit(EXIT_FAILURE);
  }
  auto anim_time_comp =
    ecm.Component<ignition::gazebo::components::AnimationTime>(entity);
  if (nullptr == anim_name_comp)
  {
    RCLCPP_ERROR(crowd_sim_interface->logger(),
      "Model [" + obj_ptr->model_name + "] has no AnimationTime component.");
    exit(EXIT_FAILURE);
  }
  auto actor_comp =
    ecm.Component<ignition::gazebo::components::Actor>(entity);

  ignition::math::Pose3d current_pose = traj_pose_comp->Data();
  auto distance_traveled_vector = agent_pose.Pos() - current_pose.Pos();
  // might need future work on 3D case
  // the center of human has a z_elevation, which will make the human keep walking even if he reached the target
  distance_traveled_vector.Z(0.0);
  double distance_traveled = distance_traveled_vector.Length();

  // switch animation
  auto model_type = crowd_sim_interface->_model_type_db_ptr->get(
    obj_ptr->type_name);
  AnimState next_state = obj_ptr->get_next_state(
    distance_traveled < crowd_sim_interface->get_switch_anim_distance_th() &&
    !model_type->idle_animation.empty());

  switch (next_state)
  {
    case AnimState::WALK:
      anim_time_comp->Data() +=
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(distance_traveled / animation_speed));
      if (obj_ptr->current_state != next_state)
        anim_name_comp->Data() = model_type->animation;
      break;

    case AnimState::IDLE:
      anim_time_comp->Data() +=
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(delta_sim_time));
      agent_pose.Rot() = current_pose.Rot();
      if (obj_ptr->current_state != next_state)
        anim_name_comp->Data() = model_type->idle_animation;
      break;
  }

  if (obj_ptr->current_state != next_state)
    ecm.SetChanged(entity,
      ignition::gazebo::components::AnimationName::typeId,
      ignition::gazebo::ComponentState::OneTimeChange);
  obj_ptr->current_state = next_state;

  // set trajectory
  traj_pose_comp->Data() = agent_pose;
  ecm.SetChanged(entity,
    ignition::gazebo::components::TrajectoryPose::typeId,
    ignition::gazebo::ComponentState::OneTimeChange);
  ecm.SetChanged(entity,
    ignition::gazebo::components::AnimationTime::typeId,
    ignition::gazebo::ComponentState::OneTimeChange);
}

} //namespace crowd_simulation_ign

IGNITION_ADD_PLUGIN(
  crowd_simulation_ign::CrowdSimulatorPlugin,
  ignition::gazebo::System,
  crowd_simulation_ign::CrowdSimulatorPlugin::ISystemConfigure,
  crowd_simulation_ign::CrowdSimulatorPlugin::ISystemPreUpdate)

// TODO would prefer namespace
IGNITION_ADD_PLUGIN_ALIAS(crowd_simulation_ign::CrowdSimulatorPlugin,
  "crowd_simulation")