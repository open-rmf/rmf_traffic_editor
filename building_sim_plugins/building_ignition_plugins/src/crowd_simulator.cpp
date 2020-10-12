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
  RCLCPP_INFO(_crowd_sim_interface->logger(),
    "Initializing world plugin with name: " + _world->Name(ecm));
  _world_name = _world->Name(ecm);

  if (!_crowd_sim_interface->read_sdf(sdf))
  {
    RCLCPP_ERROR(_crowd_sim_interface->logger(),
      "Error loading crowd simulator plugin. Load params failed!");
    exit(EXIT_FAILURE);
  }

  if (!_crowd_sim_interface->init_crowd_sim())
  {
    RCLCPP_ERROR(_crowd_sim_interface->logger(),
      "Error loading crowd simulator plugin. Load [ Menge ] failed!");
    exit(EXIT_FAILURE);
  }

  if (!_spawn_agents_in_world(ecm))
  {
    RCLCPP_ERROR(
      _crowd_sim_interface->logger(),
      "Error loading crowd simulator plugin. Crowd Simulator failed to spawn agents in the world!");
    exit(EXIT_FAILURE);
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
    _init_spawned_agents(ecm);
    return;
  }

  std::chrono::duration<double> delta_sim_time_tmp = info.simTime -
    _last_sim_time;
  double delta_sim_time = delta_sim_time_tmp.count();
  if (_crowd_sim_interface->get_sim_time_step() <= delta_sim_time)
  {
    _last_sim_time = info.simTime;
    _crowd_sim_interface->one_step_sim();
    _update_all_objects(delta_sim_time, ecm);
  }
}

//==========================================================
bool CrowdSimulatorPlugin::_spawn_agents_in_world(
  ignition::gazebo::EntityComponentManager& ecm)
{
  size_t object_count = this->_crowd_sim_interface->get_num_objects();
  for (size_t id = 0; id < object_count; ++id)
  {
    auto object_ptr = this->_crowd_sim_interface->get_object_by_id(id);
    assert(object_ptr);
    _object_dic[object_ptr->model_name] = id;

    if (!object_ptr->is_external)
    {
      auto type_ptr = _crowd_sim_interface->_model_type_db_ptr->get(
        object_ptr->type_name);
      assert(type_ptr);
      if (!this->_create_entity(ecm, object_ptr->model_name, type_ptr) )
      {
        RCLCPP_ERROR(_crowd_sim_interface->logger(),
          "Failed to insert model [ " + object_ptr->model_name + " ] in world");
        return false;
      }
    }
  }
  return true;
}

//==========================================================
void CrowdSimulatorPlugin::_init_spawned_agents(
  ignition::gazebo::EntityComponentManager& ecm)
{
  // check all the models are in the world
  std::unordered_map<std::string, size_t> objects_name;
  size_t object_count = _crowd_sim_interface->get_num_objects();
  for (size_t id = 0; id < object_count; id++)
  {
    auto obj = _crowd_sim_interface->get_object_by_id(id);
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
        _crowd_sim_interface->get_object_by_id(it_objects_name->second);
        // config internal spawned agent for custom trajectory
        if (!obj_ptr->is_external)
        {
          _config_spawned_agents(obj_ptr, entity, ecm);
        }
        objects_name.erase(name->Data());
        RCLCPP_INFO(_crowd_sim_interface->logger(),
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
        _crowd_sim_interface->get_object_by_id(it_objects_name->second);
        // config internal spawned agent for custom trajectory
        if (!obj_ptr->is_external)
        {
          _config_spawned_agents(obj_ptr, entity, ecm);
        }
        objects_name.erase(name->Data());
        RCLCPP_INFO(_crowd_sim_interface->logger(),
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
    _crowd_sim_interface->logger(),
    "Ignition Models are all loaded! Start simulating...");
}

//===================================================================
bool CrowdSimulatorPlugin::_create_entity(
  ignition::gazebo::EntityComponentManager& ecm,
  const std::string& model_name,
  const crowd_simulator::ModelTypeDatabase::RecordPtr model_type_ptr) const
{
  // Use ignition create service to spawn actors
  // calling ignition gazebo create service, you can use "ign service -l" to check the service available
  assert(model_type_ptr);
  std::string service = "/world/" + this->_world_name + "/create";
  ignition::msgs::EntityFactory request;
  request.set_sdf_filename(model_type_ptr->file_name);
  request.set_name(model_name);
  ignition::math::Pose3d pose(0, 0, 0, 0, 0, 0);

  ignition::msgs::Boolean response;
  bool result;
  uint32_t timeout = 5000;
  bool executed = this->_transport_node_ptr->Request(service, request, timeout,
      response, result);
  if (executed)
  {
    if (result && response.data())
    {
      RCLCPP_INFO(_crowd_sim_interface->logger(),
        "Requested creation of entity: " + model_name);
      return true;
    }
    else
    {
      RCLCPP_ERROR(_crowd_sim_interface->logger(),
        "Failed request to create entity.\n" + request.DebugString());
    }
  }
  else
  {
    RCLCPP_ERROR(
      _crowd_sim_interface->logger(),
      "Request to create entity from service " + service + "timer out ...\n" +
      request.DebugString());
  }
  return false;
}

//==================================================
void CrowdSimulatorPlugin::_config_spawned_agents(
  const crowd_simulator::CrowdSimInterface::ObjectPtr obj_ptr,
  const ignition::gazebo::Entity& entity,
  ignition::gazebo::EntityComponentManager& ecm) const
{
  assert(obj_ptr);
  auto agent_ptr = obj_ptr->agent_ptr;
  auto model_type = _crowd_sim_interface->_model_type_db_ptr->get(
    obj_ptr->type_name);
  // different from gazebo plugin, the pose component is the origin of the trajPose
  ignition::math::Pose3d actor_pose(
    static_cast<double>(agent_ptr->_pos.x()),
    static_cast<double>(agent_ptr->_pos.y()), 0.0,
    0, 0, 0
  );

  // get pose component for entity
  auto pose_comp = ecm.Component<ignition::gazebo::components::Pose>(entity);
  if (nullptr == pose_comp)
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
      ignition::gazebo::components::Pose(ignition::math::Pose3d(0, 0, 0, 0, 0,
        0));
  }

  // initialize agent animationName
  std::string animation_name = model_type->animation;
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
  // check idle animation name
  auto actor_comp =
    ecm.Component<ignition::gazebo::components::Actor>(entity);
  for (auto idle_anim : _crowd_sim_interface->get_switch_anim_name())
  {
    if (actor_comp->Data().AnimationNameExists(idle_anim))
    {
      model_type->idle_animation = idle_anim;
      break;
    }
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
    ecm.CreateComponent(entity, ignition::gazebo::components::AnimationTime());
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

//============================================================================
void CrowdSimulatorPlugin::_update_all_objects(
  double delta_sim_time,
  ignition::gazebo::EntityComponentManager& ecm) const
{
  auto objects_count = _crowd_sim_interface->get_num_objects();
  for (size_t id = 0; id < objects_count; id++)
  {
    auto obj_ptr = _crowd_sim_interface->get_object_by_id(id);
    auto it_entity = _entity_dic.find(obj_ptr->model_name);
    if (it_entity == _entity_dic.end())   //safe check
    {
      RCLCPP_ERROR(_crowd_sim_interface->logger(),
        "Didn't initialize external agent [" + obj_ptr->model_name + "]");
      exit(EXIT_FAILURE);
    }
    auto entity = it_entity->second;

    // for external agent
    if (obj_ptr->is_external)
    {
      auto model_pose =
        ecm.Component<ignition::gazebo::components::Pose>(entity)->Data();
      _crowd_sim_interface->update_external_agent(obj_ptr->agent_ptr,
        model_pose);
      continue;
    }

    // for internal agent
    _update_internal_object(delta_sim_time, obj_ptr, entity, ecm);
  }
}

void CrowdSimulatorPlugin::_update_internal_object(
  double delta_sim_time,
  const crowd_simulator::CrowdSimInterface::ObjectPtr obj_ptr,
  const ignition::gazebo::Entity& entity,
  ignition::gazebo::EntityComponentManager& ecm) const
{
  double animation_speed = _crowd_sim_interface->_model_type_db_ptr->get(
    obj_ptr->type_name)->animation_speed;
  ignition::math::Pose3d initial_pose =
    _crowd_sim_interface->_model_type_db_ptr->get(obj_ptr->type_name)->pose.
    convert_to_ign_math_pose_3d<ignition::math::Pose3d>();
  ignition::math::Pose3d agent_pose =
    _crowd_sim_interface->get_agent_pose<ignition::math::Pose3d>(
    obj_ptr->agent_ptr, delta_sim_time);
  agent_pose += initial_pose;

  // get components to be updated
  auto traj_pose_comp =
    ecm.Component<ignition::gazebo::components::TrajectoryPose>(entity);
  if (nullptr == traj_pose_comp)
  {
    RCLCPP_ERROR(_crowd_sim_interface->logger(),
      "Model [" + obj_ptr->model_name + "] has no TrajectoryPose component.");
    exit(EXIT_FAILURE);
  }
  auto anim_name_comp =
    ecm.Component<ignition::gazebo::components::AnimationName>(entity);
  if (nullptr == anim_name_comp)
  {
    RCLCPP_ERROR(_crowd_sim_interface->logger(),
      "Model [" + obj_ptr->model_name + "] has no AnimationName component.");
    exit(EXIT_FAILURE);
  }
  auto anim_time_comp =
    ecm.Component<ignition::gazebo::components::AnimationTime>(entity);
  if (nullptr == anim_name_comp)
  {
    RCLCPP_ERROR(_crowd_sim_interface->logger(),
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
  auto model_type = _crowd_sim_interface->_model_type_db_ptr->get(
    obj_ptr->type_name);
  AnimState next_state = obj_ptr->get_next_state(
    distance_traveled < _crowd_sim_interface->get_switch_anim_distance_th() &&
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