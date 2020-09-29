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

#ifndef BUILDING_SIM_COMMON__CROWD_SIMULATOR_GAZEBO_HPP
#define BUILDING_SIM_COMMON__CROWD_SIMULATOR_GAZEBO_HPP

#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <ignition/math/Pose3.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Mesh.hh>
#include <gazebo/common/Skeleton.hh>
#include <gazebo/common/SkeletonAnimation.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/physics/physics.hh>

#include <building_sim_common/crowd_simulator_common.hpp>


namespace crowd_simulation_gazebo {

using ObjectPtr = crowd_simulator::CrowdSimInterface::ObjectPtr;
//================================================================
/*
* class CrowdSimulatorPlugin
*/

class CrowdSimulatorPlugin : public gazebo::WorldPlugin
{
  using AnimState = crowd_simulator::CrowdSimInterface::AnimState;
public:
  CrowdSimulatorPlugin()
  : _crowd_sim_interface(std::make_shared<crowd_simulator::CrowdSimInterface>()),
    _initialized(false),
    _objects_count(0)
  {}

  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

private:
  std::shared_ptr<crowd_simulator::CrowdSimInterface> _crowd_sim_interface;
  bool _initialized;
  size_t _objects_count;
  gazebo::physics::WorldPtr _world;
  gazebo::event::ConnectionPtr _update_connection_ptr;
  gazebo::common::Time _last_sim_time;

  bool _spawn_agents_in_world();
  void _init_spawned_agents();
  void _update(const gazebo::common::UpdateInfo& update_info); //Update trigger function
  void _update_all_objects(double delta_sim_time);
  void _update_internal_object(
    double delta_sim_time,
    const ObjectPtr object_ptr,
    const gazebo::physics::ModelPtr model_ptr,
    const crowd_simulator::ModelTypeDatabase::RecordPtr type_ptr);
  bool _create_model(
    const std::string& model_name,
    const crowd_simulator::ModelTypeDatabase::RecordPtr model_type_ptr,
    const crowd_simulator::AgentPtr agent_ptr);
};

} //namespace crowd_simulation_gazebo

#endif // CROWD_SIMULATION_GAZEBO__CROWD_SIMULATOR_GAZEBO_HPP