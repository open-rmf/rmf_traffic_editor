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

#include <fstream>
#include <cmath>

#include <rclcpp/logger.hpp>

#include <building_sim_common/crowd_simulator_common.hpp>

namespace crowd_simulator {

//================================================================
std::shared_ptr<MengeHandle> MengeHandle::init_and_make(
  const std::string& resource_path,
  const std::string& behavior_file,
  const std::string& scene_file,
  const float sim_time_step
)
{
  auto menge_handle = std::make_shared<MengeHandle>(
    resource_path, behavior_file, scene_file, sim_time_step);
  if (!menge_handle->_load_simulation())
  {
    return nullptr;
  }
  return menge_handle;
}

void MengeHandle::set_sim_time_step(float sim_time_step)
{
  this->_sim_time_step = sim_time_step;
}

float MengeHandle::get_sim_time_step() const
{
  return this->_sim_time_step;
}

size_t MengeHandle::get_agent_count()
{
  if (this->_agent_count == 0)
  {
    this->_agent_count = this->_sim->getNumAgents();
  }
  return this->_agent_count;
}

void MengeHandle::sim_step() const
{
  this->_sim->step();
}

AgentPtr MengeHandle::get_agent(size_t id) const
{
  return AgentPtr(this->_sim->getAgent(id));
}

std::string MengeHandle::_resource_file_path(const std::string& relative_path)
const
{
  std::string full_path = this->_resource_path + "/" + relative_path;
  std::cout << "Finding resource file: " << full_path << std::endl;
  std::ifstream ifile(full_path);
  if (!static_cast<bool>(ifile))
  {
    std::cerr << "File not found! " << full_path << std::endl;
    assert(static_cast<bool>(ifile));
  }
  std::cout << "Found." << std::endl;
  return full_path;
}

bool MengeHandle::_load_simulation()
{
  Menge::SimulatorDB sim_db;
  Menge::PluginEngine::CorePluginEngine engine(&sim_db);

  std::cout << "Start CrowdSimulator initializing [Menge]..." << std::endl;

  this->_sim = std::shared_ptr<Menge::Agents::SimulatorInterface>(
    sim_db.getDBEntry("orca")->getSimulator(
      this->_agent_count,
      this->_sim_time_step,
      0,
      std::numeric_limits<float>::infinity(),
      this->_behavior_file,
      this->_scene_file,
      "",
      "",
      false)
  );

  if (this->_sim)
  {
    std::cout << std::endl << "Crowd Simulator initialized success [Menge]. " <<
      std::endl;
    return true;
  }
  std::cout <<
    "Error in provided navmesh. Menge simulator initialized false." <<
    std::endl;
  return false;
}

//============================================
ModelTypeDatabase::RecordPtr ModelTypeDatabase::emplace(
  std::string type_name,
  RecordPtr record_ptr)
{
  auto pair = this->_records.emplace(type_name, record_ptr); //return pair<iterator, bool>
  assert(pair.second);
  return pair.first->second;
}

ModelTypeDatabase::RecordPtr ModelTypeDatabase::get(
  const std::string& type_name)
const
{
  auto it = this->_records.find(type_name);
  if (it == this->_records.end())
  {
    std::cout << "The model type [ " << type_name <<
      " ] is not defined in scene file!" << std::endl;
    return nullptr;
  }
  return it->second;
}

size_t ModelTypeDatabase::size() const
{
  return this->_records.size();
}

//================================================================

rclcpp::Logger CrowdSimInterface::logger() const
{
  return rclcpp::get_logger("crowdsim");
}

void CrowdSimInterface::init_ros_node(const rclcpp::Node::SharedPtr node)
{
  _ros_node = std::move(node);
}

bool CrowdSimInterface::init_crowd_sim()
{
  _menge_handle = MengeHandle::init_and_make(
    _resource_path,
    _behavior_file,
    _scene_file,
    _sim_time_step);

  if (!_sdf_loaded)
  {
    RCLCPP_ERROR(
      logger(),
      "Please load the sdf before initialize the crowd_sim interface!");
    return false;
  }
  _spawn_object();
  return true;
}

double CrowdSimInterface::get_sim_time_step() const
{
  return static_cast<double>(_sim_time_step);
}

bool CrowdSimInterface::_spawn_object()
{
  //External models are loaded first in scene file
  size_t external_count = _external_agents.size();
  size_t total_agent_count = _menge_handle->get_agent_count();

  //external model must be included in scene file
  assert(external_count <= total_agent_count);

  for (size_t i = 0; i < external_count; ++i)
  {
    auto agent_ptr = _menge_handle->get_agent(i);
    agent_ptr->_external = true;
    _add_object(agent_ptr, _external_agents[i], "0", true);
  }

  for (size_t i = external_count; i < total_agent_count; ++i)
  {
    auto agent_ptr = this->_menge_handle->get_agent(i);
    agent_ptr->_external = false;
    std::string model_name = "agent" + std::to_string(i);
    _add_object(agent_ptr, model_name, agent_ptr->_typeName, false);
  }
  return true;
}

void CrowdSimInterface::_add_object(AgentPtr agent_ptr,
  const std::string& model_name,
  const std::string& type_name,
  bool is_external = false)
{
  assert(agent_ptr);
  // must provide a model name in gazebo if it's an external agent
  if (is_external)
  {
    assert(!model_name.empty());
  }
  _objects.emplace_back(
    new Object{agent_ptr, model_name, type_name, is_external,
      AnimState::WALK});
}

size_t CrowdSimInterface::get_num_objects() const
{
  return _objects.size();
}

CrowdSimInterface::ObjectPtr CrowdSimInterface::get_object_by_id(size_t id)
const
{
  assert(id < _objects.size());
  return _objects[id];
}

void CrowdSimInterface::one_step_sim() const
{
  _menge_handle->sim_step();
}

double CrowdSimInterface::get_switch_anim_distance_th() const
{
  return _switch_anim_distance_th;
}

std::vector<std::string> CrowdSimInterface::get_switch_anim_name() const
{
  return _switch_anim_name;
}

//=============================================
CrowdSimInterface::AnimState CrowdSimInterface::Object::get_next_state(
  bool condition)
{
  if (condition)
    return AnimState::IDLE;
  else
    return AnimState::WALK;

  return current_state;
}

} //namespace crowd_simulator
