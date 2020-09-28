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

#ifndef BUILDING_SIM_COMMON__CROWD_SIMULATOR_COMMON_HPP
#define BUILDING_SIM_COMMON__CROWD_SIMULATOR_COMMON_HPP

#include <functional>
#include <list>
#include <queue>
#include <memory>
#include <regex> //for parsing initial_pose

#include <MengeCore/Runtime/SimulatorDB.h>
#include <MengeCore/Orca/ORCADBEntry.h>
#include <MengeCore/Orca/ORCASimulator.h>
#include <MengeCore/PluginEngine/CorePluginEngine.h>

#include <rclcpp/rclcpp.hpp>

namespace crowd_simulator {

using AgentPtr = std::shared_ptr<Menge::Agents::BaseAgent>;

class AgentPose3d
{
public:
  AgentPose3d()
  : _x(0), _y(0), _z(0), _roll(0), _pitch(0), _yaw(0)
  {}
  AgentPose3d(double x, double y, double z, double roll, double pitch,
    double yaw)
  : _x(x), _y(y), _z(z), _roll(roll), _pitch(pitch), _yaw(yaw)
  {}

  double x() const {return _x;}
  double y() const {return _y;}
  double z() const {return _z;}
  double roll() const {return _roll;}
  double pitch() const {return _pitch;}
  double yaw() const {return _yaw;}

  void x(double x) {_x = x;}
  void y(double y) {_y = y;}
  void z(double z) {_z = z;}
  void roll(double roll) {_roll = roll;}
  void pitch(double pitch) {_pitch = pitch;}
  void yaw(double yaw) {_yaw = yaw;}

  template<typename IgnMathPose3d>
  IgnMathPose3d convert_to_ign_math_pose_3d()
  {
    return IgnMathPose3d(_x, _y, _z, _roll, _pitch, _yaw);
  }

private:
  double _x, _y, _z, _roll, _pitch, _yaw;
};

//================================================================
/*
* class MengeHandle, provides a wrap-up class handle for actual menge lib
* only exposing several menge function interface
*/
class MengeHandle : public std::enable_shared_from_this<MengeHandle>
{
public:

  static std::shared_ptr<MengeHandle> init_and_make(
    const std::string& resource_path,
    const std::string& behavior_file,
    const std::string& scene_file,
    const float sim_time_step
  );

  MengeHandle(const std::string& resource_path,
    const std::string& behavior_file,
    const std::string& scene_file,
    const float sim_time_step = 0.0
  )
  : _resource_path(resource_path),
    _behavior_file(behavior_file),
    _scene_file(scene_file),
    _sim_time_step(sim_time_step),
    _agent_count(0)
  {
    _behavior_file = this->_resource_file_path(_behavior_file);
    _scene_file = this->_resource_file_path(_scene_file);
  }

  void set_sim_time_step(float sim_time_step);
  float get_sim_time_step() const;
  size_t get_agent_count();
  void sim_step() const; //proceed one-time simulation step in _sim
  AgentPtr get_agent(size_t id) const;

private:
  std::string _resource_path;
  std::string _behavior_file;
  std::string _scene_file;
  float _sim_time_step;
  size_t _agent_count;
  std::shared_ptr<Menge::Agents::SimulatorInterface> _sim;

  std::string _resource_file_path(const std::string& relative_path) const;
  bool _load_simulation(); //initialize simulatorinterface
};

//================================================================
/*
* class ModelTypeDatabase
*/
class ModelTypeDatabase
{
public:
  struct Record
  {
    std::string type_name;
    std::string file_name;
    AgentPose3d pose;
    std::string animation;
    std::string idle_animation;
    double animation_speed;
  };

  using RecordPtr = std::shared_ptr<Record>;

  //Create a new record and returns a reference to the record
  RecordPtr emplace(std::string type_name, RecordPtr record_ptr);
  size_t size() const;
  RecordPtr get(const std::string& type_name) const;

private:
  std::unordered_map<std::string, RecordPtr> _records;
};

//================================================================
/*
* class CrowdSimInterface
* provides the relationship between menge agents and gazebo models
* provides the interface to set position between gazebo models and menge agents
*/
class CrowdSimInterface
{
public:
  enum class AnimState
  {
    WALK,
    IDLE,
  };

  struct Object
  {
    AgentPtr agent_ptr;
    std::string model_name;
    std::string type_name;
    bool is_external = false;
    AnimState current_state;
    AnimState get_next_state(bool condition);
  };
  using ObjectPtr = std::shared_ptr<Object>;

  CrowdSimInterface()
  : _model_type_db_ptr(std::make_shared<crowd_simulator::ModelTypeDatabase>()),
    _sdf_loaded(false),
    _switch_anim_distance_th(0.01),
    _switch_anim_name({"idle", "stand"})
  {}

  std::shared_ptr<ModelTypeDatabase> _model_type_db_ptr;
  rclcpp::Logger logger() const;
  void init_ros_node(const rclcpp::Node::SharedPtr node);

  bool init_crowd_sim();
  double get_sim_time_step() const;
  size_t get_num_objects() const;
  ObjectPtr get_object_by_id(size_t id) const;
  void one_step_sim() const;
  double get_switch_anim_distance_th() const;
  std::vector<std::string> get_switch_anim_name() const;

  template<typename SdfPtrT>
  bool read_sdf(SdfPtrT& sdf);

  template<typename IgnMathPose3d>
  void update_external_agent(
    size_t id, const IgnMathPose3d& model_pose);

  template<typename IgnMathPose3d>
  void update_external_agent(
    const AgentPtr agent_ptr, const IgnMathPose3d& model_pose);

  template<typename IgnMathPose3d>
  IgnMathPose3d get_agent_pose(
    size_t id, double delta_sim_time);

  template<typename IgnMathPose3d>
  IgnMathPose3d get_agent_pose(
    const AgentPtr agent_ptr, double delta_sim_time);

private:
  bool _sdf_loaded;
  double _switch_anim_distance_th;
  std::vector<std::string> _switch_anim_name;
  std::vector<ObjectPtr> _objects; //Database, use id to access ObjectPtr
  std::shared_ptr<MengeHandle> _menge_handle;
  float _sim_time_step;
  std::string _resource_path;
  std::string _behavior_file;
  std::string _scene_file;
  std::vector<std::string> _external_agents;
  rclcpp::Node::SharedPtr _ros_node;

  template<typename SdfPtrT>
  bool _load_model_init_pose(
    SdfPtrT& model_type_element, AgentPose3d& result) const;

  bool _spawn_object();
  void _add_object(
    AgentPtr agent_ptr, const std::string& model_name,
    const std::string& type_name, bool is_external);
};

template<typename SdfPtrT>
bool CrowdSimInterface::read_sdf(
  SdfPtrT& sdf)
{
  if (!sdf->template HasElement("resource_path"))
  {
    char* menge_resource_path;
    menge_resource_path = getenv("MENGE_RESOURCE_PATH");
    RCLCPP_WARN(logger(),
      "No resource path provided! <env MENGE_RESOURCE_PATH> " +
      std::string(menge_resource_path) + " will be used.");
    _resource_path = std::string(menge_resource_path);
  }
  else
  {
    _resource_path =
      sdf->template GetElementImpl("resource_path")->template Get<std::string>();
  }

  if (!sdf->template HasElement("behavior_file"))
  {
    RCLCPP_ERROR(logger(),
      "No behavior file found! <behavior_file> Required!");
    return false;
  }
  _behavior_file =
    sdf->template GetElementImpl("behavior_file")->template Get<std::string>();

  if (!sdf->template HasElement("scene_file"))
  {
    RCLCPP_ERROR(logger(),
      "No scene file found! <scene_file> Required!");
    return false;
  }
  _scene_file =
    sdf->template GetElementImpl("scene_file")->template Get<std::string>();

  if (!sdf->template HasElement("update_time_step"))
  {
    RCLCPP_ERROR(logger(),
      "No update_time_step found! <update_time_step> Required!");
    return false;
  }
  _sim_time_step =
    sdf->template GetElementImpl("update_time_step")->template Get<float>();

  if (!sdf->template HasElement("model_type"))
  {
    RCLCPP_ERROR(logger(),
      "No model type for agents found! <model_type> element Required!");
    return false;
  }
  auto model_type_element = sdf->template GetElementImpl("model_type");
  while (model_type_element)
  {
    std::string s;
    if (!model_type_element->template Get<std::string>("typename", s, ""))
    {
      RCLCPP_ERROR(logger(),
        "No model type name configured in <model_type>! <typename> Required");
      return false;
    }

    auto model_type_ptr = this->_model_type_db_ptr->emplace(s,
        std::make_shared<ModelTypeDatabase::Record>() ); //unordered_map
    model_type_ptr->type_name = s;

    if (!model_type_element->template Get<std::string>("filename",
      model_type_ptr->file_name, ""))
    {
      RCLCPP_ERROR(logger(),
        "No actor skin configured in <model_type>! <filename> Required");
      return false;
    }

    if (!model_type_element->template Get<std::string>("animation",
      model_type_ptr->animation, ""))
    {
      RCLCPP_ERROR(logger(),
        "No animation configured in <model_type>! <animation> Required");
      return false;
    }

    if (!model_type_element->template Get<double>("animation_speed",
      model_type_ptr->animation_speed, 0.0))
    {
      RCLCPP_ERROR(
        logger(),
        "No animation speed configured in <model_type>! <animation_speed> Required");
      return false;
    }

    if (!model_type_element->template HasElement("initial_pose"))
    {
      RCLCPP_ERROR(
        logger(),
        "No model initial pose configured in <model_type>! <initial_pose> Required [" + s +
        "]");
      return false;
    }
    if (!_load_model_init_pose(model_type_element, model_type_ptr->pose))
    {
      RCLCPP_ERROR(
        logger(),
        "Error loading model initial pose in <model_type>! Check <initial_pose> in [" + s +
        "]");
      return false;
    }

    model_type_element = model_type_element->template GetNextElement(
      "model_type");
  }

  if (!sdf->template HasElement("external_agent"))
  {
    RCLCPP_ERROR(
      logger(),
      "No external agent provided. <external_agent> is needed with a unique name defined above.");
  }
  auto external_agent_element = sdf->template GetElementImpl("external_agent");
  while (external_agent_element)
  {
    auto ex_agent_name = external_agent_element->template Get<std::string>();
    RCLCPP_INFO(logger(),
      "Added external agent: [ " + ex_agent_name + " ].");
    _external_agents.emplace_back(ex_agent_name); //just store the name
    external_agent_element = external_agent_element->template GetNextElement(
      "external_agent");
  }

  _sdf_loaded = true;
  return true;
}

template<typename SdfPtrT>
bool CrowdSimInterface::_load_model_init_pose(
  SdfPtrT& model_type_element, AgentPose3d& result) const
{
  std::string pose_str;
  if (model_type_element->template Get<std::string>(
      "initial_pose", pose_str, ""))
  {
    std::regex ws_re("\\s+"); //whitespace
    std::vector<std::string> parts(
      std::sregex_token_iterator(pose_str.begin(), pose_str.end(), ws_re, -1),
      std::sregex_token_iterator());

    if (parts.size() != 6)
    {
      RCLCPP_ERROR(
        logger(),
        "Error loading <initial_pose> in <model_type>, 6 floats (x, y, z, pitch, roll, yaw) expected.");
      return false;
    }

    result.x(std::stod(parts[0]) );
    result.y(std::stod(parts[1]) );
    result.z(std::stod(parts[2]) );
    result.pitch(std::stod(parts[3]) );
    result.roll(std::stod(parts[4]) );
    result.yaw(std::stod(parts[5]) );
  }
  return true;
}

template<typename IgnMathPose3d>
IgnMathPose3d CrowdSimInterface::get_agent_pose(
  size_t id, double delta_sim_time)
{
  assert(id < get_num_objects());
  auto agent_ptr = _objects[id]->agent_ptr;
  return get_agent_pose<IgnMathPose3d>(agent_ptr, delta_sim_time);
}

template<typename IgnMathPose3d>
IgnMathPose3d CrowdSimInterface::get_agent_pose(
  const AgentPtr agent_ptr, double delta_sim_time)
{
  //calculate future position in delta_sim_time. currently in 2d
  assert(agent_ptr);
  double px = static_cast<double>(agent_ptr->_pos.x()) +
    static_cast<double>(agent_ptr->_vel.x()) * delta_sim_time;
  double py = static_cast<double>(agent_ptr->_pos.y()) +
    static_cast<double>(agent_ptr->_vel.y()) * delta_sim_time;

  double x_rot = static_cast<double>(agent_ptr->_orient.x());
  double y_rot = static_cast<double>(agent_ptr->_orient.y());

  IgnMathPose3d agent_pose(px, py, 0, 0, 0, std::atan2(y_rot, x_rot));
  return agent_pose;
}

template<typename IgnMathPose3d>
void CrowdSimInterface::update_external_agent(
  size_t id, const IgnMathPose3d& model_pose)
{
  assert(id < get_num_objects());
  auto agent_ptr = _objects[id]->agent_ptr;
  update_external_agent<IgnMathPose3d>(agent_ptr, model_pose);
}

template<typename IgnMathPose3d>
void CrowdSimInterface::update_external_agent(
  const AgentPtr agent_ptr, const IgnMathPose3d& model_pose)
{
  assert(agent_ptr);
  agent_ptr->_pos.setX(model_pose.Pos().X());
  agent_ptr->_pos.setY(model_pose.Pos().Y());
}

} // namespace crowd_simulator

#endif // CROWD_SIMULATION_COMMON__CROWD_SIMULATOR_COMMON_HPP