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
  AgentPose3d(double x, double y, double z, double roll, double pitch, double yaw)
    : _x(x), _y(y), _z(z), _roll(roll), _pitch(pitch), _yaw(yaw)
  {}

  double X() const {return _x;}
  double Y() const {return _y;}
  double Z() const {return _z;}
  double Roll() const {return _roll;}
  double Pitch() const {return _pitch;}
  double Yaw() const {return _yaw;}

  void X(double x) {_x = x;}
  void Y(double y) {_y = y;}
  void Z(double z) {_z = z;}
  void Roll(double roll) {_roll = roll;}
  void Pitch(double pitch) {_pitch = pitch;}
  void Yaw(double yaw) {_yaw = yaw;}

  template<typename IgnMathPose3d>
  IgnMathPose3d ConvertToIgnMathPose3d() {
    return IgnMathPose3d(_x, _y, _z, _roll, _pitch, _yaw);
  }

private:
  double _x, _y, _z, _roll, _pitch, _yaw;
};
//================================================================
/*
* class MengeHandle
*/
class MengeHandle : public std::enable_shared_from_this<MengeHandle>
{
public:

  static std::shared_ptr<MengeHandle> init_and_make(
    const std::string& resourcePath,
    const std::string& behaviorFile,
    const std::string& sceneFile,
    const float simTimeStep
  );

  MengeHandle(const std::string& resourcePath,
    const std::string& behaviorFile,
    const std::string& sceneFile,
    const float simTimeStep = 0.0
  )
  : _resourcePath(resourcePath),
    _behaviorFile(behaviorFile),
    _sceneFile(sceneFile),
    _simTimeStep(simTimeStep),
    _agentCount(0)
  {
    _behaviorFile = this->_resource_file_path(_behaviorFile);
    _sceneFile = this->_resource_file_path(_sceneFile);
  }

  void set_sim_time_step(float simTimeStep);
  float get_sim_time_step() const;
  size_t get_agent_count();
  void sim_step() const; //proceed one-time simulation step in _sim
  AgentPtr get_agent(size_t id) const;

private:
  std::string _resourcePath;
  std::string _behaviorFile;
  std::string _sceneFile;
  float _simTimeStep;
  size_t _agentCount;
  std::shared_ptr<Menge::Agents::SimulatorInterface> _sim;

  std::string _resource_file_path(const std::string& relativePath) const;
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
    std::string typeName;
    std::string fileName;
    AgentPose3d pose;
    std::string animation;
    double animationSpeed;
  };

  using RecordPtr = std::shared_ptr<Record>;

  //Create a new record and returns a reference to the record
  RecordPtr emplace(std::string typeName, RecordPtr record_ptr);
  size_t size() const;
  RecordPtr get(const std::string& typeName) const;

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
  struct Object
  {
    AgentPtr agentPtr;
    std::string modelName;
    std::string typeName;
    bool isExternal = false;
  };
  using ObjectPtr = std::shared_ptr<Object>;

  CrowdSimInterface()
    : _modelTypeDBPtr(std::make_shared<crowd_simulator::ModelTypeDatabase>()),
    _initialized(false),
    _sdf_loaded(false)
  {}

  std::shared_ptr<ModelTypeDatabase> _modelTypeDBPtr;
  rclcpp::Logger logger() const;
  void init_ros_node(const rclcpp::Node::SharedPtr node);

  template<typename SdfPtrT>
  bool read_sdf(SdfPtrT& sdf);

  bool init_crowd_sim();
  bool is_initialized() const;

  double get_sim_time_step() const;

  size_t get_num_objects() const;
  ObjectPtr get_object_by_id(size_t id) const;

  void one_step_sim() const;

  template<typename IgnMathPose3d>
  void update_external_agent(size_t id, const IgnMathPose3d& modelPose);

  template<typename IgnMathPose3d>
  void update_external_agent(const AgentPtr agentPtr, const IgnMathPose3d& modelPose);

  template<typename IgnMathPose3d>
  IgnMathPose3d get_agent_pose(size_t id, double deltaSimTime);

  template<typename IgnMathPose3d>
  IgnMathPose3d get_agent_pose(const AgentPtr agentPtr, double deltaSimTime);

private:
  bool _initialized;
  bool _sdf_loaded;
  std::vector<ObjectPtr> _objects; //Database, use id to access ObjectPtr
  std::shared_ptr<MengeHandle> _mengeHandle;
  float _simTimeStep;
  std::string _resourcePath;
  std::string _behaviorFile;
  std::string _sceneFile;
  std::vector<std::string> _externalAgents;
  rclcpp::Node::SharedPtr _ros_node;

  template<typename SdfPtrT>
  bool _load_model_init_pose(SdfPtrT& modelTypeElement, AgentPose3d& result) const;

  bool _spawn_object();
  void _add_object(AgentPtr agentPtr, const std::string& modelName,
    const std::string& typeName, bool isExternal);
};

template<typename SdfPtrT>
bool CrowdSimInterface::read_sdf(SdfPtrT& sdf) 
{
  if (!sdf->template HasElement("resource_path"))
  {
    char* menge_resource_path;
    menge_resource_path = getenv("MENGE_RESOURCE_PATH");
    RCLCPP_WARN(logger(), 
      "No resource path provided! <env MENGE_RESOURCE_PATH> " + std::string(menge_resource_path) + " will be used." ); 
    _resourcePath = std::string(menge_resource_path);
  } else{
    _resourcePath = sdf->template GetElementImpl("resource_path")->template Get<std::string>();
  }
  
  if (!sdf->template HasElement("behavior_file"))
  {
    RCLCPP_ERROR(logger(), 
      "No behavior file found! <behavior_file> Required!" ); 
    return false;
  }
  _behaviorFile = sdf->template GetElementImpl("behavior_file")->template Get<std::string>();

  if (!sdf->template HasElement("scene_file"))
  {
    RCLCPP_ERROR(logger(), 
      "No scene file found! <scene_file> Required!" );
    return false;
  }
  _sceneFile = sdf->template GetElementImpl("scene_file")->template Get<std::string>();

  if (!sdf->template HasElement("update_time_step"))
  {
    RCLCPP_ERROR(logger(), 
      "No update_time_step found! <update_time_step> Required!");
    return false;
  }
  _simTimeStep = sdf->template GetElementImpl("update_time_step")->template Get<float>();

  if (!sdf->template HasElement("model_type"))
  {
    RCLCPP_ERROR(logger(), 
      "No model type for agents found! <model_type> element Required!");
    return false;
  }
  auto modelTypeElement = sdf->template GetElementImpl("model_type");
  while (modelTypeElement)
  {
    std::string s;
    if (!modelTypeElement->template Get<std::string>("typename", s, ""))
    {
      RCLCPP_ERROR(logger(), 
        "No model type name configured in <model_type>! <typename> Required");
      return false;
    }

    auto modelTypePtr = this->_modelTypeDBPtr->emplace(s, std::make_shared<ModelTypeDatabase::Record>() ); //unordered_map
    modelTypePtr->typeName = s;

    if (!modelTypeElement->template Get<std::string>("filename", modelTypePtr->fileName,""))
    {
      RCLCPP_ERROR(logger(), 
        "No actor skin configured in <model_type>! <filename> Required");
      return false;
    }

    if (!modelTypeElement->template Get<std::string>("animation", modelTypePtr->animation, ""))
    {
      RCLCPP_ERROR(logger(), 
        "No animation configured in <model_type>! <animation> Required");
      return false;
    }

    if (!modelTypeElement->template Get<double>("animation_speed", modelTypePtr->animationSpeed, 0.0))
    {
      RCLCPP_ERROR(logger(), 
        "No animation speed configured in <model_type>! <animation_speed> Required");
      return false;
    }

    if (!modelTypeElement->template HasElement("initial_pose"))
    {
      RCLCPP_ERROR(logger(), 
        "No model initial pose configured in <model_type>! <initial_pose> Required [" + s + "]");
      return false;
    }
    if (!_loadModelInitPose(modelTypeElement, modelTypePtr->pose))
    {
      RCLCPP_ERROR(logger(), 
        "Error loading model initial pose in <model_type>! Check <initial_pose> in [" + s + "]");
      return false;
    }

    modelTypeElement = modelTypeElement->template GetNextElement("model_type");
  }

  if (!sdf->template HasElement("external_agent"))
  {
    RCLCPP_ERROR(logger(), 
      "No external agent provided. <external_agent> is needed with a unique name defined above.");
  }
  auto externalAgentElement = sdf->template GetElementImpl("external_agent");
  while (externalAgentElement)
  {
    auto exAgentName = externalAgentElement->template Get<std::string>();
    RCLCPP_INFO(logger(), 
      "Added external agent: [ " + exAgentName + " ].");
    _externalAgents.emplace_back(exAgentName); //just store the name
    externalAgentElement = externalAgentElement->template GetNextElement("external_agent");
  }

  _sdf_loaded = true;
  return true;
}

template<typename SdfPtrT>
bool CrowdSimInterface::_load_model_init_pose(SdfPtrT& modelTypeElement, AgentPose3d& result) const
{
  std::string poseStr;
  if (modelTypeElement->template Get<std::string>("initial_pose", poseStr, ""))
  {
    std::regex ws_re("\\s+"); //whitespace
    std::vector<std::string> parts(
      std::sregex_token_iterator(poseStr.begin(), poseStr.end(), ws_re, -1),
      std::sregex_token_iterator());

    if (parts.size() != 6)
    {
      RCLCPP_ERROR(logger(), 
        "Error loading <initial_pose> in <model_type>, 6 floats (x, y, z, pitch, roll, yaw) expected.");
      return false;
    }

    result.X( std::stod(parts[0]) );
    result.Y( std::stod(parts[1]) );
    result.Z( std::stod(parts[2]) );
    result.Pitch( std::stod(parts[3]) );
    result.Roll( std::stod(parts[4]) );
    result.Yaw( std::stod(parts[5]) );
  }
  return true;
}

template<typename IgnMathPose3d>
IgnMathPose3d CrowdSimInterface::get_agent_pose(size_t id, double deltaSimTime) {
  assert(id < get_num_objects());
  auto agentPtr = _objects[id]->agentPtr;
  return get_agent_pose<IgnMathPose3d>(agentPtr, deltaSimTime);
}

template<typename IgnMathPose3d>
IgnMathPose3d CrowdSimInterface::get_agent_pose(const AgentPtr agentPtr, double deltaSimTime) {
  //calculate future position in deltaSimTime. currently in 2d
  assert(agentPtr);
  double Px = static_cast<double>(agentPtr->_pos.x()) +
    static_cast<double>(agentPtr->_vel.x()) * deltaSimTime;
  double Py = static_cast<double>(agentPtr->_pos.y()) +
    static_cast<double>(agentPtr->_vel.y()) * deltaSimTime;

  double xRot = static_cast<double>(agentPtr->_orient.x());
  double yRot = static_cast<double>(agentPtr->_orient.y());

  IgnMathPose3d agent_pose(Px, Py, 0, 0, 0, std::atan2(yRot, xRot));
  return agent_pose;
}

template<typename IgnMathPose3d>
void CrowdSimInterface::update_external_agent(size_t id, const IgnMathPose3d& modelPose) {
  assert(id < get_num_objects());
  auto agentPtr = _objects[id]->agentPtr;
  update_external_agent<IgnMathPose3d>(agentPtr, modelPose);
}

template<typename IgnMathPose3d>
void CrowdSimInterface::update_external_agent(const AgentPtr agentPtr, const IgnMathPose3d& modelPose) {
  assert(agentPtr);
  agentPtr->_pos.setX(modelPose.Pos().X());
  agentPtr->_pos.setY(modelPose.Pos().Y());
}

} //namespace crowd_simulator


#endif //CROWD_SIMULATION_COMMON__CROWD_SIMULATOR_COMMON_HPP