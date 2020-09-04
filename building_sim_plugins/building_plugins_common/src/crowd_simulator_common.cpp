#include <fstream>
#include <cmath>

#include <rclcpp/logger.hpp>

#include <building_sim_common/crowd_simulator_common.hpp>

namespace crowd_simulator {

//================================================================
std::shared_ptr<MengeHandle> MengeHandle::init_and_make(
  const std::string& resourcePath,
  const std::string& behaviorFile,
  const std::string& sceneFile,
  const float simTimeStep
) {
  auto menge_handle = std::make_shared<MengeHandle>(resourcePath, behaviorFile, sceneFile, simTimeStep);
  if (!menge_handle->_load_simulation()) {
    return nullptr;
  }
  return menge_handle;
}

void MengeHandle::set_sim_time_step(float simTimeStep)
{
  this->_simTimeStep = simTimeStep;
}

float MengeHandle::get_sim_time_step() const
{
  return this->_simTimeStep;
}

size_t MengeHandle::get_agent_count()
{
  if (this->_agentCount == 0)
  {
    this->_agentCount = this->_sim->getNumAgents();
  }
  return this->_agentCount;
}

void MengeHandle::sim_step() const
{
  this->_sim->step();
}

AgentPtr MengeHandle::get_agent(size_t id) const
{
  return AgentPtr(this->_sim->getAgent(id));
}

std::string MengeHandle::_resource_file_path(const std::string& relativePath) const
{
  std::string fullPath = this->_resourcePath + "/" + relativePath;
  std::cout << "Finding resource file: " << fullPath << std::endl;
  std::ifstream ifile(fullPath);
  if (!static_cast<bool>(ifile))
  {
    std::cerr << "File not found! " << fullPath << std::endl;
    assert(static_cast<bool>(ifile));
  }
  std::cout << "Found." << std::endl;
  return fullPath;
}

bool MengeHandle::_load_simulation()
{
  Menge::SimulatorDB simDB;
  Menge::PluginEngine::CorePluginEngine engine(&simDB);

  std::cout << "Start CrowdSimulator initializing [Menge]..." << std::endl;

  this->_sim = std::shared_ptr<Menge::Agents::SimulatorInterface> (
    simDB.getDBEntry("orca")->getSimulator(
      this->_agentCount,
      this->_simTimeStep,
      0,
      std::numeric_limits<float>::infinity(),
      this->_behaviorFile,
      this->_sceneFile,
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
  std::cout << "Error in provided navmesh. Menge simulator initialized false." << std::endl;
  return false;
}

//============================================
ModelTypeDatabase::RecordPtr ModelTypeDatabase::emplace(std::string typeName, RecordPtr record_ptr){
  auto pair = this->_records.emplace(typeName, record_ptr); //return pair<iterator, bool>
  assert(pair.second);
  return pair.first->second;
}

ModelTypeDatabase::RecordPtr ModelTypeDatabase::get(const std::string& typeName) const
{
  auto it = this->_records.find(typeName);
  if (it == this->_records.end())
  {
    std::cout << "The model type [ " << typeName << " ] is not defined in scene file!" << std::endl;
    return nullptr;
  }
  return it->second;
}

size_t ModelTypeDatabase::size() const
{
  return this->_records.size();
}

//================================================================

rclcpp::Logger CrowdSimInterface::logger() const {
  return rclcpp::get_logger("crowdsim");
}

void CrowdSimInterface::init_ros_node(const rclcpp::Node::SharedPtr node)
{
  _ros_node = std::move(node);
}

bool CrowdSimInterface::init_crowd_sim() 
{
  _mengeHandle = MengeHandle::init_and_make(
    _resourcePath,
    _behaviorFile, 
    _sceneFile,
    _simTimeStep);

  if (!_sdf_loaded) {
    RCLCPP_ERROR(logger(), "Please load the sdf before initialize the crowd_sim interface!");
    return false;
  }
  _spawn_object();
  _initialized = true;
  return true;
}

bool CrowdSimInterface::is_initialized() const 
{
  return _initialized;
}

double CrowdSimInterface::get_sim_time_step() const {
  return static_cast<double>(_simTimeStep);
}

bool CrowdSimInterface::_spawn_object()
{
  //External models are loaded first in scene file
  size_t externalCount = _externalAgents.size();
  size_t totalAgentCount = _mengeHandle->get_agent_count();

  //external model must be included in scene file
  assert(externalCount <= totalAgentCount);

  for (size_t i = 0; i < externalCount; ++i)
  {
    auto agentPtr = _mengeHandle->get_agent(i);
    agentPtr->_external = true;
    _add_object(agentPtr, _externalAgents[i], "0", true);
  }

  for (size_t i = externalCount; i < totalAgentCount; ++i)
  {
    auto agentPtr = this->_mengeHandle->get_agent(i);
    agentPtr->_external = false;
    std::string modelName = "agent" + std::to_string(i);
    _add_object(agentPtr, modelName, agentPtr->_typeName, false);
  }
  return true;
}

void CrowdSimInterface::_add_object(AgentPtr agentPtr,
  const std::string& modelName,
  const std::string& typeName,
  bool isExternal = false)
{
  assert(agentPtr);
  // must provide a model name in gazebo if it's an external agent
  if (isExternal)
  {
    assert(!modelName.empty());
  }
  _objects.emplace_back(new Object{agentPtr, modelName, typeName, isExternal});
}

size_t CrowdSimInterface::get_num_objects() const
{
  return _objects.size();
}

CrowdSimInterface::ObjectPtr CrowdSimInterface::get_object_by_id(size_t id) const
{
  assert(id < _objects.size());
  return _objects[id];
}

void CrowdSimInterface::one_step_sim() const
{
  _mengeHandle->sim_step();
}

} //namespace crowd_simulator
