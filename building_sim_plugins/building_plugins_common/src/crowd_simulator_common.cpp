#include <fstream>
#include <cmath>

#include <rclcpp/logger.hpp>

#include <building_sim_common/crowd_simulator_common.hpp>

namespace crowd_simulator {

//================================================================
void MengeHandle::SetSimTimeStep(float simTimeStep)
{
  this->_simTimeStep = simTimeStep;
}

float MengeHandle::GetSimTimeStep()
{
  return this->_simTimeStep;
}

size_t MengeHandle::GetAgentCount()
{
  if (this->_agentCount == 0)
  {
    this->_agentCount = this->_sim->getNumAgents();
  }
  return this->_agentCount;
}

void MengeHandle::SimStep()
{
  this->_sim->step();
}

AgentPtr MengeHandle::GetAgent(size_t id)
{
  return AgentPtr(this->_sim->getAgent(id));
}

std::string MengeHandle::_ResourceFilePath(const std::string& relativePath)
const
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

bool MengeHandle::_LoadSimulation()
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

ModelTypeDatabase::RecordPtr ModelTypeDatabase::Get(const std::string& typeName)
{
  auto it = this->_records.find(typeName);
  if (it == this->_records.end())
  {
    std::cout << "The model type [ " << typeName << " ] is not defined in scene file!" << std::endl;
    return nullptr;
  }
  return it;
}

size_t ModelTypeDatabase::Size()
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

bool CrowdSimInterface::initCrowdSim() 
{
  _mengeHandle = std::make_shared<MengeHandle>(
    _resourcePath,
    _behaviorFile, 
    _sceneFile,
    _simTimeStep);
  
  if (!_mengeHandle->initialized) return false;

  if (!_sdf_loaded) {
    RCLCPP_ERROR(logger(), "Please load the sdf before initialize the crowd_sim interface!");
    return false;
  }
  _spawnObject();
  _initialized = true;
  return true;
}

bool CrowdSimInterface::isInitialized() const 
{
  return _initialized;
}

double CrowdSimInterface::getSimTimeStep() const {
  return static_cast<double>(_simTimeStep);
}

bool CrowdSimInterface::_spawnObject()
{
  //External models are loaded first in scene file
  size_t externalCount = _externalAgents.size();
  size_t totalAgentCount = _mengeHandle->GetAgentCount();

  //external model must be included in scene file
  assert(externalCount <= totalAgentCount);

  for (size_t i = 0; i < externalCount; ++i)
  {
    auto agentPtr = _mengeHandle->GetAgent(i);
    agentPtr->_external = true;
    _addObject(agentPtr, _externalAgents[i], "0", true);
  }

  for (size_t i = externalCount; i < totalAgentCount; ++i)
  {
    auto agentPtr = this->_mengeHandle->GetAgent(i);
    agentPtr->_external = false;
    std::string modelName = "agent" + std::to_string(i);
    _addObject(agentPtr, modelName, agentPtr->_typeName, false);
  }
  return true;
}

void CrowdSimInterface::_addObject(AgentPtr agentPtr,
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

size_t CrowdSimInterface::getNumObjects() const
{
  return _objects.size();
}

CrowdSimInterface::ObjectPtr CrowdSimInterface::getObjectById(size_t id) const
{
  assert(id < _objects.size());
  return _objects[id];
}

void CrowdSimInterface::oneStepSim() const
{
  _mengeHandle->SimStep();
}

} //namespace crowd_simulator
