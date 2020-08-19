#include <fstream>
#include <cmath>

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

ModelTypeDatabase::Record* ModelTypeDatabase::Get(const std::string& typeName)
{
  if (this->_records.find(typeName) == this->_records.end())
  {
    std::cout << "The model type [ " << typeName << " ] is not defined in scene file!" << std::endl;
    return nullptr;
  }
  return this->_records[typeName];
}

size_t ModelTypeDatabase::Size()
{
  return this->_records.size();
}

//================================================================
bool CrowdSimInterface::SpawnObject(std::vector<std::string>& externalModels)
{
  //External models are loaded first in scene file
  size_t externalCount = externalModels.size();
  size_t totalAgentCount = this->_mengeHandle->GetAgentCount();

  //external model must be included in scene file
  assert(externalCount <= totalAgentCount);

  for (size_t i = 0; i < externalCount; ++i)
  {
    auto agentPtr = this->_mengeHandle->GetAgent(i);
    agentPtr->_external = true;
    this->AddObject(agentPtr, externalModels[i], "0", true);
  }

  for (size_t i = externalCount; i < totalAgentCount; ++i)
  {
    auto agentPtr = this->_mengeHandle->GetAgent(i);
    agentPtr->_external = false;

    std::string modelName = "agent" + std::to_string(i);

    this->AddObject(agentPtr, modelName, agentPtr->_typeName, false);
  }

  return true;
}

void CrowdSimInterface::AddObject(AgentPtr agentPtr,
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

  this->_objects.emplace_back(new Object{agentPtr, modelName, typeName,
      isExternal});
}


size_t CrowdSimInterface::GetNumObjects()
{
  return this->_objects.size();
}

CrowdSimInterface::ObjectPtr CrowdSimInterface::GetObjectById(size_t id)
{
  assert(id < this->_objects.size());
  return this->_objects[id];
}


void CrowdSimInterface::OneStepSim()
{
  this->_mengeHandle->SimStep();
}


void CrowdSimInterface::UpdateExternalAgent(size_t id, const AgentPose3d& modelPose){

  assert(id < this->GetNumObjects());

  auto agentPtr = this->_objects[id]->agentPtr;
  this->UpdateExternalAgent(agentPtr, modelPose);
}


void CrowdSimInterface::UpdateExternalAgent(const AgentPtr agentPtr, const AgentPose3d& modelPose){

  assert(agentPtr);

  agentPtr->_pos.setX(modelPose.X());
  agentPtr->_pos.setY(modelPose.Y());
}


void CrowdSimInterface::GetAgentPose(size_t id, double deltaSimTime, AgentPose3d& modelPose){

  assert(id < this->GetNumObjects());

  auto agentPtr = this->_objects[id]->agentPtr;
  this->GetAgentPose(agentPtr, deltaSimTime, modelPose);
}


void CrowdSimInterface::GetAgentPose(const AgentPtr agentPtr, double deltaSimTime, AgentPose3d& modelPose){
  //calculate future position in deltaSimTime.
  assert(agentPtr);
  double Px = static_cast<double>(agentPtr->_pos.x()) +
    static_cast<double>(agentPtr->_vel.x()) * deltaSimTime;
  double Py = static_cast<double>(agentPtr->_pos.y()) +
    static_cast<double>(agentPtr->_vel.y()) * deltaSimTime;

  modelPose.X(Px);
  modelPose.Y(Py);
  modelPose.Z() = 0.0;

  double xRot = static_cast<double>(agentPtr->_orient.x());
  double yRot = static_cast<double>(agentPtr->_orient.y());

  modelPose.Pitch(0);
  modelPose.Roll(0);
  modelPose.Yaw(std::atan2(yRot, xRot));
}

//=============================================================
AgentPose3d::AgentPose3d(){
  this->_x = 0.0;
  this->_y = 0.0;
  this->_z = 0.0;
  this->_pitch = 0.0;
  this->_roll = 0.0;
  this->_yaw = 0.0;
}

AgentPose3d::~AgentPose3d(){
  //do nothing
}

AgentPose3d::AgentPose3d(double& x, double& y, double& z, double& pitch, double& roll, double& yaw){
  this->_x = x;
  this->_y = y;
  this->_z = z;
  this->_pitch = pitch;
  this->_roll = roll;
  this->_yaw = yaw;
}

double AgentPose3d::X() const {
  return this->_x;
}

double AgentPose3d::Y() const {
  return this->_y;
}

double AgentPose3d::Z() const {
  return this->_z;
}

double AgentPose3d::Pitch() const {
  return this->_pitch;
}

double AgentPose3d::Roll() const {
  return this->_roll;
}

double AgentPose3d::Yaw() const {
  return this->_yaw;
}

double& AgentPose3d::X(){
  return this->_x;
}

double& AgentPose3d::Y(){
  return this->_y;
}

double& AgentPose3d::Z(){
  return this->_z;
}

double& AgentPose3d::Pitch(){
  return this->_pitch;
}

double& AgentPose3d::Roll(){
  return this->_roll;
}

double& AgentPose3d::Yaw(){
  return this->_yaw;
}

void AgentPose3d::X(const double& x){
  this->_x = x;
}

void AgentPose3d::Y(const double& y){
  this->_y = y;
}
void AgentPose3d::Z(const double& z){
  this->_z = z;
}

void AgentPose3d::Pitch(const double& pitch){
  this->_pitch = pitch;
}

void AgentPose3d::Roll(const double& roll){
  this->_roll = roll;
}

void AgentPose3d::Yaw(const double& yaw){
  this->_yaw = yaw;
}

} //namespace crowd_simulator
