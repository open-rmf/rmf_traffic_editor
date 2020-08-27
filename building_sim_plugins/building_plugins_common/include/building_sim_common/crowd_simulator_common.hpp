#ifndef BUILDING_SIM_COMMON__CROWD_SIMULATOR_COMMON_HPP
#define BUILDING_SIM_COMMON__CROWD_SIMULATOR_COMMON_HPP

#include <functional>
#include <list>
#include <queue>
#include <memory>

#include <MengeCore/Runtime/SimulatorDB.h>
#include <MengeCore/Orca/ORCADBEntry.h>
#include <MengeCore/Orca/ORCASimulator.h>
#include <MengeCore/PluginEngine/CorePluginEngine.h>

namespace crowd_simulator {

using AgentPtr = std::shared_ptr<Menge::Agents::BaseAgent>;
//===============================================================
/*
* class AgentPose3d
* replace the common interface of ignition::math::Pose3d, which might apply with different version in different plugins
*/
class  AgentPose3d{

public:
  AgentPose3d();
  AgentPose3d(double& x, double& y, double& z, double& pitch, double& roll, double& yaw);

  ~AgentPose3d();

  double X() const;
  double Y() const;
  double Z() const;
  double Pitch() const;
  double Roll() const;
  double Yaw() const;

  double& X();
  double& Y();
  double& Z();
  double& Pitch();
  double& Roll();
  double& Yaw();

  void X(const double& x);
  void Y(const double& y);
  void Z(const double& z);
  void Pitch(const double& pitch);
  void Roll(const double& roll);
  void Yaw(const double& yaw);

private:
  double _x;
  double _y;
  double _z;
  double _pitch;
  double _roll;
  double _yaw;

};

//================================================================
/*
* class MengeHandle
*/
class MengeHandle
{
public:

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

    _behaviorFile = this->_ResourceFilePath(_behaviorFile);
    _sceneFile = this->_ResourceFilePath(_sceneFile);

    if (this->_LoadSimulation())
    {
      this->initialized = true;
    }
    assert(this->initialized);
  }

  bool initialized = false;

  void SetSimTimeStep(float simTimeStep);
  float GetSimTimeStep();
  size_t GetAgentCount();
  void SimStep(); //proceed one-time simulation step in _sim

  AgentPtr GetAgent(size_t id);

private:
  std::string _resourcePath;
  std::string _behaviorFile;
  std::string _sceneFile;
  float _simTimeStep;
  size_t _agentCount;
  
  //Have some problem when transfer raw pointer to shared_ptr or unique_ptr
  std::shared_ptr<Menge::Agents::SimulatorInterface> _sim;

  std::string _ResourceFilePath(const std::string& relativePath) const;
  bool _LoadSimulation(); //initialize simulatorinterface

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
    
    //for ignition
    std::string modelFilePath;
  };

  using RecordPtr = std::shared_ptr<Record>;

  ModelTypeDatabase() {}

  //Create a new record and returns a reference to the record
  template<typename... Args>
  RecordPtr Emplace(Args&& ... args){
    auto pair = this->_records.emplace(std::forward<Args>(args)...); //return pair<iterator, bool>
    assert(pair.second);
    return pair.first->second;
  }

  //Get the total number of actors
  size_t Size();

  RecordPtr Get(const std::string& typeName);

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

  CrowdSimInterface(const std::string& resourcePath,
    const std::string& behaviorFile,
    const std::string& sceneFile,
    float simTimeStep = 0.0
  )
  {
    this->_mengeHandle = std::make_shared<MengeHandle>(resourcePath,
        behaviorFile, sceneFile,
        simTimeStep);
  }

  bool SpawnObject(std::vector<std::string>& externalModels);
  void AddObject(AgentPtr agentPtr, const std::string& modelName,
    const std::string& typeName, bool isExternal);

  size_t GetNumObjects();
  ObjectPtr GetObjectById(size_t id);

  void OneStepSim();

  void UpdateExternalAgent(size_t id, const AgentPose3d& modelPose);
  void UpdateExternalAgent(const AgentPtr agentPtr, const AgentPose3d& modelPose);
  void GetAgentPose(size_t id, double deltaSimTime, AgentPose3d& modelPose);
  void GetAgentPose(const AgentPtr agentPtr, double deltaSimTime, AgentPose3d& modelPose);

private:
  std::vector<ObjectPtr> _objects; //Database, use id to access ObjectPtr
  std::shared_ptr<MengeHandle> _mengeHandle;
  float _simTimeStep;
};

} //namespace crowd_simulator


#endif //CROWD_SIMULATION_COMMON__CROWD_SIMULATOR_COMMON_HPP