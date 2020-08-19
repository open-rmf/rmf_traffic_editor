#ifndef BUILDING_SIM_COMMON__CROWD_SIMULATOR_GAZEBO_HPP
#define BUILDING_SIM_COMMON__CROWD_SIMULATOR_GAZEBO_HPP

#include <memory>
#include <unordered_map>

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

template<typename... Args>
using Task = std::function<bool(Args... args)>;

template<typename TaskType>
class TaskManager
{
public:
  using TaskPtr = std::shared_ptr<TaskType>;

  void AddTask(const TaskType& task);

  template<typename... TaskArgs>
  void RunAllTasks(TaskArgs... args);

  size_t GetTasksCount();

private:
  std::list<TaskPtr> _tasks;
};

template<typename TaskType>
void TaskManager<TaskType>::AddTask(const TaskType& task)
{
  this->_tasks.emplace_back(std::make_shared<TaskType>(task));
}

template<typename TaskType>
template<typename... TaskArgs>
void TaskManager<TaskType>::RunAllTasks(TaskArgs... args)
{
  std::list<TaskPtr> done;
  for (const auto& task : this->_tasks)
  {
    if ((*task)(args...))
    {
      done.emplace_back(task);
    }
  }

  for (const auto& task : done)
  {
    this->_tasks.remove(task);
  }
}

template<typename TaskType>
size_t TaskManager<TaskType>::GetTasksCount()
{
  return this->_tasks.size();
}

//===============================================================
crowd_simulator::AgentPose3d Convert(const ignition::math::Pose3d& ignition_pose);

ignition::math::Pose3d Convert(const crowd_simulator::AgentPose3d& agent_pose);

//================================================================
/*
* class CrowdSimulatorPlugin
*/

class CrowdSimulatorPlugin : public gazebo::WorldPlugin
{

public:
  CrowdSimulatorPlugin();
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

private:
  gazebo::physics::WorldPtr _world;
  gazebo::event::ConnectionPtr _updateConnectionPtr;
  gazebo::common::Time _lastAnimTime;
  gazebo::common::Time _lastSimTime;
  size_t _objectsCount;

  mutable bool _initialized = false; //avoid optimization from compiler

  //CrowdSimInterface related
  std::string _resourcePath;
  std::string _behaviorFile;
  std::string _sceneFile;
  float _simTimeStep; //must be initialized from world file
  std::vector<std::string> _externalAgents; //loaded from world file, store unique model name


  //template model type for construct corresponding model for agents
  std::shared_ptr<crowd_simulator::ModelTypeDatabase> _modelTypeDBPtr;
  std::shared_ptr<crowd_simulator::CrowdSimInterface> _crowdSimInterface;

  using UpdateObjectTask = Task<double, double>;
  TaskManager<UpdateObjectTask> _updateTaskManager;

  void _Update(const gazebo::common::UpdateInfo& updateInfo); //Update trigger function
  void _UpdateObject(double deltaTime, double deltaSimTime,
    const crowd_simulator::AgentPtr agentPtr,
    const gazebo::physics::ModelPtr modelPtr,
    const crowd_simulator::ModelTypeDatabase::Record* typePtr);

  ignition::math::Pose3d _AnimationRootPose(
    const gazebo::physics::ActorPtr actorPtr,
    const gazebo::common::SkeletonAnimation* animation);

  bool _LoadParams(const sdf::ElementPtr& sdf);
  bool _LoadCrowdSim();
  bool _LoadModelInitPose(const sdf::ElementPtr& modelTypeElement,
    crowd_simulator::AgentPose3d& result) const;
  bool _CreateModel(const std::string& modelName,
    const crowd_simulator::ModelTypeDatabase::Record* modelTypePtr,
    const crowd_simulator::AgentPtr agentPtr);

};


} //namespace crowd_simulation_gazebo

#endif // CROWD_SIMULATION_GAZEBO__CROWD_SIMULATOR_GAZEBO_HPP