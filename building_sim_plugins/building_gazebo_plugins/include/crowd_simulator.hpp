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

  bool _initialized = false; //avoid optimization from compiler

  //CrowdSimInterface related
  std::string _resourcePath;
  std::string _behaviorFile;
  std::string _sceneFile;
  float _simTimeStep; //must be initialized from world file
  std::vector<std::string> _externalAgents; //loaded from world file, store unique model name


  //template model type for construct corresponding model for agents
  std::shared_ptr<crowd_simulator::ModelTypeDatabase> _modelTypeDBPtr;
  std::shared_ptr<crowd_simulator::CrowdSimInterface> _crowdSimInterface;

  void _Update(const gazebo::common::UpdateInfo& updateInfo); //Update trigger function
  void _UpdateAllObjects(double deltaTime, double deltaSimTime);
  void _UpdateInternalObject(double deltaTime, double deltaSimTime,
    const crowd_simulator::AgentPtr agentPtr,
    const gazebo::physics::ModelPtr modelPtr,
    const crowd_simulator::ModelTypeDatabase::RecordPtr typePtr);
  void _Initialization();

  ignition::math::Pose3d _AnimationRootPose(
    const gazebo::physics::ActorPtr actorPtr,
    const gazebo::common::SkeletonAnimation* animation);

  bool _LoadParams(const sdf::ElementPtr& sdf);
  bool _LoadCrowdSim();
  bool _CreateModel(const std::string& modelName,
    const crowd_simulator::ModelTypeDatabase::RecordPtr modelTypePtr,
    const crowd_simulator::AgentPtr agentPtr);

};


} //namespace crowd_simulation_gazebo

#endif // CROWD_SIMULATION_GAZEBO__CROWD_SIMULATOR_GAZEBO_HPP