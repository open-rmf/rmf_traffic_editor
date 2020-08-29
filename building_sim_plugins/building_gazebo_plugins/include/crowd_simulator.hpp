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

public:
  CrowdSimulatorPlugin() 
    : _crowdSimInterface(std::make_shared<crowd_simulator::CrowdSimInterface>()),
    _initialized(false),
    _objectsCount(0)
  {}

  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

private:
  std::shared_ptr<crowd_simulator::CrowdSimInterface> _crowdSimInterface;
  bool _initialized;
  size_t _objectsCount;
  gazebo::physics::WorldPtr _world;
  gazebo::event::ConnectionPtr _updateConnectionPtr;
  gazebo::common::Time _lastTime;
  gazebo::common::Time _lastSimTime;

  bool _spawnAgentsInWorld();
  void _initSpawnedAgents();
  void _Update(const gazebo::common::UpdateInfo& updateInfo); //Update trigger function
  void _UpdateAllObjects(double deltaTime, double deltaSimTime);
  void _UpdateInternalObject(double deltaTime, double deltaSimTime,
    const crowd_simulator::AgentPtr agentPtr,
    const gazebo::physics::ModelPtr modelPtr,
    const crowd_simulator::ModelTypeDatabase::RecordPtr typePtr);
  
  ignition::math::Pose3d _AnimationRootPose(
    const gazebo::physics::ActorPtr actorPtr,
    const gazebo::common::SkeletonAnimation* animation);

  bool _CreateModel(const std::string& modelName,
    const crowd_simulator::ModelTypeDatabase::RecordPtr modelTypePtr,
    const crowd_simulator::AgentPtr agentPtr);

};


} //namespace crowd_simulation_gazebo

#endif // CROWD_SIMULATION_GAZEBO__CROWD_SIMULATOR_GAZEBO_HPP