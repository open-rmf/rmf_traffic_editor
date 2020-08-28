#include <memory>
#include <unordered_set>
#include <regex>
#include <cstdlib>

#include <ignition/math/Pose3.hh>

#include "crowd_simulator.hpp"

namespace crowd_simulation_gazebo {

//============================================
//WorldPlugin
void CrowdSimulatorPlugin::Load(gazebo::physics::WorldPtr world,
  sdf::ElementPtr sdf)
{
  _world = world;
  
  if (!_crowdSimInterface->readSDF(sdf)) {
    return;
  }

  if (!_crowdSimInterface->initCrowdSim()) {
    RCLCPP_ERROR(_crowdSimInterface->logger(), "Crowd simulation failed to initialize.");
    return;
  }

  _spawnAgentsInWorld();

  _updateConnectionPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
    [this](gazebo::common::UpdateInfo updateInfo)
    {
      _Update(updateInfo);
    }
  );
}

//============================================
void CrowdSimulatorPlugin::_Update(const gazebo::common::UpdateInfo& updateInfo)
{
  //first round do nothing, initialize time stamp
  if (_lastSimTime == gazebo::common::Time::Zero)
  {
    _lastSimTime = updateInfo.simTime;
    _lastTime = updateInfo.simTime;
  }

  // if initialized, do updates
  if (_initialized)
  {
    auto deltaTime = (updateInfo.simTime - _lastTime).Double();
    _lastTime = updateInfo.simTime;

    auto deltaSimTime = (updateInfo.simTime - _lastSimTime).Double();
    if (deltaSimTime - _crowdSimInterface->getSimTimeStep() < 1e-6)
    {
      deltaSimTime = 0.0;
    }
    else
    {
      _lastSimTime = updateInfo.simTime;
      _crowdSimInterface->oneStepSim();
    }
    _UpdateAllObjects(deltaTime, deltaSimTime);
    return;
  }

  // not initizalied
  _initSpawnedAgents();
}

//============================================
void CrowdSimulatorPlugin::_UpdateAllObjects(double deltaTime, double deltaSimTime) 
{
  for (size_t id = 0; id < _objectsCount; id++)
  {
    ObjectPtr objPtr = _crowdSimInterface->getObjectById(id);
    gazebo::physics::ModelPtr modelPtr = _world->ModelByName(objPtr->modelName);

    //update external agents
    if (objPtr->agentPtr->_external)
    {
      ignition::math::Pose3d pose = modelPtr->WorldPose();
      _crowdSimInterface->updateExternalAgent<ignition::math::Pose3d>(objPtr->agentPtr, pose);
      continue;
    }

    //update internal agents
    //not yet reach the simulation time step, the internal agent position only updated at simulation time step
    if(deltaSimTime - 0.0 < 1e-6) continue;
    auto typePtr = _crowdSimInterface->_modelTypeDBPtr->Get(objPtr->typeName);
    _UpdateInternalObject(deltaTime, deltaSimTime, objPtr->agentPtr, modelPtr, typePtr);
  }
}

//============================================
void CrowdSimulatorPlugin::_UpdateInternalObject(double deltaTime, double deltaSimTime,
  const crowd_simulator::AgentPtr agentPtr,
  const gazebo::physics::ModelPtr modelPtr,
  const crowd_simulator::ModelTypeDatabase::RecordPtr typePtr)
{

  if (!agentPtr)
  {
    RCLCPP_ERROR(_crowdSimInterface->logger(), "Null agentPtr when update Object!");
    assert(agentPtr);
  }
  if (!modelPtr)
  {
    RCLCPP_ERROR(_crowdSimInterface->logger(), "Null modelPtr when update Object!");
    assert(modelPtr);
  }

  //update pose from menge to gazebo
  ignition::math::Pose3d pose = _crowdSimInterface->getAgentPose<ignition::math::Pose3d>(agentPtr, deltaSimTime);

  gazebo::physics::ActorPtr actorPtr =
    boost::dynamic_pointer_cast<gazebo::physics::Actor>(modelPtr);

  auto delta_dist_vector = pose.Pos() - actorPtr->WorldPose().Pos() ;
  delta_dist_vector.Z(0.0);
  double deltaDist = delta_dist_vector.Length();

  // _simTimeStep is small, then deltaDist is small, the scriptTime is small than expected.
  actorPtr->SetScriptTime(
    actorPtr->ScriptTime() + deltaDist / typePtr->animationSpeed );

  //add on original loaded pose
  auto animation = actorPtr->SkeletonAnimations().at(typePtr->animation);
  auto animPose = _AnimationRootPose(actorPtr, animation);
  animPose += Convert(typePtr->pose);

  //update x and y coordinates
  animPose.Pos().X(pose.Pos().X());
  animPose.Pos().Y(pose.Pos().Y());
  animPose.Rot() = pose.Rot() * animPose.Rot();

  actorPtr->SetWorldPose(animPose);
}

//============================================
void CrowdSimulatorPlugin::_initSpawnedAgents() 
{
  _objectsCount = _crowdSimInterface->getNumObjects();
  for (size_t id = 0; id < _objectsCount; ++id)
  {
    ObjectPtr objPtr = _crowdSimInterface->getObjectById(id);
    // spawned agents are not fully loaded
    if (!_world->ModelByName(objPtr->modelName))
    {
      _initialized = false;
      return;
    }
    // all agents are loaded, set internal actors as non-static model and set custom trajectory
    // because only non-static model can interact with slotcars
    if (!objPtr->isExternal)
    {
      gazebo::physics::ModelPtr modelPtr = _world->ModelByName( objPtr->modelName);
      gazebo::physics::ActorPtr actorPtr =
        boost::dynamic_pointer_cast<gazebo::physics::Actor>(modelPtr);
      gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new gazebo::physics::
        TrajectoryInfo()); //matches the actor skeleton
      
      crowd_simulator::ModelTypeDatabase::RecordPtr typePtr = 
        _crowdSimInterface->_modelTypeDBPtr->Get(objPtr->typeName);
      trajectoryInfo->type = typePtr->animation;
      actorPtr->SetCustomTrajectory(trajectoryInfo);
      actorPtr->SetStatic(false);
    }
  }
  _initialized = true;
  RCLCPP_INFO(_crowdSimInterface->logger(), "Gazebo models all loaded! Start simulating...");
}

//============================================
ignition::math::Pose3d CrowdSimulatorPlugin::_AnimationRootPose(
  const gazebo::physics::ActorPtr actorPtr,
  const gazebo::common::SkeletonAnimation* animation)
{
  auto* rootNode = actorPtr->Mesh()->GetSkeleton()->GetRootNode();
  auto rootNodeName = rootNode->GetName();
  if (!animation->HasNode(rootNodeName))
  {
    throw std::runtime_error("unable to find root node pose");
  }
  //get the animation trans for current time
  auto animTrans = animation->PoseAt(actorPtr->ScriptTime(), true); //map<string, matrix4d>
  auto& rootAnimTrans = animTrans[rootNodeName];

  auto scaleTrans = ignition::math::Matrix4d::Identity;
  auto actorScale = actorPtr->Scale();
  scaleTrans.Scale(actorScale.X(), actorScale.Y(), actorScale.Z());
  rootAnimTrans = scaleTrans * rootAnimTrans;
  return rootAnimTrans.Pose();
}


//============================================
bool CrowdSimulatorPlugin::_spawnAgentsInWorld()
{
  //create model in world for each internal agents
  _objectsCount = _crowdSimInterface->getNumObjects();
  for (size_t id = 0; id < _objectsCount; id++)
  {
    if (!_crowdSimInterface->getObjectById(id)->isExternal)
    {
      auto objectPtr = _crowdSimInterface->getObjectById(id);
      assert(objectPtr);
      auto typePtr = _crowdSimInterface->_modelTypeDBPtr->Get(objectPtr->typeName);
      assert(typePtr);
      if (!_CreateModel(objectPtr->modelName, typePtr, objectPtr->agentPtr) )
      {
        RCLCPP_INFO(_crowdSimInterface->logger(),
          "Failed to insert model [" + objectPtr->modelName + "] in world");
        return false;
      }
    }
  }
  return true;
}

//============================================
bool CrowdSimulatorPlugin::_CreateModel(const std::string& modelName,
  const crowd_simulator::ModelTypeDatabase::RecordPtr modelTypePtr,
  const crowd_simulator::AgentPtr agentPtr)
{
  sdf::ElementPtr modelElement(new sdf::Element());
  modelElement->SetName("actor");
  modelElement->AddAttribute("name", "string", modelName, true);

  sdf::ElementPtr skinElement(new sdf::Element());
  skinElement->SetName("skin");
  modelElement->InsertElement(skinElement);

  sdf::ElementPtr animElement(new sdf::Element());
  animElement->SetName("animation");
  animElement->AddAttribute("name", "string", "walk", true);
  modelElement->InsertElement(animElement);

  sdf::ElementPtr filenameElement(new sdf::Element());
  filenameElement->SetName("filename");
  filenameElement->AddValue("string", modelTypePtr->fileName, true);
  skinElement->InsertElement(filenameElement);
  animElement->InsertElement(filenameElement);

  sdf::ElementPtr poseElement(new sdf::Element());
  poseElement->SetName("pose");
  std::ostringstream oss;

  oss << agentPtr->_pos.x() << " " << agentPtr->_pos.y() << " " << "0 0 0 0";
  poseElement->AddValue("pose", oss.str(), true);
  modelElement->InsertElement(poseElement);

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf->Root(modelElement);

  assert(sdf);
  _world->InsertModelSDF(*sdf);
  RCLCPP_INFO(_crowdSimInterface->logger(),
    "Insert actor for crowd simulator agent: [" + modelName + "] at ["+ oss.str() +"].");
  return true;
}

//===================================================

crowd_simulator::AgentPose3d Convert(const ignition::math::Pose3d& ignition_pose){
  auto pos = ignition_pose.Pos();
  auto euler = ignition_pose.Rot().Euler();

  return crowd_simulator::AgentPose3d(pos.X(), pos.Y(), pos.Z(), euler.X(), euler.Y(), euler.Z());
}

ignition::math::Pose3d Convert(const crowd_simulator::AgentPose3d& agent_pose){
  return ignition::math::Pose3d(
    agent_pose.X(), 
    agent_pose.Y(), 
    agent_pose.Z(), 
    agent_pose.Pitch(), 
    agent_pose.Roll(), 
    agent_pose.Yaw());
}


// insert the plugin
GZ_REGISTER_WORLD_PLUGIN(CrowdSimulatorPlugin)
} //namespace crowd_simulation_gazebo