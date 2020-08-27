#include <memory>
#include <unordered_set>
#include <regex>
#include <cstdlib>

#include <ignition/math/Pose3.hh>

#include "crowd_simulator.hpp"

namespace crowd_simulation_gazebo {

//============================================
CrowdSimulatorPlugin::CrowdSimulatorPlugin()
{
  this->_modelTypeDBPtr = std::make_shared<crowd_simulator::ModelTypeDatabase>();
}

//WorldPlugin
void CrowdSimulatorPlugin::Load(gazebo::physics::WorldPtr world,
  sdf::ElementPtr sdf)
{

  this->_world = world;
  if (!this->_LoadParams(sdf))
  {
    gzerr << "Error loading crowd simulator plugin. Load params failed." <<
      std::endl;
    return;
  }

  if (!this->_LoadCrowdSim())
  {
    gzerr << "Failed initializing crowd simulator interface." << std::endl;
    return;
  }

  this->_updateConnectionPtr = gazebo::event::Events::ConnectWorldUpdateBegin(
    [this](gazebo::common::UpdateInfo updateInfo)
    {
      this->_Update(updateInfo);
    }
  );
}

//============================================
void CrowdSimulatorPlugin::_Update(const gazebo::common::UpdateInfo& updateInfo)
{
  //first round do nothing, initialize time stamp
  if (this->_lastSimTime == gazebo::common::Time::Zero)
  {
    this->_lastSimTime = updateInfo.simTime;
    this->_lastAnimTime = updateInfo.simTime;
  }

  // if initialized, do updates
  if (this->_initialized)
  {
    auto deltaTime = (updateInfo.simTime - this->_lastAnimTime).Double();
    this->_lastAnimTime = updateInfo.simTime;

    auto deltaSimTime = (updateInfo.simTime - this->_lastSimTime).Double();
    if (deltaSimTime < this->_simTimeStep)
    {
      deltaSimTime = 0.0;
    }
    else
    {
      this->_lastSimTime = updateInfo.simTime;
      this->_crowdSimInterface->OneStepSim();
    }
    this->_UpdateAllObjects(deltaTime, deltaSimTime);
    return;
  }

  // not initizalied
  this->_Initialization();
}

//============================================
void CrowdSimulatorPlugin::_UpdateAllObjects(double deltaTime, double deltaSimTime) 
{
  for (size_t id = 0; id < _objectsCount; ++id)
  {
    ObjectPtr objPtr = this->_crowdSimInterface->GetObjectById(id);
    gazebo::physics::ModelPtr modelPtr = this->_world->ModelByName(objPtr->modelName);

    //update external agents
    if (objPtr->agentPtr->_external)
    {
      ignition::math::Pose3d pose = modelPtr->WorldPose();
      this->_crowdSimInterface->UpdateExternalAgent(objPtr->agentPtr, Convert(pose));
      continue;
    }

    //update internal agents
    //not yet reach the simulation time step, the internal agent position only updated at simulation time step
    if(deltaSimTime - 0.0 < 1e-6) continue;
    crowd_simulator::ModelTypeDatabase::Record* typePtr = this->_modelTypeDBPtr->Get(objPtr->typeName);
    this->_UpdateInternalObject(deltaTime, deltaSimTime, objPtr->agentPtr, modelPtr, typePtr);
  }
}

//============================================
void CrowdSimulatorPlugin::_UpdateInternalObject(double deltaTime, double deltaSimTime,
  const crowd_simulator::AgentPtr agentPtr,
  const gazebo::physics::ModelPtr modelPtr,
  const crowd_simulator::ModelTypeDatabase::Record* typePtr)
{

  if (!agentPtr)
  {
    gzerr << "Null agentPtr when update Object!" << std::endl;
    assert(agentPtr);
  }
  if (!modelPtr)
  {
    gzerr << "Null modelPtr when update Object!" << std::endl;
    assert(modelPtr);
  }

  //update pose from menge to gazebo
  crowd_simulator::AgentPose3d agent_pose;
  this->_crowdSimInterface->GetAgentPose(agentPtr, deltaSimTime, agent_pose);
  ignition::math::Pose3d pose = Convert(agent_pose); 

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
  auto animPose = this->_AnimationRootPose(actorPtr, animation);
  animPose += Convert(typePtr->pose);

  //update x and y coordinates
  animPose.Pos().X(pose.Pos().X());
  animPose.Pos().Y(pose.Pos().Y());
  animPose.Rot() = pose.Rot() * animPose.Rot();

  actorPtr->SetWorldPose(animPose);

}

//============================================
void CrowdSimulatorPlugin::_Initialization() 
{
  this->_objectsCount = this->_crowdSimInterface->GetNumObjects();
  for (size_t id = 0; id < _objectsCount; ++id)
  {
    ObjectPtr objPtr = this->_crowdSimInterface->GetObjectById(id);
    //if model hasn't been added in world, not initialized, return
    if (!this->_world->ModelByName(objPtr->modelName))
    {
      this->_initialized = false;
      return;
    }

    //model has been added, set internal actors as non-static model and set custom trajectory
    //because only non-static model can interact with slotcars
    if (!objPtr->isExternal)
    {
      gazebo::physics::ModelPtr modelPtr = this->_world->ModelByName(
        objPtr->modelName);
      gazebo::physics::ActorPtr actorPtr =
        boost::dynamic_pointer_cast<gazebo::physics::Actor>(modelPtr);
      gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new gazebo::physics::
        TrajectoryInfo()); //matches the actor skeleton
      
      crowd_simulator::ModelTypeDatabase::Record* typePtr = this->_modelTypeDBPtr->Get(objPtr->typeName);
      trajectoryInfo->type = typePtr->animation;
      actorPtr->SetCustomTrajectory(trajectoryInfo);
      actorPtr->SetStatic(false);
    }
  }

  this->_initialized = true;
  std::cout << "Gazebo models all loaded! Start simulating..." << std::endl;
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
bool CrowdSimulatorPlugin::_LoadParams(const sdf::ElementPtr& sdf)
{

  //Load Params
  //resource_path is not necessary
  if (!sdf->HasElement("resource_path"))
  {
    char* menge_resource_path;
    menge_resource_path = getenv("MENGE_RESOURCE_PATH");
    gzwarn << "No resource path provided! <env MENGE_RESOURCE_PATH> " << std::string(menge_resource_path) <<" will be used." <<
      std::endl;
    this->_resourcePath = std::string(menge_resource_path);
  } else{
    this->_resourcePath = sdf->GetElement("resource_path")->Get<std::string>();
  }
  

  if (!sdf->HasElement("behavior_file"))
  {
    gzerr << "No behavior file found! <behavior_file> Required!" << std::endl;
    return false;
  }
  this->_behaviorFile = sdf->GetElement("behavior_file")->Get<std::string>();

  if (!sdf->HasElement("scene_file"))
  {
    gzerr << "No scene file found! <scene_file> Required!" << std::endl;
    return false;
  }
  this->_sceneFile = sdf->GetElement("scene_file")->Get<std::string>();

  if (!sdf->HasElement("update_time_step"))
  {
    gzerr << "No update_time_step found! <update_time_step> Required!" <<
      std::endl;
    return false;
  }
  this->_simTimeStep = sdf->GetElement("update_time_step")->Get<float>();

  if (!sdf->HasElement("model_type"))
  {
    gzerr << "No model type for agents found! <model_type> element Required!" <<
      std::endl;
    return false;
  }
  auto modelTypeElement = sdf->GetElement("model_type");
  while (modelTypeElement)
  {
    std::string s;
    if (!modelTypeElement->Get<std::string>("typename", s, ""))
    {
      gzerr <<
        "No model type name configured in <model_type>! <typename> Required"
            << std::endl;
      return false;
    }

    auto modelTypePtr = this->_modelTypeDBPtr->Emplace(s, new crowd_simulator::ModelTypeDatabase::Record()); //unordered_map
    modelTypePtr->typeName = s;

    if (!modelTypeElement->Get<std::string>("filename", modelTypePtr->fileName,
      ""))
    {
      gzerr <<
        "No actor skin configured in <model_type>! <filename> Required" <<
        std::endl;
      return false;
    }

    if (!modelTypeElement->Get<std::string>("animation",
      modelTypePtr->animation, ""))
    {
      gzerr <<
        "No animation configured in <model_type>! <animation> Required" <<
        std::endl;
      return false;
    }

    if (!modelTypeElement->Get<double>("animation_speed",
      modelTypePtr->animationSpeed, 0.0))
    {
      gzerr <<
        "No animation speed configured in <model_type>! <animation_speed> Required"
            << std::endl;
      return false;
    }

    if (!modelTypeElement->HasElement("initial_pose"))
    {
      gzerr <<
        "No model initial pose configured in <model_type>! <initial_pose> Required [" << s << "]" << std::endl;
      return false;
    }
    if (!this->_LoadModelInitPose(modelTypeElement, modelTypePtr->pose))
    {
      gzerr <<
        "Error loading model initial pose in <model_type>! Check <initial_pose> in [" << s << "]" << std::endl;
      return false;
    }

    modelTypeElement = modelTypeElement->GetNextElement("model_type");
  }

  if (!sdf->HasElement("external_agent"))
  {
    gzwarn <<
      "No external agent provided. <external_agent> is needed with a unique name defined above."
           << std::endl;
  }
  auto externalAgentElement = sdf->GetElement("external_agent");
  while (externalAgentElement)
  {
    auto exAgentName = externalAgentElement->Get<std::string>();
    if (!this->_world->ModelByName(exAgentName))
    {
      gzerr << "Model " << exAgentName <<
        " not defined in world! Check <external_agent>" << std::endl;
      return false;
    }
    gzdbg << "Added external agent: [ " << exAgentName << " ]." << std::endl;
    this->_externalAgents.emplace_back(exAgentName); //just store the name
    externalAgentElement =
      externalAgentElement->GetNextElement("external_agent");
  }

  return true;
}

//============================================
bool CrowdSimulatorPlugin::_LoadCrowdSim()
{
  _crowdSimInterface = std::make_shared<crowd_simulator::CrowdSimInterface>(
    _resourcePath,
    _behaviorFile,
    _sceneFile,
    _simTimeStep);

  assert(_crowdSimInterface);

  //create crowd sim object database, loaded all agents defined in scene file
  //assigned external agents with specific model name in world
  //internal agents named by "agent[NO.]"
  _crowdSimInterface->SpawnObject(this->_externalAgents);

  //create model in world for each internal agents
  size_t objectCount = this->_crowdSimInterface->GetNumObjects();
  for (size_t id = 0; id < objectCount; ++id)
  {
    if (!this->_crowdSimInterface->GetObjectById(id)->isExternal)
    {
      auto objectPtr = this->_crowdSimInterface->GetObjectById(id);
      assert(objectPtr);

      //TODO: specify different modelType
      // (size_t) agentPtr->_class identifies the profile type defined in scene.xml
      // this is corresponds to the modelType id
      auto typePtr = this->_modelTypeDBPtr->Get(objectPtr->typeName);
      assert(typePtr);
      if (!this->_CreateModel(objectPtr->modelName, typePtr,
        objectPtr->agentPtr) )
      {
        gzerr << "Failed to insert model [ "<< objectPtr->modelName <<
          " ] in world" << std::endl;
        return false;
      }
    }
  }

  return true;
}


//============================================
bool CrowdSimulatorPlugin::_LoadModelInitPose(
  const sdf::ElementPtr& modelTypeElement,
  crowd_simulator::AgentPose3d& result) const
{
  std::string poseStr;
  if (modelTypeElement->Get<std::string>("initial_pose", poseStr, ""))
  {
    std::regex ws_re("\\s+"); //whitespace
    std::vector<std::string> parts(
      std::sregex_token_iterator(poseStr.begin(), poseStr.end(), ws_re, -1),
      std::sregex_token_iterator());

    if (parts.size() != 6)
    {
      gzerr <<
        "Error loading <initial_pose> in <model_type>, 6 floats (x, y, z, pitch, roll, yaw) expected.";
      return false;
    }

    double x = ignition::math::parseFloat(parts[0]);
    double y = ignition::math::parseFloat(parts[1]);
    double z = ignition::math::parseFloat(parts[2]);
    double pitch = ignition::math::parseFloat(parts[3]);
    double roll = ignition::math::parseFloat(parts[4]);
    double yaw = ignition::math::parseFloat(parts[5]);

    result.X() = x;
    result.Y() = y;
    result.Z() = z;
    result.Pitch() = pitch;
    result.Roll() = roll;
    result.Yaw() = yaw;
  }
  return true;
}


//============================================
bool CrowdSimulatorPlugin::_CreateModel(const std::string& modelName,
  const crowd_simulator::ModelTypeDatabase::Record* modelTypePtr,
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
  this->_world->InsertModelSDF(*sdf);
  gzdbg << "Insert corresponding model for simulator agent: [" << modelName <<
    " ]." << std::endl;
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