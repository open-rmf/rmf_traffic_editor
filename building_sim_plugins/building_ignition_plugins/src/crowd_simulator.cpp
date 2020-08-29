#include <regex>
#include <cstdlib>

#include <sdf/Actor.hh>

#include <ignition/math/Pose3.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>

#include "crowd_simulator.hpp"

namespace crowd_simulation_ign {

//=================================================
void CrowdSimulatorPlugin::Configure(const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        ignition::gazebo::EntityComponentManager& ecm, ignition::gazebo::EventManager& eventMgr) 
{
    _world = std::make_shared<ignition::gazebo::Model>(entity);
    RCLCPP_INFO(_crowdSimInterface->logger(), 
        "Initializing world plugin with name: " + _world->Name(ecm));
    _worldName = _world->Name(ecm);

    if(!_crowdSimInterface->readSDF(sdf)) {
        RCLCPP_ERROR(_crowdSimInterface->logger(), 
            "Error loading crowd simulator plugin. Load params failed!");
        exit(EXIT_FAILURE);
    }

    if(!_crowdSimInterface->initCrowdSim()) {
        RCLCPP_ERROR(_crowdSimInterface->logger(), 
            "Error loading crowd simulator plugin. Load [ Menge ] failed!");
        exit(EXIT_FAILURE);
    }

    if(!_spawnAgentsInWorld(ecm)) {
        RCLCPP_ERROR(_crowdSimInterface->logger(), 
            "Error loading crowd simulator plugin. Crowd Simulator failed to spawn agents in the world!");
        exit(EXIT_FAILURE);
    }
}

//=================================================
void CrowdSimulatorPlugin::PreUpdate(const ignition::gazebo::UpdateInfo& info, ignition::gazebo::EntityComponentManager& ecm)
{
    if(!_initialized) {
        _initSpawnedAgents(ecm);
        return;
    }

    std::chrono::duration<double> deltaTime_tmp = info.simTime - _lastTime;
    double deltaTime = deltaTime_tmp.count();
    _lastTime = info.simTime;

    std::chrono::duration<double> deltaSimTime_tmp = info.simTime - _lastSimTime;
    double deltaSimTime = deltaSimTime_tmp.count();
    if(_crowdSimInterface->getSimTimeStep() > deltaSimTime){ // not reach one time sim update
        deltaSimTime = 0.0;
    } else{
        this->_lastSimTime = info.simTime;
        this->_crowdSimInterface->oneStepSim();
    }

    this->_UpdateAllObjects(deltaTime, deltaSimTime, ecm);
}

//==========================================================
bool CrowdSimulatorPlugin::_spawnAgentsInWorld(ignition::gazebo::EntityComponentManager& ecm) {
    size_t objectCount = this->_crowdSimInterface->getNumObjects();
    for (size_t id = 0; id < objectCount; ++id)
    {
        auto objectPtr = this->_crowdSimInterface->getObjectById(id);
        assert(objectPtr);
        _objectDic[objectPtr->modelName] = id;

        if (!objectPtr->isExternal) {
            auto typePtr = _crowdSimInterface->_modelTypeDBPtr->Get(objectPtr->agentPtr->_typeName);
            assert(typePtr);
            if (!this->_CreateEntity(ecm, objectPtr->modelName, typePtr) ) {
                RCLCPP_ERROR(_crowdSimInterface->logger(), 
                    "Failed to insert model [ " + objectPtr->modelName + " ] in world");
                return false;
            }
        }
    }
    return true;
}

//==========================================================
void CrowdSimulatorPlugin::_initSpawnedAgents(ignition::gazebo::EntityComponentManager& ecm) {
    // check all the models are in the world
    std::unordered_map<std::string, size_t> objects_name;
    size_t objectCount = _crowdSimInterface->getNumObjects();
    for (size_t id = 0; id < objectCount; id++) {
        auto obj = _crowdSimInterface->getObjectById(id);
        // already found in the Dic
        if(_entityDic.find(obj->modelName) != _entityDic.end())
            continue;
        objects_name.insert( {obj->modelName, id} );
    }

    ecm.Each<ignition::gazebo::components::Model,
        ignition::gazebo::components::Name>(
            [&](const ignition::gazebo::Entity& entity,
            const ignition::gazebo::components::Model*,
            const ignition::gazebo::components::Name* name) -> bool {
                auto it_objects_name = objects_name.find(name->Data());
                if(it_objects_name != objects_name.end()){
                    // update in entityDic
                    _entityDic[name->Data()] = entity;
                    auto objPtr = _crowdSimInterface->getObjectById(it_objects_name->second);
                    // config internal spawned agent for custom trajectory
                    if (!objPtr->isExternal) {
                        _configSpawnedAgents(objPtr, entity, ecm);
                    }
                    objects_name.erase(name->Data()); 
                    RCLCPP_INFO(_crowdSimInterface->logger(),
                        "Crowd Simulator found agent: " + name->Data() );
                }
                return true;
            }
        );
    
    // external agents not found or not loaded yet
    if(objects_name.size() != 0) {
        _initialized = false;
        return;
    }
    _initialized = true;
    RCLCPP_INFO(_crowdSimInterface->logger(), "Ignition Models are all loaded! Start simulating...");
}

//===================================================================
bool CrowdSimulatorPlugin::_CreateEntity(ignition::gazebo::EntityComponentManager& ecm,
    const std::string& modelName, 
    const crowd_simulator::ModelTypeDatabase::RecordPtr modelTypePtr) const 
{
    // Use ignition create service to spawn actors
    // calling ignition gazebo create service, you can use "ign service -l" to check the service available
    assert(modelTypePtr);
    std::string service = "/world/" + this->_worldName + "/create";
    ignition::msgs::EntityFactory request;
    request.set_sdf_filename(modelTypePtr->fileName);
    request.set_name(modelName);
    ignition::math::Pose3d pose(0, 0, 0, 0, 0, 0);

    ignition::msgs::Boolean response;
    bool result;
    uint32_t timeout = 5000;
    bool executed = this->_transportNodePtr->Request(service, request, timeout, response, result);
    if (executed) {
        if (result && response.data()) {
            RCLCPP_INFO(_crowdSimInterface->logger(),
                "Requested creation of entity: " + modelName);
            return true;
        } else {
            RCLCPP_ERROR(_crowdSimInterface->logger(),
                "Failed request to create entity.\n" + request.DebugString());
        }
    } else {
        RCLCPP_ERROR(_crowdSimInterface->logger(),
            "Request to create entity from service " + service + "timer out ...\n" + request.DebugString());
    }
    return false;
}

//==================================================
void CrowdSimulatorPlugin::_configSpawnedAgents(
    const crowd_simulator::CrowdSimInterface::ObjectPtr objPtr,
    const ignition::gazebo::Entity& entity, 
    ignition::gazebo::EntityComponentManager& ecm) const 
{
    assert(objPtr);
    auto agentPtr = objPtr->agentPtr;
    // different from gazebo plugin, the pose component is the origin of the trajPose
    ignition::math::Pose3d actor_pose(
        static_cast<double>(agentPtr->_pos.x()), static_cast<double>(agentPtr->_pos.y()), 0.0,
        0, 0, 0
    );

    // get pose component for entity
    auto poseComp = ecm.Component<ignition::gazebo::components::Pose>(entity);
    if(nullptr == poseComp){
        // use the initial_pose for actor type
        ignition::math::Pose3d initial_pose = _crowdSimInterface->_modelTypeDBPtr->Get(objPtr->typeName)->pose.ConvertToIgnMathPose3d<ignition::math::Pose3d>();
        ecm.CreateComponent(entity, ignition::gazebo::components::Pose(initial_pose));
    } else{
        //original pose in the world
        *poseComp = ignition::gazebo::components::Pose(ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
    }

    // initialize agent animationName
    std::string animationName = _crowdSimInterface->_modelTypeDBPtr->Get(objPtr->typeName)->animation;
    assert(!animationName.empty());

    auto animationNameComp = ecm.Component<ignition::gazebo::components::AnimationName>(entity);
    if(nullptr == animationNameComp){
        ecm.CreateComponent(entity, ignition::gazebo::components::AnimationName(animationName));
    } else{
        *animationNameComp = ignition::gazebo::components::AnimationName(animationName);
    }
    //mark as one-time-change
    ecm.SetChanged(entity, ignition::gazebo::components::AnimationName::typeId, ignition::gazebo::ComponentState::OneTimeChange);
    // initialize agent animationTime
    auto animTimeComp = ecm.Component<ignition::gazebo::components::AnimationTime>(entity);
    if(nullptr == animTimeComp){
        ecm.CreateComponent(entity, ignition::gazebo::components::AnimationTime());
    }
    // having a trajectory pose prevents the actor from moving with the sdf script
    auto trajPoseComp = ecm.Component<ignition::gazebo::components::TrajectoryPose>(entity);
    if(nullptr == trajPoseComp){
        ecm.CreateComponent(entity, ignition::gazebo::components::TrajectoryPose(actor_pose));
    }
}

//============================================================================
void CrowdSimulatorPlugin::_UpdateAllObjects(
    double deltaTime, 
    double deltaSimTime, 
    ignition::gazebo::EntityComponentManager& ecm) const
{
    auto objectsCount = _crowdSimInterface->getNumObjects();
    for (size_t id = 0; id < objectsCount; id++) {
        auto objPtr = _crowdSimInterface->getObjectById(id);
        auto it_entity = _entityDic.find(objPtr->modelName);
        if (it_entity == _entityDic.end()) { //safe check
            RCLCPP_ERROR(_crowdSimInterface->logger(),
                "Didn't initialize external agent [" + objPtr->modelName + "]");
            exit(EXIT_FAILURE);
        }
        auto entity = it_entity->second;

        // for external agent
        if (objPtr->isExternal) {
            auto model_pose = ecm.Component<ignition::gazebo::components::Pose>(entity)->Data();
            _crowdSimInterface->updateExternalAgent(objPtr->agentPtr, model_pose);
            continue;
        }

        // for internal agent
        if(deltaSimTime - 0.0 < 1e-6) continue; // not yet reach the simulation update time
        std::cout << "in here" << std::endl;
        _UpdateInternalObject(deltaSimTime, objPtr, entity, ecm);
    }
}

void CrowdSimulatorPlugin::_UpdateInternalObject(
    double deltaSimTime,
    const crowd_simulator::CrowdSimInterface::ObjectPtr objPtr,
    const ignition::gazebo::Entity& entity,
    ignition::gazebo::EntityComponentManager& ecm) const
{
    double animation_speed = _crowdSimInterface->_modelTypeDBPtr->Get(objPtr->typeName)->animationSpeed;
    ignition::math::Pose3d initial_pose = _crowdSimInterface->_modelTypeDBPtr->Get(objPtr->typeName)->pose.ConvertToIgnMathPose3d<ignition::math::Pose3d>();
    ignition::math::Pose3d agent_pose = _crowdSimInterface->getAgentPose<ignition::math::Pose3d>(objPtr->agentPtr, deltaSimTime);
    agent_pose += initial_pose;

    auto trajPoseComp = ecm.Component<ignition::gazebo::components::TrajectoryPose>(entity);
    if(nullptr == trajPoseComp){
        RCLCPP_ERROR(_crowdSimInterface->logger(),
            "Model [" + objPtr->modelName + "] has no TrajectoryPose component." );
        exit(EXIT_FAILURE);
    }
    ignition::math::Pose3d current_pose = trajPoseComp->Data();
    
    auto distance_traveled_vector = agent_pose.Pos() - current_pose.Pos();
    // might need future work on 3D case
    // the center of human has a z_elevation, which will make the human keep walking even if he reached the target
    distance_traveled_vector.Z(0.0);
    double distance_traveled = distance_traveled_vector.Length();

    // set trajectory
    *trajPoseComp = ignition::gazebo::components::TrajectoryPose(agent_pose);
    ecm.SetChanged(entity, 
        ignition::gazebo::components::TrajectoryPose::typeId, 
        ignition::gazebo::ComponentState::OneTimeChange);
    //set animation
    auto animTimeComp = ecm.Component<ignition::gazebo::components::AnimationTime>(entity);
    if(nullptr == animTimeComp){
        RCLCPP_ERROR(_crowdSimInterface->logger(),
            "Model [" + objPtr->modelName + "] has no AnimationTime component");
        exit(EXIT_FAILURE);
    }
    auto animTime = animTimeComp->Data() + 
        std::chrono::duration_cast<std::chrono::steady_clock::duration>( 
            std::chrono::duration<double>(distance_traveled / animation_speed ));

    *animTimeComp = ignition::gazebo::components::AnimationTime(animTime);
    ecm.SetChanged(entity, 
        ignition::gazebo::components::AnimationTime::typeId, 
        ignition::gazebo::ComponentState::OneTimeChange);
}

} //namespace crowd_simulation_ign

IGNITION_ADD_PLUGIN(
  crowd_simulation_ign::CrowdSimulatorPlugin,
  ignition::gazebo::System,
  crowd_simulation_ign::CrowdSimulatorPlugin::ISystemConfigure,
  crowd_simulation_ign::CrowdSimulatorPlugin::ISystemPreUpdate)

// TODO would prefer namespace
IGNITION_ADD_PLUGIN_ALIAS(crowd_simulation_ign::CrowdSimulatorPlugin, "crowd_simulator")