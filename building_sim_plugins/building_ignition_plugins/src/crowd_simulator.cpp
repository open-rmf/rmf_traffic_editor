#include <regex>
#include <cstdlib>
#include <unordered_set>

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
    if(_initialized) {
        _initSpawnedAgents(ecm);
        return;
    }

    std::chrono::duration<double> deltaTime_tmp = info.simTime - this->_lastTime;
    double deltaTime = deltaTime_tmp.count();
    this->_lastTime = info.simTime;

    std::chrono::duration<double> deltaSimTime_tmp = info.simTime - this->_lastSimTime;
    double deltaSimTime = deltaSimTime_tmp.count();
    // if(this->_simTimeStep > deltaSimTime){ // not reach one time sim update
    //     deltaSimTime = 0.0;
    // } else{
    //     this->_lastSimTime = info.simTime;
    //     this->_crowdSimInterface->oneStepSim();
    // }
    // this->_UpdateObject(deltaTime, deltaSimTime, ecm);
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
            if (!this->_CreateEntity(ecm, objectPtr->modelName, typePtr, objectPtr->agentPtr) ) {
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
    std::unordered_set<std::string> objects_name;
    size_t objectCount = _crowdSimInterface->getNumObjects();
    for (size_t id = 0; id < objectCount; id++) {
        auto obj = _crowdSimInterface->getObjectById(id);
        objects_name.insert(obj->modelName);
    }

    ecm.Each<ignition::gazebo::components::Model,
        ignition::gazebo::components::Name>(
            [&](const ignition::gazebo::Entity& entity,
            const ignition::gazebo::components::Model*,
            const ignition::gazebo::components::Name* name) -> bool {
                if(objects_name.find(name->Data()) != objects_name.end()){
                    RCLCPP_INFO(_crowdSimInterface->logger(),
                        "Crowd Simulator found external agent: " + name->Data() );
                    objects_name.erase(name->Data()); 
                    // update in entityDic
                    this->_entityDic[name->Data()] = entity;
                }
                return true;
            }
        );
    
    // external agents not found or not loaded yet
    if(objects_name.size() != 0) {
        std::string name_list = "";
        for(auto name : objects_name){
            name_list += name + " ";
        }
        RCLCPP_WARN(_crowdSimInterface->logger(), 
            "Following external agents are not found or not yet loaded in world file: " + name_list);
        return;
    }
    _initialized = true;
    RCLCPP_INFO(_crowdSimInterface->logger(), "Ignition Models are all loaded! Start simulating...");
}

//===================================================================
bool CrowdSimulatorPlugin::_CreateEntity(ignition::gazebo::EntityComponentManager& ecm,
    const std::string& modelName, 
    const crowd_simulator::ModelTypeDatabase::RecordPtr modelTypePtr, 
    const crowd_simulator::AgentPtr agentPtr) {

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


// bool CrowdSimulatorPlugin::_CheckSpawnedAgents(ignition::gazebo::EntityComponentManager& ecm){

//     size_t spawned_number = this->_objectDic.size() - this->_externalAgents.size();

//     ecm.Each<ignition::gazebo::components::Actor,
//                 ignition::gazebo::components::Name> (
//             [&](const ignition::gazebo::Entity& entity,
//             ignition::gazebo::components::Actor* actor,
//             ignition::gazebo::components::Name* name) -> bool{

//                 if(this->_objectDic.find(name->Data()) != this->_objectDic.end()){

//                     // initialize agent world pose
//                     auto objectPtr = this->_crowdSimInterface->getObjectById(this->_objectDic[name->Data()]);
//                     assert(objectPtr);
//                     auto agentPtr = objectPtr->agentPtr;
//                     // different from gazebo plugin, the pose component is the origin of the trajPose
//                     ignition::math::Pose3d actor_pose(
//                         static_cast<double>(agentPtr->_pos.x()), static_cast<double>(agentPtr->_pos.y()), 0.0,
//                         0, 0, 0
//                     );

//                     auto poseComp = ecm.Component<ignition::gazebo::components::Pose>(entity);
//                     if(nullptr == poseComp){
//                         ecm.CreateComponent(entity, ignition::gazebo::components::Pose(ignition::math::Pose3d(0, 0, 1.0, 0, 0, 0)));
//                     } else{
//                         //original pose in the world
//                         *poseComp = ignition::gazebo::components::Pose(ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
//                     }

//                     // initialize agent animationName
//                     std::string animationName = _crowdSimInterface->_modelTypeDBPtr->Get(objectPtr->typeName)->animation;
//                     assert(!animationName.empty());

//                     auto animationNameComp = ecm.Component<ignition::gazebo::components::AnimationName>(entity);
//                     if(nullptr == animationNameComp){
//                         ecm.CreateComponent(entity, ignition::gazebo::components::AnimationName(animationName));
//                     } else{
//                         *animationNameComp = ignition::gazebo::components::AnimationName(animationName);
//                     }
//                     //mark as one-time-change
//                     ecm.SetChanged(entity, ignition::gazebo::components::AnimationName::typeId, ignition::gazebo::ComponentState::OneTimeChange);
                    
//                     // initialize agent animationTime
//                     auto animTimeComp = ecm.Component<ignition::gazebo::components::AnimationTime>(entity);
//                     if(nullptr == animTimeComp){
//                         ecm.CreateComponent(entity, ignition::gazebo::components::AnimationTime());
//                     }

//                     // having a trajectory pose prevents the actor from moving with the sdf script
//                     auto trajPoseComp = ecm.Component<ignition::gazebo::components::TrajectoryPose>(entity);
//                     if(nullptr == trajPoseComp){
//                         ecm.CreateComponent(entity, ignition::gazebo::components::TrajectoryPose(actor_pose));
//                     }

//                     // update internal agent in the entity Dic
//                     this->_entityDic[name->Data()] = entity;
//                     spawned_number--;

//                 }

//                 return true;
//             }
//     );

//     if(0 == spawned_number){
//         return true;
//     }
//     return false;

// }

// void CrowdSimulatorPlugin::_UpdateObject(double deltaTime, double deltaSimTime, ignition::gazebo::EntityComponentManager& ecm){

//     // update the external agent world pose, not time related
//     for(auto external_name : this->_externalAgents){

//         assert(this->_entityDic.end() != this->_entityDic.find(external_name));

//         auto external_entity = this->_entityDic[external_name];
//         auto poseComp = ecm.Component<ignition::gazebo::components::Pose>(external_entity);

//         auto objectPtr = this->_crowdSimInterface->getObjectById(this->_objectDic[external_name]);
//         assert(objectPtr);

//         this->_crowdSimInterface->UpdateExternalAgent(objectPtr->agentPtr, Convert(poseComp->Data()) );
//     }

//     if(deltaSimTime - 0.0 < 1e-6) return;

//     // update the internal agent world pose and the animation
//     for(auto internal_agent_entity : this->_entityDic){
//         auto agent_name = internal_agent_entity.first;

//         auto object_ptr = this->_crowdSimInterface->getObjectById(this->_objectDic[agent_name]);
//         assert(object_ptr);
//         if(object_ptr->isExternal) continue;

//         auto agent_ptr = object_ptr->agentPtr;
//         assert(agent_ptr);

//         auto entity = internal_agent_entity.second;


//         double animation_speed = _crowdSimInterface->_modelTypeDBPtr->Get(object_ptr->typeName)->animationSpeed;
//         auto initial_pose = Convert(_crowdSimInterface->_modelTypeDBPtr->Get(object_ptr->typeName)->pose);

//         crowd_simulator::AgentPose3d agent_pose;
//         this->_crowdSimInterface->GetAgentPose(agent_ptr, deltaSimTime, agent_pose);
//         ignition::math::Pose3d update_pose = Convert(agent_pose) + initial_pose;

//         auto trajPoseComp = ecm.Component<ignition::gazebo::components::TrajectoryPose>(entity);
//         if(nullptr == trajPoseComp){
//             ignerr << agent_name << " has no TrajectoryPose component."<< std::endl;
//             exit(EXIT_FAILURE);
//         }
//         ignition::math::Pose3d current_pose = trajPoseComp->Data();
        
//         //eliminate z coordinate
//         auto distance_traveled_vector = update_pose.Pos() - current_pose.Pos();
//         distance_traveled_vector.Z(0.0);
//         double distance_traveled = distance_traveled_vector.Length();
//         // double distance_traveled = deltaSimTime * static_cast<double>(agent_ptr->_vel.Length());

//         *trajPoseComp = ignition::gazebo::components::TrajectoryPose(update_pose);
//         ecm.SetChanged(entity, 
//             ignition::gazebo::components::TrajectoryPose::typeId, 
//             ignition::gazebo::ComponentState::OneTimeChange);
        
//         auto animTimeComp = ecm.Component<ignition::gazebo::components::AnimationTime>(entity);
//         if(nullptr == animTimeComp){
//             ignerr << agent_name << " has no TrajectoryPose component."<< std::endl;
//             exit(EXIT_FAILURE);
//         }
//         auto animTime = animTimeComp->Data() + 
//             std::chrono::duration_cast<std::chrono::steady_clock::duration>( std::chrono::duration<double>(distance_traveled / animation_speed ));

//         *animTimeComp = ignition::gazebo::components::AnimationTime(animTime);
//         ecm.SetChanged(entity, 
//             ignition::gazebo::components::AnimationTime::typeId, 
//             ignition::gazebo::ComponentState::OneTimeChange);
//     }

// }

// void CrowdSimulatorPlugin::_UpdateInternalObject(
//     double deltaTime, double deltaAnimTime,
//     const crowd_simulator::AgentPtr agentPtr,
//     const gazebo::physics::ModelPtr modelPtr,
//     const crowd_simulator::ModelTypeDatabase::RecordPtr typePtr,
//     ignition::gazebo::EntityComponentManager& ecm
// ) {

// }

} //namespace crowd_simulation_ign

IGNITION_ADD_PLUGIN(
  crowd_simulation_ign::CrowdSimulatorPlugin,
  ignition::gazebo::System,
  crowd_simulation_ign::CrowdSimulatorPlugin::ISystemConfigure,
  crowd_simulation_ign::CrowdSimulatorPlugin::ISystemPreUpdate)

// TODO would prefer namespace
IGNITION_ADD_PLUGIN_ALIAS(crowd_simulation_ign::CrowdSimulatorPlugin, "crowd_simulator")