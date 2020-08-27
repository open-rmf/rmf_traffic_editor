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
CrowdSimulatorPlugin::CrowdSimulatorPlugin(){

    this->_modelTypeDBPtr = std::make_shared<crowd_simulator::ModelTypeDatabase>();
    this->_transportNodePtr = std::make_shared<ignition::transport::Node>();
}

//=================================================
void CrowdSimulatorPlugin::Configure(const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        ignition::gazebo::EntityComponentManager& ecm, ignition::gazebo::EventManager& eventMgr) 
{
    
    this->_world = std::make_shared<ignition::gazebo::Model>(entity);
    igndbg << "Initializing world plugin with name: " << this->_world->Name(ecm) << std::endl;
    this->_worldName = this->_world->Name(ecm);

    if(!this->_LoadParams(sdf)){
        ignerr << "Error loading crowd simulator plugin. Load params failed!" << std::endl;
        exit(EXIT_FAILURE);
    }

    if(!this->_LoadCrowdSim()){
        ignerr << "Error loading crowd simulator plugin. Load [ Menge ] failed!" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    this->_pluginInitialized = false;
    this->_agentInitialized = false;

}

//=================================================
void CrowdSimulatorPlugin::PreUpdate(const ignition::gazebo::UpdateInfo& info, ignition::gazebo::EntityComponentManager& ecm)
{
    if(!this->_pluginInitialized){

        if(!this->_agentInitialized){
            // add external agents to menge, and send spawn actor service from menge to plugin
            if(!this->_LoadAgents(ecm)){
                ignerr << "Error loading agents! External agents not found!" << std::endl;
                exit(EXIT_FAILURE);
            } else{
                this->_agentInitialized = true;
            }
        }

        // check all the spawned actors are loaded
        if(this->_CheckSpawnedAgents(ecm)){ // all the spawned actors are loaded

            this->_lastSimTime = info.simTime;
            this->_lastAnimTime = info.simTime;

            this->_pluginInitialized = true;
            std::cout << "Crowd simulator plugin initialized complete!" << std::endl;
            std::cout << "Start crowd simulation..." << std::endl;
        }
        return;
    }

    std::chrono::duration<double> deltaTime_tmp = info.simTime - this->_lastAnimTime;
    double deltaTime = deltaTime_tmp.count();
    this->_lastAnimTime = info.simTime;

    std::chrono::duration<double> deltaSimTime_tmp = info.simTime - this->_lastSimTime;
    double deltaSimTime = deltaSimTime_tmp.count();
    if(this->_simTimeStep > deltaSimTime){ // not reach one time sim update
        deltaSimTime = 0.0;
    } else{
        this->_lastSimTime = info.simTime;
        this->_crowdSimInterface->OneStepSim();
    }
    this->_UpdateObject(deltaTime, deltaSimTime, ecm);
}


//=================================================
bool CrowdSimulatorPlugin::_LoadParams(const std::shared_ptr<const sdf::Element>& sdf){

    if (!sdf->HasElement("resource_path"))
    {   
        char* menge_resource_path;
        menge_resource_path = getenv("MENGE_RESOURCE_PATH");
        ignwarn << "No <resource_path> provided! <env MENGE_RESOURCE_PATH> " << std::string(menge_resource_path) <<" will be used." <<
        std::endl;
        this->_resourcePath = std::string(menge_resource_path);
    } else{
        this->_resourcePath = sdf->Get<std::string>("resource_path");
    }
    
    if(!sdf->HasElement("behavior_file")){
        ignerr << "No <behavior_file> provided!" << std::endl;
        return false;
    }
    this->_behaviorFile = sdf->Get<std::string>("behavior_file");

    if(!sdf->HasElement("scene_file")){
        ignerr << "No <scene_file> provided!" << std::endl;
        return false;
    }
    this->_sceneFile = sdf->Get<std::string>("scene_file");

    if(!sdf->HasElement("update_time_step")){
        ignerr << "No <update_time_step> provided!" << std::endl;
        return false;
    }
    this->_simTimeStep = sdf->Get<double>("update_time_step");


    //need to check whether these external_agent exsist in the world file
    if(!sdf->HasElement("external_agent")){
        ignwarn << "No <external_agent> provided! <external_agent> is needed with a unique name defined in world file." << std::endl;
    }
    auto externalAgentElement = sdf->GetElementImpl("external_agent");
    while(externalAgentElement){
        auto exAgentName = externalAgentElement->Get<std::string>();
        this->_externalAgents.emplace_back(exAgentName);
        externalAgentElement = externalAgentElement->GetNextElement("external_agent");
    }

    if (!sdf->HasElement("model_type")) {
        ignerr << "No model type for agents found! <model_type> element Required!" << std::endl;
        return false;
    }
    auto modelTypeElement = sdf->GetElementImpl("model_type");
    while(modelTypeElement){
        std::string s;
        if (!modelTypeElement->Get<std::string>("typename", s, "")) {
            ignerr <<
                "No model type name configured in <model_type>! <typename> Required"
                    << std::endl;
            return false;
        }
        
        auto modelTypePtr = this->_modelTypeDBPtr->Emplace(s, new crowd_simulator::ModelTypeDatabase::Record()); //unordered_map
        modelTypePtr->typeName = s;

        if (!modelTypeElement->Get<std::string>("animation", modelTypePtr->animation, "")){
            ignerr <<
                "No animation configured in <model_type>! <animation> Required" <<
                std::endl;
            return false;
        }

        if (!modelTypeElement->Get<double>("animation_speed", modelTypePtr->animationSpeed, 0.0)) {
            ignerr <<
                "No animation speed configured in <model_type>! <animation_speed> Required"
                    << std::endl;
            return false;
        }
        
        // important for ign plugin
        if (!modelTypeElement->HasElement("initial_pose")) {
            ignerr <<
                "No model initial pose configured in <model_type>! <initial_pose> Required"
                    << std::endl;
            return false;
        }
        if (!this->_LoadModelInitPose(modelTypeElement, modelTypePtr->pose)) {
            ignerr <<
                "Error loading model initial pose in <model_type>! Check <initial_pose>" <<
                std::endl;
            return false;
        }

        if (!modelTypeElement->Get<std::string>("model_file_path", modelTypePtr->modelFilePath, "")){
            ignerr <<
                "No model file path configured in <model_type>! <model_file_path> Required" <<
                std::endl;
            return false;
        }

        modelTypeElement = modelTypeElement->GetNextElement("model_type");

    }

    return true;
}

bool CrowdSimulatorPlugin::_LoadModelInitPose(const sdf::ElementPtr& modelTypeElement, crowd_simulator::AgentPose3d& result) const {

    std::string poseStr;
    if (modelTypeElement->Get<std::string>("initial_pose", poseStr, ""))
    {
        std::regex ws_re("\\s+"); //whitespace
        std::vector<std::string> parts(
        std::sregex_token_iterator(poseStr.begin(), poseStr.end(), ws_re, -1),
        std::sregex_token_iterator());
        
        if (parts.size() != 6)
        {
        ignerr <<
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

bool CrowdSimulatorPlugin::_LoadCrowdSim(){

    _crowdSimInterface = std::make_shared<crowd_simulator::CrowdSimInterface>(
        this->_resourcePath,
        this->_behaviorFile,
        this->_sceneFile,
        this->_simTimeStep);

    assert(_crowdSimInterface);
    return true;
    
    // Unlike gazebo, ignition gazebo seems load the plugin in first, and then load all the models
    // So when processing Configure() function, seems like you can't get models defined in the world file
    // No matter you put the plugin section in the front or in the back
}

bool CrowdSimulatorPlugin::_LoadAgents(ignition::gazebo::EntityComponentManager& ecm) {

    // First check all the external agents are defined in world file
    // for Each() the callback function should always return true to keep iteration. a false return will stop iteration.
    std::unordered_set<std::string> tmp_external_agents;
    for(auto s : this->_externalAgents){
        tmp_external_agents.insert(s);
    }
    std::cout << "Start loading agents." << std::endl;

    ecm.Each<ignition::gazebo::components::Model,
        ignition::gazebo::components::Name,
        ignition::gazebo::components::Static>(
            [&](const ignition::gazebo::Entity& entity,
            const ignition::gazebo::components::Model*,
            const ignition::gazebo::components::Name* name,
            const ignition::gazebo::components::Static* is_static) -> bool{
                if(is_static->Data() == true){
                    if(tmp_external_agents.find(name->Data()) != tmp_external_agents.end()){
                        std::cout << "Crowd Simulator found external agent: " << name->Data() << std::endl;
                        tmp_external_agents.erase(name->Data()); 

                        // update in entityDic
                        this->_entityDic[name->Data()] = entity;
                    }
                }
                return true;
            }
        );
    
    if(tmp_external_agents.size() != 0){
        ignerr << "Following external agents are not defined in world file:" << std::endl;
        for(auto s : tmp_external_agents){
            ignerr << s << std::endl;
        }
        // TODO: at this stage, the external_agent might not yet loaded, which makes the plugin fail to load
        return false;
    }

    // create crowd sim object database, loaded all agents defined in scene file
    // assigned external agents with specific model name in world
    // internal agents named by "agent[NO.]"
    this->_crowdSimInterface->SpawnObject(this->_externalAgents);
    std::cout << "External agents loaded success." << std::endl;

    //create model in world for each internal agents
    size_t objectCount = this->_crowdSimInterface->GetNumObjects();
    for (size_t id = 0; id < objectCount; ++id)
    {
        auto objectPtr = this->_crowdSimInterface->GetObjectById(id);
        assert(objectPtr);
        this->_objectDic[objectPtr->modelName] = id;

        if (!objectPtr->isExternal) {
            //TODO: specify different modelType
            // (size_t) agentPtr->_class identifies the profile type defined in scene.xml
            // this is corresponds to the modelType id
            auto typePtr = this->_modelTypeDBPtr->Get(objectPtr->agentPtr->_typeName);
            assert(typePtr);

            if (!this->_CreateEntity(ecm, objectPtr->modelName, typePtr, objectPtr->agentPtr) ) {
                ignerr << "Failed to insert model [ "<< objectPtr->modelName << " ] in world" << std::endl;
                return false;
            }
        }
    }

    return true;
}

bool CrowdSimulatorPlugin::_CreateEntity(ignition::gazebo::EntityComponentManager& ecm,
    const std::string& modelName, 
    const crowd_simulator::ModelTypeDatabase::RecordPtr modelTypePtr, 
    const crowd_simulator::AgentPtr agentPtr) {

    // Use ignition create service to spawn actors
    // calling ignition gazebo create service, you can use "ign service -l" to check the service available
    assert(modelTypePtr);
    std::string service = "/world/" + this->_worldName + "/create";
    ignition::msgs::EntityFactory request;
    // TODO: direct to the predifined actor sdf, which is specified in modelTypePtr->modelFilePath
    request.set_sdf_filename(modelTypePtr->modelFilePath);
    request.set_name(modelName);
    ignition::math::Pose3d pose(0, 0, 0, 0, 0, 0);

    ignition::msgs::Boolean response;
    bool result;
    uint32_t timeout = 5000;
    bool executed = this->_transportNodePtr->Request(service, request, timeout, response, result);
    if (executed){
        if (result && response.data()) {
            std::cout << "Requested creation of entity: " << modelName << std::endl;
            return true;
        } else {
            ignerr << "Failed request to create entity.\n" << request.DebugString() << std::endl;
        }
    } else{
        ignerr << "Request to create entity from service " << service << " timed out..\n" << request.DebugString() << std::endl;
    }

    return false;
}

bool CrowdSimulatorPlugin::_CheckSpawnedAgents(ignition::gazebo::EntityComponentManager& ecm){

    size_t spawned_number = this->_objectDic.size() - this->_externalAgents.size();

    ecm.Each<ignition::gazebo::components::Actor,
                ignition::gazebo::components::Name> (
            [&](const ignition::gazebo::Entity& entity,
            ignition::gazebo::components::Actor* actor,
            ignition::gazebo::components::Name* name) -> bool{

                if(this->_objectDic.find(name->Data()) != this->_objectDic.end()){

                    // initialize agent world pose
                    auto objectPtr = this->_crowdSimInterface->GetObjectById(this->_objectDic[name->Data()]);
                    assert(objectPtr);
                    auto agentPtr = objectPtr->agentPtr;
                    // different from gazebo plugin, the pose component is the origin of the trajPose
                    ignition::math::Pose3d actor_pose(
                        static_cast<double>(agentPtr->_pos.x()), static_cast<double>(agentPtr->_pos.y()), 0.0,
                        0, 0, 0
                    );

                    auto poseComp = ecm.Component<ignition::gazebo::components::Pose>(entity);
                    if(nullptr == poseComp){
                        ecm.CreateComponent(entity, ignition::gazebo::components::Pose(ignition::math::Pose3d(0, 0, 1.0, 0, 0, 0)));
                    } else{
                        //original pose in the world
                        *poseComp = ignition::gazebo::components::Pose(ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
                    }

                    // initialize agent animationName
                    std::string animationName = this->_modelTypeDBPtr->Get(objectPtr->typeName)->animation;
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

                    // update internal agent in the entity Dic
                    this->_entityDic[name->Data()] = entity;
                    spawned_number--;

                }

                return true;
            }
    );

    if(0 == spawned_number){
        return true;
    }
    return false;

}

void CrowdSimulatorPlugin::_UpdateObject(double deltaTime, double deltaSimTime, ignition::gazebo::EntityComponentManager& ecm){

    // update the external agent world pose, not time related
    for(auto external_name : this->_externalAgents){

        assert(this->_entityDic.end() != this->_entityDic.find(external_name));

        auto external_entity = this->_entityDic[external_name];
        auto poseComp = ecm.Component<ignition::gazebo::components::Pose>(external_entity);

        auto objectPtr = this->_crowdSimInterface->GetObjectById(this->_objectDic[external_name]);
        assert(objectPtr);

        this->_crowdSimInterface->UpdateExternalAgent(objectPtr->agentPtr, Convert(poseComp->Data()) );
    }

    if(deltaSimTime - 0.0 < 1e-6) return;

    // update the internal agent world pose and the animation
    for(auto internal_agent_entity : this->_entityDic){
        auto agent_name = internal_agent_entity.first;

        auto object_ptr = this->_crowdSimInterface->GetObjectById(this->_objectDic[agent_name]);
        assert(object_ptr);
        if(object_ptr->isExternal) continue;

        auto agent_ptr = object_ptr->agentPtr;
        assert(agent_ptr);

        auto entity = internal_agent_entity.second;


        double animation_speed = this->_modelTypeDBPtr->Get(object_ptr->typeName)->animationSpeed;
        auto initial_pose = Convert(this->_modelTypeDBPtr->Get(object_ptr->typeName)->pose);

        crowd_simulator::AgentPose3d agent_pose;
        this->_crowdSimInterface->GetAgentPose(agent_ptr, deltaSimTime, agent_pose);
        ignition::math::Pose3d update_pose = Convert(agent_pose) + initial_pose;

        auto trajPoseComp = ecm.Component<ignition::gazebo::components::TrajectoryPose>(entity);
        if(nullptr == trajPoseComp){
            ignerr << agent_name << " has no TrajectoryPose component."<< std::endl;
            exit(EXIT_FAILURE);
        }
        ignition::math::Pose3d current_pose = trajPoseComp->Data();
        
        //eliminate z coordinate
        auto distance_traveled_vector = update_pose.Pos() - current_pose.Pos();
        distance_traveled_vector.Z(0.0);
        double distance_traveled = distance_traveled_vector.Length();
        // double distance_traveled = deltaSimTime * static_cast<double>(agent_ptr->_vel.Length());

        *trajPoseComp = ignition::gazebo::components::TrajectoryPose(update_pose);
        ecm.SetChanged(entity, 
            ignition::gazebo::components::TrajectoryPose::typeId, 
            ignition::gazebo::ComponentState::OneTimeChange);
        
        auto animTimeComp = ecm.Component<ignition::gazebo::components::AnimationTime>(entity);
        if(nullptr == animTimeComp){
            ignerr << agent_name << " has no TrajectoryPose component."<< std::endl;
            exit(EXIT_FAILURE);
        }
        auto animTime = animTimeComp->Data() + 
            std::chrono::duration_cast<std::chrono::steady_clock::duration>( std::chrono::duration<double>(distance_traveled / animation_speed ));

        *animTimeComp = ignition::gazebo::components::AnimationTime(animTime);
        ecm.SetChanged(entity, 
            ignition::gazebo::components::AnimationTime::typeId, 
            ignition::gazebo::ComponentState::OneTimeChange);
    }

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

} //namespace crowd_simulation_ign

IGNITION_ADD_PLUGIN(
  crowd_simulation_ign::CrowdSimulatorPlugin,
  ignition::gazebo::System,
  crowd_simulation_ign::CrowdSimulatorPlugin::ISystemConfigure,
  crowd_simulation_ign::CrowdSimulatorPlugin::ISystemPreUpdate)

// TODO would prefer namespace
IGNITION_ADD_PLUGIN_ALIAS(crowd_simulation_ign::CrowdSimulatorPlugin, "crowd_simulator")