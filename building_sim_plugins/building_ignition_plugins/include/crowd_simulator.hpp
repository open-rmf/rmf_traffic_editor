#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>

#include <ignition/transport/Node.hh>

#include <building_sim_common/crowd_simulator_common.hpp>


namespace crowd_simulation_ign {

class IGNITION_GAZEBO_VISIBLE CrowdSimulatorPlugin 
    : public ignition::gazebo::System, 
    public ignition::gazebo::ISystemConfigure, 
    public ignition::gazebo::ISystemPreUpdate
{   
public:
    CrowdSimulatorPlugin();

    // inherit from ISystemConfigure 
    void Configure(const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        ignition::gazebo::EntityComponentManager& ecm, ignition::gazebo::EventManager& eventMgr) override;

    // inherit from ISystemPreUpdate
    void PreUpdate(const ignition::gazebo::UpdateInfo& info, ignition::gazebo::EntityComponentManager& ecm) override;

private:

    std::shared_ptr<ignition::gazebo::Model> _world;
    std::string _worldName;

    std::string _resourcePath;
    std::string _behaviorFile;
    std::string _sceneFile;
    double _simTimeStep; // load "update_time_step" tag from world file
    std::vector<std::string> _externalAgents; //store the name of all "external_agent"

    std::chrono::steady_clock::duration _lastSimTime{0};
    std::chrono::steady_clock::duration _lastAnimTime{0};

    // map for <model_name, object_id>, contains both external (models) and internal agents (actors)
    std::unordered_map<std::string, size_t> _objectDic;
    // map for <model_name, entity_id> contains external and internal agents
    std::unordered_map<std::string, ignition::gazebo::Entity> _entityDic;

    std::shared_ptr<crowd_simulator::ModelTypeDatabase> _modelTypeDBPtr;
    std::shared_ptr<crowd_simulator::CrowdSimInterface> _crowdSimInterface;
    std::shared_ptr<rclcpp::Node> _nodePtr;
    std::shared_ptr<ignition::transport::Node> _transportNodePtr;

    volatile bool _pluginInitialized = false; // disable compiler optimization
    volatile bool _agentInitialized = false;

    bool _LoadParams(const std::shared_ptr<const sdf::Element>& sdf);

    bool _LoadModelInitPose(const sdf::ElementPtr& modelTypeElement, crowd_simulator::AgentPose3d& result) const;
    
    bool _LoadCrowdSim();
    
    bool _LoadAgents(ignition::gazebo::EntityComponentManager& ecm);
   
    bool _CreateEntity(ignition::gazebo::EntityComponentManager& ecm, 
      const std::string& modelName, 
      const crowd_simulator::ModelTypeDatabase::Record* modelTypePtr, 
      const crowd_simulator::AgentPtr agentPtr);
    
    bool _CheckSpawnedAgents(ignition::gazebo::EntityComponentManager& ecm);

    void _UpdateObject(double deltaTime, double deltaAnimTime, ignition::gazebo::EntityComponentManager& ecm);

};

//================================================================================
crowd_simulator::AgentPose3d Convert(const ignition::math::Pose3d& ignition_pose);

ignition::math::Pose3d Convert(const crowd_simulator::AgentPose3d& agent_pose);

} //namespace crowd_simulation_ign