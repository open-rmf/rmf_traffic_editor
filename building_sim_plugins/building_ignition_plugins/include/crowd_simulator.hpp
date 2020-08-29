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
    CrowdSimulatorPlugin()
     : _transportNodePtr(std::make_shared<ignition::transport::Node>()),
     _crowdSimInterface(std::make_shared<crowd_simulator::CrowdSimInterface>()),
     _initialized(false)
    {}

    // inherit from ISystemConfigure 
    void Configure(const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        ignition::gazebo::EntityComponentManager& ecm, ignition::gazebo::EventManager& eventMgr) override;

    // inherit from ISystemPreUpdate
    void PreUpdate(const ignition::gazebo::UpdateInfo& info, ignition::gazebo::EntityComponentManager& ecm) override;

private:
    std::shared_ptr<ignition::transport::Node> _transportNodePtr;
    std::shared_ptr<crowd_simulator::CrowdSimInterface> _crowdSimInterface;
    bool _initialized;
    std::chrono::steady_clock::duration _lastSimTime{0};
    std::chrono::steady_clock::duration _lastTime{0};
    
    std::shared_ptr<ignition::gazebo::Model> _world;
    std::string _worldName;

    // map for <model_name, object_id>, contains both external (models) and internal agents (actors)
    std::unordered_map<std::string, size_t> _objectDic;
    // map for <model_name, entity_id> contains external and internal agents
    std::unordered_map<std::string, ignition::gazebo::Entity> _entityDic;

    bool _spawnAgentsInWorld(ignition::gazebo::EntityComponentManager& ecm);
    void _initSpawnedAgents(ignition::gazebo::EntityComponentManager& ecm);
    void _configSpawnedAgents(
        const crowd_simulator::CrowdSimInterface::ObjectPtr objPtr,
        const ignition::gazebo::Entity& enity, 
        ignition::gazebo::EntityComponentManager& ecm) const;
   
    bool _CreateEntity(ignition::gazebo::EntityComponentManager& ecm, 
      const std::string& modelName, 
      const crowd_simulator::ModelTypeDatabase::RecordPtr modelTypePtr) const;
    
    void _UpdateAllObjects(
        double deltaTime, double deltaAnimTime, 
        ignition::gazebo::EntityComponentManager& ecm) const;
    void _UpdateInternalObject(
        double deltaSimTime,
        const crowd_simulator::CrowdSimInterface::ObjectPtr objPtr,
        const ignition::gazebo::Entity& enity,
        ignition::gazebo::EntityComponentManager& ecm
    ) const;

};

} //namespace crowd_simulation_ign