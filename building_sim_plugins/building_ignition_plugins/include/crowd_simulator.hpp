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
     : _transport_node_ptr(std::make_shared<ignition::transport::Node>()),
     _crowd_sim_interface(std::make_shared<crowd_simulator::CrowdSimInterface>()),
     _initialized(false)
    {}

    // inherit from ISystemConfigure 
    void Configure(const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        ignition::gazebo::EntityComponentManager& ecm, ignition::gazebo::EventManager& eventMgr) override;

    // inherit from ISystemPreUpdate
    void PreUpdate(const ignition::gazebo::UpdateInfo& info, ignition::gazebo::EntityComponentManager& ecm) override;

private:
    std::shared_ptr<ignition::transport::Node> _transport_node_ptr;
    std::shared_ptr<crowd_simulator::CrowdSimInterface> _crowd_sim_interface;
    bool _initialized;
    std::chrono::steady_clock::duration _last_sim_time{0};
    std::chrono::steady_clock::duration _last_time{0};
    
    std::shared_ptr<ignition::gazebo::Model> _world;
    std::string _world_name;

    // map for <model_name, object_id>, contains both external (models) and internal agents (actors)
    std::unordered_map<std::string, size_t> _object_dic;
    // map for <model_name, entity_id> contains external and internal agents
    std::unordered_map<std::string, ignition::gazebo::Entity> _entity_dic;

    bool _spawn_agents_in_world(ignition::gazebo::EntityComponentManager& ecm);
    void _init_spawned_agents(ignition::gazebo::EntityComponentManager& ecm);
    void _config_spawned_agents(
        const crowd_simulator::CrowdSimInterface::ObjectPtr obj_ptr,
        const ignition::gazebo::Entity& enity, 
        ignition::gazebo::EntityComponentManager& ecm) const;
   
    bool _create_entity(
        ignition::gazebo::EntityComponentManager& ecm, 
      const std::string& model_name, 
      const crowd_simulator::ModelTypeDatabase::RecordPtr model_type_ptr) const;
    
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