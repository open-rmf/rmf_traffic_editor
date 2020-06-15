// gazebo & ROS
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

// stl
#include <utility>
#include <unordered_map>

// msgs
#include <rmf_lift_msgs/msg/lift_state.hpp>
#include <rmf_lift_msgs/msg/lift_request.hpp>
#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>

#include "utils.hpp"

namespace building_gazebo_plugins {

class Lift
{
  using LiftState = rmf_lift_msgs::msg::LiftState;
  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  using DoorState = rmf_door_msgs::msg::DoorState;
  using DoorMode = rmf_door_msgs::msg::DoorMode;

private:
  std::string _lift_name;
  std::string _cabin_joint_name = "cabin_joint";
  MotionParams _cabin_motion_params;
  bool _load_complete;

  std::vector<std::string> _floor_names;
  std::unordered_map<std::string, double> _floor_name_to_elevation;
  std::unordered_map<std::string, std::vector<std::string>> _floor_name_to_shaft_door_name;
  std::unordered_map<std::string, std::vector<std::string>> _floor_name_to_cabin_door_name;
  std::unordered_map<std::string, DoorState::UniquePtr> _shaft_door_states;
  std::unordered_map<std::string, DoorState::UniquePtr> _cabin_door_states;

  rclcpp::Node::SharedPtr _ros_node;
  rclcpp::Publisher<LiftState>::SharedPtr _lift_state_pub;
  rclcpp::Publisher<DoorRequest>::SharedPtr _door_request_pub;
  rclcpp::Subscription<LiftRequest>::SharedPtr _lift_request_sub;
  rclcpp::Subscription<DoorState>::SharedPtr _door_state_sub;

  LiftState _lift_state;
  LiftRequest::UniquePtr _lift_request;

  double _last_update_time = 0.0;
  double _last_pub_time = 0.0;

  void publish_door_request(const double time, std::string door_name, uint32_t door_state)
  {
    DoorRequest request;
    request.request_time = rclcpp::Time(time);
    request.requester_id = _lift_name;
    request.door_name = door_name;
    request.requested_mode.value = door_state;
    
    _door_request_pub->publish(request);
  }

  Lift(const std::string& lift_name,
    rclcpp::Node::SharedPtr node,
    const MotionParams& cabin_motion_params,
    std::vector<std::string> floor_names,
    std::unordered_map<std::string, double> floor_name_to_elevation,
    std::unordered_map<std::string, std::vector<std::string>> floor_name_to_shaft_door_name,
    std::unordered_map<std::string, std::vector<std::string>> floor_name_to_cabin_door_name,
    std::vector<std::string> shaft_door_names,  // Changed from passing the hashmap directly to
    std::vector<std::string> cabin_door_names,  // passing vector of names to avoid UniquePtr issues
    std::string init_floor_name)
  : _lift_name(lift_name),
    _ros_node(node),
    _cabin_motion_params(cabin_motion_params),
    _floor_names(floor_names),
    _floor_name_to_elevation(floor_name_to_elevation),
    _floor_name_to_shaft_door_name(floor_name_to_shaft_door_name),
    _floor_name_to_cabin_door_name(floor_name_to_cabin_door_name)
  {
    for (std::string cabin_door_name : cabin_door_names)
      _cabin_door_states[cabin_door_name] = nullptr;
    for (std::string shaft_door_name : shaft_door_names)
      _shaft_door_states[shaft_door_name] = nullptr;

    // prints out available floors for this lift
    std::cout << "Loaded lift: " << _lift_name << std::endl;
    std::cout << "Names  |  Elevations" << std::endl;
    for (const auto& it : _floor_name_to_elevation)
    {
      std::cout << it.first << "  |  " << _floor_name_to_elevation[it.first] << std::endl;
    }
    std::cout << std::endl;

    // initialize pub & sub
    _lift_state_pub = _ros_node->create_publisher<LiftState>(
      "/lift_states", rclcpp::SystemDefaultsQoS());

    _door_request_pub = _ros_node->create_publisher<DoorRequest>(
      "/adapter_door_requests", rclcpp::SystemDefaultsQoS());

    _lift_request_sub = _ros_node->create_subscription<LiftRequest>(
      "/lift_requests", rclcpp::SystemDefaultsQoS(),
      [&](LiftRequest::UniquePtr msg)
    {
      std::cout << "Incoming lift request!" << std::endl;
      if (msg->lift_name == _lift_name)
      {
        if (_lift_request)    // Check if previous request is finished
        {
          RCLCPP_INFO(_ros_node->get_logger(),
            "Failed to request: [%s] is busy at the moment",
            _lift_name.c_str());
          return;
        }
        _lift_request = std::move(msg);
        std::cout << "Update lift request successfull!" << std::endl;
      }
    });

    _door_state_sub = _ros_node->create_subscription<DoorState>(
      "/door_states", rclcpp::SystemDefaultsQoS(),
      [&](DoorState::UniquePtr msg)
    {
      std::string name = msg->door_name;
      if (_cabin_door_states.count(name) > 0)
        _cabin_door_states[name] = std::move(msg);
      else if (_shaft_door_states.count(name) > 0)
        _shaft_door_states[name] = std::move(msg);
    });

    // Initial request to move lift to the initial floor
    _lift_request.reset(new LiftRequest);
    _lift_request->lift_name = _lift_name;
    _lift_request->destination_floor = init_floor_name;
    _lift_request->door_state = LiftRequest::DOOR_CLOSED;

    // Initial lift state
    _lift_state.lift_name = _lift_name;
    _lift_state.current_floor = _floor_names[0];
    _lift_state.destination_floor = init_floor_name;
    _lift_state.door_state = LiftState::DOOR_CLOSED;
    _lift_state.motion_state = LiftState::MOTION_STOPPED;
    _lift_state.available_floors.clear();
    for (const std::string& floor_name : _floor_names)
      _lift_state.available_floors.push_back(floor_name);
    
    _load_complete = true;
  }

public:
  struct LiftUpdateResult
  {
    double velocity;
    double fmax;
  };

  template<typename SdfPtrT>
  static std::shared_ptr<Lift> make(
    const std::string& lift_name,
    rclcpp::Node::SharedPtr node,
    SdfPtrT& sdf)
  {
    MotionParams cabin_motion_params;
    std::vector<std::string> floor_names;
    std::unordered_map<std::string, double> floor_name_to_elevation;
    std::unordered_map<std::string, std::vector<std::string>> floor_name_to_shaft_door_name;
    std::unordered_map<std::string, std::vector<std::string>> floor_name_to_cabin_door_name;
    std::vector<std::string> shaft_door_names;
    std::vector<std::string> cabin_door_names;

    auto sdf_clone = sdf->Clone();

    std::cout << "Loading lift cabin motion parameters: "
      << std::endl;

    // load lift cabin motion parameters
    get_sdf_param_if_available<double>(sdf_clone, "v_max_cabin", cabin_motion_params.v_max);
    get_sdf_param_if_available<double>(sdf_clone, "a_max_cabin", cabin_motion_params.a_max);
    get_sdf_param_if_available<double>(sdf_clone, "a_nom_cabin", cabin_motion_params.a_nom);
    get_sdf_param_if_available<double>(sdf_clone, "dx_min_cabin", cabin_motion_params.dx_min);
    get_sdf_param_if_available<double>(sdf_clone, "f_max_cabin", cabin_motion_params.f_max);

    std::cout << "Loading lift floors' parameters: " << std::endl;
    // for each allocated floor, load the floor name and elevation
    sdf::ElementPtr floor_element;
    if (!get_element_required(sdf, "floor", floor_element))
    {
      RCLCPP_ERROR(node->get_logger(),
        " -- Missing required floor element for [%s] plugin",
        lift_name.c_str());
      return nullptr;
    }

    while (floor_element)
    {
      std::string floor_name;
      double floor_elevation;
      if (!get_sdf_attribute_required<std::string>(floor_element, "name", floor_name) ||
          !get_sdf_attribute_required<double>(floor_element, "elevation", floor_elevation))
      {
        RCLCPP_ERROR(node->get_logger(),
          " -- Missing required floor name or elevation attributes for [%s] plugin",
          lift_name.c_str());
        return nullptr;
      }
      floor_names.push_back(floor_name);
      floor_name_to_elevation[floor_name] = floor_elevation;

      floor_name_to_cabin_door_name[floor_name] = std::vector<std::string>();
      floor_name_to_shaft_door_name[floor_name] = std::vector<std::string>();

      sdf::ElementPtr door_pair_element;
      if (!get_element_required(floor_element, "door_pair", door_pair_element))
        continue;
      while (door_pair_element)
      {
        std::string shaft_door_name;
        std::string cabin_door_name;
        if (!get_sdf_attribute_required<std::string>(door_pair_element, "cabin_door", cabin_door_name) ||
            !get_sdf_attribute_required<std::string>(door_pair_element, "shaft_door", shaft_door_name))
        {
          RCLCPP_ERROR(node->get_logger(),
            " -- Missing required lift door attributes for [%s] plugin",
            lift_name.c_str());
          return nullptr;
        }
        floor_name_to_cabin_door_name[floor_name].push_back(cabin_door_name);
        floor_name_to_shaft_door_name[floor_name].push_back(shaft_door_name);
        if (std::count(cabin_door_names.begin(), cabin_door_names.end(), cabin_door_name))
          cabin_door_names.push_back(cabin_door_name);
        if (std::count(shaft_door_names.begin(), shaft_door_names.end(), shaft_door_name))
          shaft_door_names.push_back(shaft_door_name);

        door_pair_element = door_pair_element->GetNextElement("door_pair");
      }
      floor_element = floor_element->GetNextElement("floor");
    }

    std::string init_floor_name = floor_names[0];
    get_sdf_param_if_available<std::string>(sdf_clone, "reference_floor", init_floor_name);

    std::shared_ptr<Lift> lift(new Lift(
      lift_name,
      node,
      cabin_motion_params,
      floor_names,
      floor_name_to_elevation,
      floor_name_to_shaft_door_name,
      floor_name_to_cabin_door_name,
      cabin_door_names,
      shaft_door_names,
      init_floor_name));

    return lift;
  }

  LiftUpdateResult update(const double time, const double position, const double velocity)
  {
    const double dt = time - _last_update_time;
    _last_update_time = time;
    
    // Update lift state
    update_cabin_state(position, velocity);
    update_door_state();

    // Construct LiftUpdateResult
    LiftUpdateResult result;
    result.velocity = 0.0;
    result.fmax = _cabin_motion_params.f_max;

    // Handle lift request
    if (_lift_request)
    {
      std::string desired_floor = _lift_request->destination_floor;
      uint8_t desired_door_state = _lift_request->door_state;
      auto it = _floor_name_to_elevation.find(desired_floor);
      if (it == _floor_name_to_elevation.end())
      {
        std::cout << "Received invalid floor name: " << desired_floor
          << ", ignoring..." << std::endl;
        _lift_request = nullptr;
      }
      else if ((_lift_state.current_floor == desired_floor) &&
        (_lift_state.door_state == desired_door_state) &&
        (_lift_state.motion_state == LiftState::MOTION_STOPPED))
      {
        std::cout << "Reached floor: " << desired_floor << " with doors " <<
          (desired_door_state==0 ? "closed" : "open") << std::endl;
        _lift_request = nullptr;
      }
      else
      {
        // Update lift state
        _lift_state.destination_floor = desired_floor;

        // State machine
        if (_lift_state.current_floor != _lift_state.destination_floor)
        {
          if (_lift_state.door_state != LiftState::DOOR_CLOSED)
          {
            close_doors(time);
            if (_lift_state.motion_state != LiftState::MOTION_STOPPED)
            {
              result.velocity = get_step_velocity(dt, position, velocity);
            }
          }
          else
          {
            result.velocity = get_step_velocity(dt, position, velocity);
          }
        }
        else
        {
          if (_lift_state.motion_state != LiftState::MOTION_STOPPED)
          {
            result.velocity = get_step_velocity(dt, position, velocity);
          }
          else
          {
            if (desired_door_state == LiftState::DOOR_OPEN)
            {;
              open_doors(time);
            }
            else if (desired_door_state == LiftState::DOOR_CLOSED)
            {
              close_doors(time);
            }
          }
        }
      }
    }

    // Publish lift state at 1 Hz
    if (time - _last_pub_time >= 1.0)
    {
      _last_pub_time = time;
      _lift_state.lift_time = rclcpp::Time(time);
      _lift_state_pub->publish(_lift_state);
    }

    return result;
  }

  //===========================================================================
  double get_step_velocity(const double dt, const double position, const double velocity)
  {
    double desired_elevation = _floor_name_to_elevation[_lift_state.destination_floor];
    double dz = desired_elevation - position;

    if (abs(dz) < _cabin_motion_params.dx_min / 2.0)
      dz = 0;

    return compute_desired_rate_of_change(
      dz, velocity, _cabin_motion_params, dt);
  }

  void update_cabin_state(const double position, const double velocity)
  {
    // Find closest floor and set to current floor
    double smallest_error = std::numeric_limits<double>::max();
    std::string closest_floor_name;
    for (const auto& floor : _floor_name_to_elevation)
    {
      double tmp_error = abs(position - floor.second);
      if (tmp_error < smallest_error)
      {
        smallest_error = tmp_error;
        closest_floor_name = floor.first;
      }
    }
    _lift_state.current_floor = closest_floor_name;

    // Set motion state
    if (abs(velocity) < 0.01)
      _lift_state.motion_state = LiftState::MOTION_STOPPED;
    else if (velocity > 0)
      _lift_state.motion_state = LiftState::MOTION_UP;
    else
      _lift_state.motion_state = LiftState::MOTION_DOWN;
  }

  //===========================================================================
  void open_doors(const double time)
  {
    auto cabin_door_names = _floor_name_to_cabin_door_name[_lift_state.current_floor];
    for (auto& cabin_door : _cabin_door_states)
    {
      std::string name = cabin_door.first;
      if (std::find(cabin_door_names.begin(), cabin_door_names.end(), name) != cabin_door_names.end())
      {
        if ((!cabin_door.second) ||
          (cabin_door.second->current_mode.value != DoorMode::MODE_OPEN))
          publish_door_request(time, name, DoorMode::MODE_OPEN);
      }
      else
      {
        if ((!cabin_door.second) ||
          (cabin_door.second->current_mode.value != DoorMode::MODE_CLOSED))
          publish_door_request(time, name, DoorMode::MODE_CLOSED);
      }
    }
    auto shaft_door_names = _floor_name_to_shaft_door_name[_lift_state.current_floor];
    for (auto& shaft_door : _shaft_door_states)
    {
      std::string name = shaft_door.first;
      if (std::find(shaft_door_names.begin(), shaft_door_names.end(), name) != shaft_door_names.end())
      {
        if ((!shaft_door.second) ||
          (shaft_door.second->current_mode.value != DoorMode::MODE_OPEN))
          publish_door_request(time, name, DoorMode::MODE_OPEN);
      }
      else
      {
        if ((!shaft_door.second) ||
          (shaft_door.second->current_mode.value != DoorMode::MODE_CLOSED))
          publish_door_request(time, name, DoorMode::MODE_CLOSED);
      }
    }
  }

  void close_doors(const double time)
  {
    for (auto& cabin_door : _cabin_door_states)
    {
      if ((!cabin_door.second) ||
        (cabin_door.second->current_mode.value != DoorMode::MODE_CLOSED))
        publish_door_request(time, cabin_door.first, DoorMode::MODE_CLOSED);
    }
    for (auto& shaft_door : _shaft_door_states)
    {
      if ((!shaft_door.second) ||
        (shaft_door.second->current_mode.value != DoorMode::MODE_CLOSED))
        publish_door_request(time, shaft_door.first, DoorMode::MODE_CLOSED);
    }
  }

  uint32_t get_cabin_door_state()
  {
    int open_count = 0; int closed_count = 0;
    int num = _floor_name_to_cabin_door_name[_lift_state.current_floor].size();
    for (std::string cabin_door : _floor_name_to_cabin_door_name[_lift_state.current_floor])
    {
      if ((_cabin_door_states[cabin_door]) &&
        (_cabin_door_states[cabin_door]->current_mode.value == DoorMode::MODE_CLOSED))
        closed_count++;

      else if ((_cabin_door_states[cabin_door]) &&
        (_cabin_door_states[cabin_door]->current_mode.value == DoorMode::MODE_OPEN))
        open_count++;
    }

    if (open_count == num)
      return DoorMode::MODE_OPEN;

    else if (closed_count == num)
      return DoorMode::MODE_CLOSED;

    else
      return DoorMode::MODE_MOVING;
  }

  uint32_t get_shaft_door_state()
  {
    int open_count = 0; int closed_count = 0;
    int num = _floor_name_to_shaft_door_name[_lift_state.current_floor].size();
    for (std::string shaft_door : _floor_name_to_shaft_door_name[_lift_state.current_floor])
    {
      if ((_shaft_door_states[shaft_door]) &&
        (_shaft_door_states[shaft_door]->current_mode.value == DoorMode::MODE_CLOSED))
        closed_count++;
      
      else if ((_shaft_door_states[shaft_door]) &&
        (_shaft_door_states[shaft_door]->current_mode.value == DoorMode::MODE_OPEN))
        open_count++;
    }

    if (open_count == num)
      return DoorMode::MODE_OPEN;

    else if (closed_count == num)
      return DoorMode::MODE_CLOSED;

    else
      return DoorMode::MODE_MOVING;
  }

  void update_door_state()
  {
    uint32_t cabin_door_state = get_cabin_door_state();
    uint32_t shaft_door_state = get_shaft_door_state();

    if ((cabin_door_state == DoorMode::MODE_OPEN) && (shaft_door_state == DoorMode::MODE_OPEN))
      _lift_state.door_state = LiftState::DOOR_OPEN;

    else if ((cabin_door_state == DoorMode::MODE_CLOSED) && (shaft_door_state == DoorMode::MODE_CLOSED))
      _lift_state.door_state = LiftState::DOOR_CLOSED;

    else
      _lift_state.door_state = LiftState::DOOR_MOVING;
  }

  //===========================================================================
  std::string get_joint_name()
  {
    return _cabin_joint_name;
  }

  bool load_complete()
  {
    return _load_complete;
  }
};


class LiftPlugin : public gazebo::ModelPlugin
{
private:
  // Gazebo items
  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
  gazebo::physics::JointPtr _cabin_joint_ptr;
  gazebo_ros::Node::SharedPtr _ros_node;
  
  std::shared_ptr<Lift> _lift;
  
  bool _load_complete;

public:
  LiftPlugin()
  {
    _load_complete = false;
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    _ros_node = gazebo_ros::Node::Get(sdf);
    _model = model;

    RCLCPP_INFO(_ros_node->get_logger(),
      "Loading LiftPlugin for [%s]",
      _model->GetName().c_str());

    // load Lift object
    _lift = Lift::make(_model->GetName(), _ros_node, sdf);
    if (!_lift)
      return;

    _cabin_joint_ptr = _model->GetJoint(_lift->get_joint_name());
    _load_complete = _lift->load_complete();

    // link to Gazebo world if everything is completely loaded
    if (!_load_complete)
    {
      RCLCPP_ERROR(_ros_node->get_logger(),
        "Failed when loading [%s]",
        _model->GetName().c_str());
    }
    else
    {
      _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&LiftPlugin::on_update, this));

      RCLCPP_INFO(_ros_node->get_logger(),
        "Finished loading [%s]",
        _model->GetName().c_str());
    }
  }

private:
  void on_update()
  {
    if (!_load_complete)
      return;
    
    const double t = _model->GetWorld()->SimTime().Double();
    const double position = _cabin_joint_ptr->Position(0);
    const double velocity = _cabin_joint_ptr->GetVelocity(0);

    // Send update request
    auto result = _lift->update(t, position, velocity);

    _cabin_joint_ptr->SetParam("vel", 0, result.velocity);
    _cabin_joint_ptr->SetParam("fmax", 0, result.fmax);
  }
};

GZ_REGISTER_MODEL_PLUGIN(LiftPlugin)
} // namespace building_gazebo_plugins
