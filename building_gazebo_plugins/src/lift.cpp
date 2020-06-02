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
#include <queue>

// msgs
#include <rmf_lift_msgs/msg/lift_state.hpp>
#include <rmf_lift_msgs/msg/lift_request.hpp>
#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>

#include "utils.hpp"

namespace building_gazebo_plugins {

enum class LiftCabinState : uint8_t
{
  Stopped = 0,
  Ascending = 1,
  Descending = 2
};

enum class DoorStateEnum : uint8_t
{
  Closed = 0,
  Moving = 1,
  Open = 2
};

class Lift
{
  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  using DoorState = rmf_door_msgs::msg::DoorState;

private:
  gazebo::physics::ModelPtr _model;
  std::string _lift_name;
  bool _load_complete;

  std::string _cabin_joint_name = "cabin_joint";
  gazebo::physics::JointPtr _cabin_joint_ptr;
  MotionParams _cabin_motion_params;

  std::vector<std::string> _floor_names;
  std::unordered_map<std::string, double> _floor_name_to_elevation;
  std::unordered_map<std::string, std::vector<std::string>> _floor_name_to_shaft_door_name;
  std::unordered_map<std::string, std::vector<std::string>> _floor_name_to_cabin_door_name;
  std::unordered_map<std::string, DoorStateEnum> _shaft_door_states;
  std::unordered_map<std::string, DoorStateEnum> _cabin_door_states;

  std::string _current_floor_name;
  LiftCabinState _current_cabin_state;
  DoorStateEnum _current_door_state;
  std::pair<std::string, DoorStateEnum> _lift_request;
  std::queue<DoorRequest> _door_request_queue;

  bool _load_lift_parameters(const sdf::ElementPtr& sdf)
  {
    std::cout << "Loading lift name, cabin joint, cabin motion and door motion parameters: "
      << std::endl;
    // load lift name
    if (!get_sdf_param_required<std::string>(sdf, "lift_name", _lift_name))
      return false;

    // load main lift cabin joint
    sdf::ElementPtr cabin_element;
    if (!get_sdf_param_required<std::string>(sdf, "cabin_joint_name", _cabin_joint_name))
      return false;

    _cabin_joint_ptr = _model->GetJoint(_cabin_joint_name);
    if (!_cabin_joint_ptr)
    {
      gzerr << "couldn't find joint named: " << _cabin_joint_name << std::endl;
      return false;
    }

    // load lift cabin motion parameters
    get_sdf_param_if_available<double>(sdf, "v_max_cabin", _cabin_motion_params.v_max);
    get_sdf_param_if_available<double>(sdf, "a_max_cabin", _cabin_motion_params.a_max);
    get_sdf_param_if_available<double>(sdf, "a_nom_cabin", _cabin_motion_params.a_nom);
    get_sdf_param_if_available<double>(sdf, "dx_min_cabin", _cabin_motion_params.dx_min);
    get_sdf_param_if_available<double>(sdf, "f_max_cabin", _cabin_motion_params.f_max);

    return true;
  }

  bool _load_floor_parameters(const sdf::ElementPtr& sdf)
  {
    std::cout << "Loading lift floors' parameters: " << std::endl;
    // for each allocated floor, load the floor name and elevation
    sdf::ElementPtr floor_element;
    if (!get_element_required(sdf, "floor", floor_element))
      return false;
    while (floor_element)
    {
      std::string floor_name;
      double floor_elevation;
      if (!get_sdf_attribute_required<std::string>(floor_element, "name", floor_name) ||
          !get_sdf_attribute_required<double>(floor_element, "elevation", floor_elevation))
        return false;
      _floor_names.push_back(floor_name);
      _floor_name_to_elevation[floor_name] = floor_elevation;
      floor_element = floor_element->GetNextElement("floor");
    }
    return true;
  }

  bool _load_lift_door_names(const sdf::ElementPtr& sdf)
  {
    std::cout << "Loading all lift door names: " << std::endl;
    // for each allocated floor, load lift door names
    sdf::ElementPtr floor_element;
    if (!get_element_required(sdf, "floor", floor_element))
      return false;
    while (floor_element)
    {
      std::string floor_name;
      if (!get_sdf_attribute_required<std::string>(floor_element, "name", floor_name))
        return false;

      _floor_name_to_cabin_door_name[floor_name] = std::vector<std::string>();
      _floor_name_to_shaft_door_name[floor_name] = std::vector<std::string>();

      sdf::ElementPtr door_pair_element;
      if (!get_element_required(floor_element, "door_pair", door_pair_element))
        continue;
      while (door_pair_element)
      {
        std::string shaft_door_name;
        std::string cabin_door_name;
        if (!get_sdf_attribute_required<std::string>(door_pair_element, "cabin_door", cabin_door_name) ||
            !get_sdf_attribute_required<std::string>(door_pair_element, "shaft_door", shaft_door_name))
          return false;
        _floor_name_to_cabin_door_name[floor_name].push_back(cabin_door_name);
        _floor_name_to_shaft_door_name[floor_name].push_back(shaft_door_name);
        _cabin_door_states[cabin_door_name] = DoorStateEnum::Closed;
        _shaft_door_states[shaft_door_name] = DoorStateEnum::Closed;

        door_pair_element = door_pair_element->GetNextElement("door_pair");
      }
      floor_element = floor_element->GetNextElement("floor");
    }
    return true;
  }

  std::string _find_current_closest_floor_name()
  {
    double current_elevation = _cabin_joint_ptr->Position(0);
    double smallest_error = std::numeric_limits<double>::max();
    std::string closest_floor_name;
    for (const auto& floor : _floor_name_to_elevation)
    {
      double tmp_error = abs(current_elevation - floor.second);
      if (tmp_error < smallest_error)
      {
        smallest_error = tmp_error;
        closest_floor_name = floor.first;
      }
    }
    return closest_floor_name;
  }

  DoorRequest _build_door_request(std::string door_name, DoorStateEnum door_state)
  {
    DoorRequest request;
    request.requester_id = get_name();
    request.door_name = door_name;
    request.requested_mode.value = static_cast<uint32_t>(door_state);
    return request;
  }

public:
  Lift(const gazebo::physics::ModelPtr& model_ptr, const sdf::ElementPtr& sdf)
  {
    _model = model_ptr;

    // loading all relevant information for this lift unit
    if (!_load_lift_parameters(sdf) || !_load_floor_parameters(sdf) ||
      !_load_lift_door_names(sdf))
    {
      gzerr << "failed to load either lift parameters, floor parameters or lift shaft doors.\n";
      _load_complete = false;
      return;
    }

    // prints out available floors for this lift
    std::cout << "Loaded lift: " << _lift_name << std::endl;
    std::cout << "Names  |  Elevations" << std::endl;
    for (const auto& it : _floor_name_to_elevation)
    {
      std::cout << it.first << "  |  " << _floor_name_to_elevation[it.first] << std::endl;
    }
    std::cout << std::endl;

    // setting initial idle position
    std::string init_floor_name = _floor_names[0];
    get_sdf_param_if_available<std::string>(sdf, "default_floor", init_floor_name);
    _lift_request = std::make_pair(init_floor_name, DoorStateEnum::Closed);
    _current_cabin_state = LiftCabinState::Stopped;
    _load_complete = true;
  }

  bool load_successful()
  {
    return _load_complete;
  }

  void update(double dt)
  {
    update_cabin_state();

    std::string desired_floor_name = _lift_request.first;
    DoorStateEnum desired_door_state = _lift_request.second;

    // Lift control state machine
    if (_current_floor_name != desired_floor_name)
    {
      if (_current_door_state != DoorStateEnum::Closed)
      {
        close_doors();
        return;
      }
      move_cabin(dt);
      close_doors();
    }
    else
    {
      if (_current_cabin_state != LiftCabinState::Stopped)
      {
        move_cabin(dt);
        close_doors();
        return;
      }
      else
      {
        stop_cabin();
        if (desired_door_state == DoorStateEnum::Open)
        {
          open_doors();
        }
        else if (desired_door_state == DoorStateEnum::Closed)
        {
          close_doors();
        }
        return;
      }
    }
    return;
  }

  //===========================================================================
  void move_cabin(double dt)
  {
    double desired_elevation = _floor_name_to_elevation[_lift_request.first];
    double dz = desired_elevation - _cabin_joint_ptr->Position(0);
    if (abs(dz) < _cabin_motion_params.dx_min / 2.0)
    {
      dz = 0;
    }
    const double lift_step_v = compute_desired_rate_of_change(
          dz, _cabin_joint_ptr->GetVelocity(0), _cabin_motion_params, dt);

    _cabin_joint_ptr->SetParam("vel", 0, lift_step_v);
    _cabin_joint_ptr->SetParam("fmax", 0, _cabin_motion_params.f_max);
  }

  void stop_cabin()
  {
    _cabin_joint_ptr->SetParam("vel", 0, 0.0);
    _cabin_joint_ptr->SetParam("fmax", 0, _cabin_motion_params.f_max);
  }

  void update_cabin_state(){
    _current_floor_name = _find_current_closest_floor_name();

    double cabin_speed = _cabin_joint_ptr->GetVelocity(0);
    if (abs(cabin_speed) < 0.01)
    {
      _current_cabin_state = LiftCabinState::Stopped;
    }
    else if (cabin_speed > 0)
    {
      _current_cabin_state = LiftCabinState::Ascending;
    }
    else
    {
      _current_cabin_state = LiftCabinState::Descending;
    }
  }

  //===========================================================================
  // functions in this section need to be updated to support multiple shaft/cabin doors on one level
  void open_doors()
  {
    std::vector<std::string> cabin_door_names = _floor_name_to_cabin_door_name[_current_floor_name];
    for (auto& cabin_door : _cabin_door_states)
    {
      std::string name = cabin_door.first;
      DoorStateEnum current_state = cabin_door.second;
      if (std::find(cabin_door_names.begin(), cabin_door_names.end(), name) != cabin_door_names.end())
      {
        if (current_state != DoorStateEnum::Open)
        {
          _door_request_queue.push(_build_door_request(name, DoorStateEnum::Open));
        }
      }
      else
      {
        if (current_state != DoorStateEnum::Closed)
        {
          _door_request_queue.push(_build_door_request(name, DoorStateEnum::Closed));
        }
      }
    }
    std::vector<std::string> shaft_door_names = _floor_name_to_shaft_door_name[_current_floor_name];
    for (auto& shaft_door : _shaft_door_states)
    {
      std::string name = shaft_door.first;
      DoorStateEnum current_state = shaft_door.second;
      if (std::find(shaft_door_names.begin(), shaft_door_names.end(), name) != shaft_door_names.end())
      {
        if (current_state != DoorStateEnum::Open)
        {
          _door_request_queue.push(_build_door_request(name, DoorStateEnum::Open));
        }
      }
      else
      {
        if (current_state != DoorStateEnum::Closed)
        {
          _door_request_queue.push(_build_door_request(name, DoorStateEnum::Closed));
        }
      }
    }
  }

  void close_doors()
  {
    for (auto& cabin_door : _cabin_door_states)
    {
      if (cabin_door.second != DoorStateEnum::Closed)
      {
        _door_request_queue.push(_build_door_request(cabin_door.first, DoorStateEnum::Closed));
      }
    }
    for (auto& shaft_door : _shaft_door_states)
    {
      if (shaft_door.second != DoorStateEnum::Closed)
      {
        _door_request_queue.push(_build_door_request(shaft_door.first, DoorStateEnum::Closed));
      }
    }
  }

  DoorStateEnum get_cabin_door_state()
  {
    int open_count = 0; int closed_count = 0;
    int num = _floor_name_to_cabin_door_name[_current_floor_name].size();
    for (std::string cabin_door_name : _floor_name_to_cabin_door_name[_current_floor_name])
    {
      if (_cabin_door_states[cabin_door_name] == DoorStateEnum::Closed)
      {
        closed_count++;
      }
      else if (_cabin_door_states[cabin_door_name] == DoorStateEnum::Open)
      {
        open_count++;
      }
    }
    if (open_count == num)
    {
      return DoorStateEnum::Open;
    }
    else if (closed_count == num)
    {
      return DoorStateEnum::Closed;
    }
    return DoorStateEnum::Moving;
  }

  DoorStateEnum get_shaft_door_state()
  {
    int open_count = 0; int closed_count = 0;
    int num = _floor_name_to_shaft_door_name[_current_floor_name].size();
    for (std::string shaft_door_name : _floor_name_to_shaft_door_name[_current_floor_name])
    {
      if (_shaft_door_states[shaft_door_name] == DoorStateEnum::Closed)
      {
        closed_count++;
      }
      else if (_shaft_door_states[shaft_door_name] == DoorStateEnum::Open)
      {
        open_count++;
      }
    }
    if (open_count == num)
    {
      return DoorStateEnum::Open;
    }
    else if (closed_count == num)
    {
      return DoorStateEnum::Closed;
    }
    return DoorStateEnum::Moving;
  }

  void update_door_states(DoorState& door_state)
  {
    std::string name = door_state.door_name;
    DoorStateEnum state = static_cast<DoorStateEnum>(door_state.current_mode.value);
    if (_cabin_door_states.count(name) > 0)
    {
      _cabin_door_states[name] = state;
    }
    else if (_shaft_door_states.count(name) > 0)
    {
      _shaft_door_states[name] = state;
    }

    DoorStateEnum cabin_door_state = get_cabin_door_state();
    DoorStateEnum shaft_door_state = get_shaft_door_state();
    if ((cabin_door_state == DoorStateEnum::Open) && (shaft_door_state == DoorStateEnum::Open))
    {
      _current_door_state = DoorStateEnum::Open;
    }
    else if ((cabin_door_state == DoorStateEnum::Closed) && (shaft_door_state == DoorStateEnum::Closed))
    {
      _current_door_state = DoorStateEnum::Closed;
    }
    else
    {
      _current_door_state = DoorStateEnum::Moving;
    }
  }

  //===========================================================================
  void set_lift_request(const std::string& desired_floor, const uint8_t desired_door_state)
  {
    auto it = _floor_name_to_elevation.find(desired_floor);
    if (it == _floor_name_to_elevation.end())
    {
      std::cout << "Received invalid floor name: " << desired_floor << ", ignoring..." << std::endl;
      return;
    }
    _lift_request = std::make_pair(desired_floor, static_cast<DoorStateEnum>(desired_door_state));
  }

  bool get_door_request(DoorRequest& request)
  {
    if (_door_request_queue.empty())
    {
      return false;
    }
    request = _door_request_queue.front();
    _door_request_queue.pop();
    return true;
  }

  std::string get_current_floor()
  {
    return _current_floor_name;
  }

  std::string get_destination_floor()
  {
    return _lift_request.first;
  }

  LiftCabinState get_cabin_state()
  {
    return _current_cabin_state;
  }

  DoorStateEnum get_overall_door_state()
  {
    return _current_door_state;
  }

  std::string get_name()
  {
    return _lift_name;
  }

  std::vector<std::string> get_floor_names()
  {
    return _floor_names;
  }
};


class LiftPlugin : public gazebo::ModelPlugin
{
private:
  // Gazebo items
  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
  std::unique_ptr<Lift> _lift;

  // ROS items
  using LiftState = rmf_lift_msgs::msg::LiftState;
  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  using DoorState = rmf_door_msgs::msg::DoorState;
  using DoorMode = rmf_door_msgs::msg::DoorMode;
  gazebo_ros::Node::SharedPtr _ros_node;
  rclcpp::Publisher<LiftState>::SharedPtr _lift_state_pub;
  rclcpp::Publisher<DoorRequest>::SharedPtr _door_request_pub;
  rclcpp::Subscription<LiftRequest>::SharedPtr _lift_request_sub;
  rclcpp::Subscription<DoorState>::SharedPtr _door_state_sub;

  LiftState _lift_state;
  LiftRequest _lift_request;
  DoorState _door_state;
  DoorRequest _door_request;
  friend bool Lift::get_door_request(DoorRequest& request);

  // Runtime items
  bool _load_complete;
  double _last_update_time;
  double _last_pub_time;

public:
  LiftPlugin()
  : _last_update_time(0),
    _last_pub_time(0)
  {
    _load_complete = false;
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    _ros_node = gazebo_ros::Node::Get(sdf);
    _model = model;

    RCLCPP_INFO(
          _ros_node->get_logger(),
          "Loading LiftPlugin for [%s]",
          _model->GetName().c_str());

    // load Lift object
    _lift = std::make_unique<Lift>(_model, sdf);
    _load_complete = _lift->load_successful();

    // initialize pub & sub
    _lift_state_pub = _ros_node->create_publisher<LiftState>(
          "/lift_states", rclcpp::SystemDefaultsQoS());

    _door_request_pub = _ros_node->create_publisher<DoorRequest>(
          "/door_requests", rclcpp::SystemDefaultsQoS());

    _lift_request_sub = _ros_node->create_subscription<LiftRequest>(
          "/lift_requests", rclcpp::SystemDefaultsQoS(),
          [&](LiftRequest::UniquePtr msg)
    {
      if (msg->lift_name == _lift_state.lift_name)
        _lift_request = *msg;
    });

    _door_state_sub = _ros_node->create_subscription<DoorState>(
          "/door_states", rclcpp::SystemDefaultsQoS(),
          [&](DoorState::UniquePtr msg)
    {
      _door_state = *msg;
    });

    // initialize _state
    _lift_state.lift_name = model->GetName();
    _lift_state.available_floors.clear();
    for (const std::string& floor_name : _lift->get_floor_names())
    {
      _lift_state.available_floors.push_back(floor_name);
    }

    _last_update_time = _model->GetWorld()->SimTime().Double();
    
    // link to Gazebo world if everything is completely loaded
    if (!_load_complete)
    {
      RCLCPP_ERROR(
            _ros_node->get_logger(),
            "Failed when loading [%s]",
            _model->GetName().c_str());
    }
    else
    {
      _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&LiftPlugin::on_update, this));
    
      RCLCPP_INFO(
            _ros_node->get_logger(),
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
    const double dt = t - _last_update_time;
    _last_update_time = t;

    _lift->update_door_states(_door_state);

    if (_lift_request.lift_name == _lift->get_name())
    {
      _lift->set_lift_request(_lift_request.destination_floor, _lift_request.door_state);
    }

    if (_lift->get_door_request(_door_request))
    {
      _door_request.request_time = rclcpp::Time(t);
      _door_request_pub->publish(_door_request);
    }

    // at 1 Hz, publish lift state
    if (t - _last_pub_time > 1)
    {
      _last_pub_time = t;

      _lift_state.lift_time = rclcpp::Time(t);
      _lift_state.current_floor = _lift->get_current_floor();
      _lift_state.destination_floor = _lift->get_destination_floor();
      _lift_state.door_state = static_cast<uint8_t>(_lift->get_overall_door_state());
      _lift_state.motion_state = static_cast<uint8_t>(_lift->get_cabin_state());
      // TODO: add lift modes
      _lift_state.current_mode = 1;

      _lift_state_pub->publish(_lift_state);
    }

    // at 1000 Hz, update lift controllers
    _lift->update(dt);
  }

};

GZ_REGISTER_MODEL_PLUGIN(LiftPlugin)
} // namespace building_gazebo_plugins
