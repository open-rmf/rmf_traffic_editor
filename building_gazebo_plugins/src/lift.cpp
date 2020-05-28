#include "lift.hpp"

using namespace gazebo;

namespace building_gazebo_plugins {

const std::string DoubleSlidingDoorTypeString = "DoubleSlidingDoor";

Lift::Lift(const physics::ModelPtr& model_ptr, const sdf::ElementPtr& sdf)
{
  _model = model_ptr;

  // loading all relevant information for this lift unit
  if (!_load_lift_parameters(sdf) || !_load_floor_parameters(sdf) ||
    !_load_all_lift_shaft_doors(sdf))
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

bool Lift::load_successful()
{
  return _load_complete;
}

// private methods
bool Lift::_load_lift_parameters(const sdf::ElementPtr& sdf)
{
  std::cout << "Loading lift name, cabin joint, cabin motion and door motion parameters: "
    << std::endl;
  // load lift name
  if (!get_sdf_param_required<std::string>(sdf, "lift_name", _lift_name))
    return false;

  // load main lift cabin joint
  sdf::ElementPtr cabin_element;
  sdf::ElementPtr cabin_joint_element;
  if (!get_element_required(sdf, "cabin", cabin_element) ||
      !get_element_required(cabin_element, "cabin_joint", cabin_joint_element) ||
      !get_sdf_attribute_required<std::string>(cabin_joint_element, "name", _cabin_joint_name))
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

  // load all lift door motion parameters
  get_sdf_param_if_available<double>(sdf, "v_max_door", _cabin_door_motion_params.v_max);
  get_sdf_param_if_available<double>(sdf, "a_max_door", _cabin_door_motion_params.a_max);
  get_sdf_param_if_available<double>(sdf, "a_nom_door", _cabin_door_motion_params.a_nom);
  get_sdf_param_if_available<double>(sdf, "dx_min_door", _cabin_door_motion_params.dx_min);
  get_sdf_param_if_available<double>(sdf, "f_max_door", _cabin_door_motion_params.f_max);

  // load lift cabin doors
  sdf::ElementPtr cabin_door_element;
  std::string cabin_door_type;
  if (!get_element_required(cabin_element, "door", cabin_door_element) ||
      !get_sdf_attribute_required<std::string>(cabin_door_element, "type", cabin_door_type))
    return false;
  if (cabin_door_type != DoubleSlidingDoorTypeString && cabin_door_type != "double")
  {
    gzerr << "non-" << DoubleSlidingDoorTypeString << " type cabin doors [" << cabin_door_type
          << "] are currently not supported.\n";
    return false;
  }
  if (!get_sdf_attribute_required<std::string>(cabin_door_element, "left_joint_name", _cabin_door_joint_names[0]) ||
      !get_sdf_attribute_required<std::string>(cabin_door_element, "right_joint_name", _cabin_door_joint_names[1]))
    return false;
  _cabin_doors.clear();
  _cabin_doors.emplace_back(false, _model->GetJoint(_cabin_door_joint_names[0]), _cabin_door_motion_params, true);
  _cabin_doors.emplace_back(false, _model->GetJoint(_cabin_door_joint_names[1]), _cabin_door_motion_params);

  return true;
}

bool Lift::_load_floor_parameters(const sdf::ElementPtr& sdf)
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

bool Lift::_load_all_lift_shaft_doors(const sdf::ElementPtr& sdf)
{
  std::cout << "Loading all lift floors' shaft doors' parameters: " << std::endl;
  // for each allocated floor, load lift shaft doors
  _lift_shaft_doors.clear();
  physics::WorldPtr world = _model->GetWorld();
  sdf::ElementPtr floor_element;
  if (!get_element_required(sdf, "floor", floor_element))
    return false;
  while (floor_element)
  {
    std::string floor_name;
    sdf::ElementPtr shaft_door_element;
    std::string shaft_door_model_name;
    if (!get_sdf_attribute_required<std::string>(floor_element, "name", floor_name) ||
        !get_element_required(floor_element, "door", shaft_door_element) ||
        !get_sdf_attribute_required<std::string>(shaft_door_element, "name", shaft_door_model_name))
      return false;

    physics::ModelPtr shaft_door_model_ptr = world->ModelByName(shaft_door_model_name);
    if (!shaft_door_model_ptr)
    {
      gzerr << "couldn't find door model: " << shaft_door_model_name << std::endl;
      return false;
    }

    std::string shaft_door_type;
    if (!get_sdf_attribute_required<std::string>(shaft_door_element, "type", shaft_door_type))
      return false;
    if (shaft_door_type != DoubleSlidingDoorTypeString && shaft_door_type != "double")
    {
      gzerr << "non-" << DoubleSlidingDoorTypeString << " type cabin doors [" << shaft_door_type
            << "] are currently not supported.\n";
      return false;
    }
    std::string left_joint_name;
    std::string right_joint_name;
    if (!get_sdf_attribute_required<std::string>(shaft_door_element, "left_joint_name", left_joint_name) ||
        !get_sdf_attribute_required<std::string>(shaft_door_element, "right_joint_name", right_joint_name))
      return false;
    _lift_shaft_doors[floor_name] = std::vector<Door>();
    _lift_shaft_doors[floor_name].emplace_back(false, shaft_door_model_ptr->GetJoint(left_joint_name),
                                                 _cabin_door_motion_params, true);
    _lift_shaft_doors[floor_name].emplace_back(false, shaft_door_model_ptr->GetJoint(right_joint_name),
                                                 _cabin_door_motion_params);

    floor_element = floor_element->GetNextElement("floor");
  }
  return true;
}

std::string Lift::_find_current_closest_floor_name()
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

// public methods
void Lift::update(double dt)
{
  update_cabin_state();
  update_door_state();

  std::string desired_floor_name = _lift_request.first;
  DoorStateEnum desired_door_state = static_cast<DoorStateEnum>(_lift_request.second);

  // Lift control state machine
  if (_current_floor_name != desired_floor_name)
  {
    if (_current_door_state != DoorStateEnum::Closed)
    {
      close_doors(dt);
      return;
    }
    move_cabin(dt);
    close_doors(dt);
  }
  else
  {
    if (_current_cabin_state != LiftCabinState::Stopped)
    {
      move_cabin(dt);
      close_doors(dt);
      return;
    }
    else
    {
      stop_cabin();
      if (desired_door_state == DoorStateEnum::Open)
      {
        open_doors(dt);
      }
      else if (desired_door_state == DoorStateEnum::Closed)
      {
        close_doors(dt);
      }
      return;
    }
  }
  return;
}

void Lift::move_cabin(double dt)
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

void Lift::stop_cabin()
{
  _cabin_joint_ptr->SetParam("vel", 0, 0.0);
  _cabin_joint_ptr->SetParam("fmax", 0, _cabin_motion_params.f_max);
}

void Lift::update_cabin_state()
{
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
void Lift::open_doors(double dt)
{
  _cabin_doors[0].open(dt);
  _cabin_doors[1].open(dt);
  for (auto& shaft_door : _lift_shaft_doors)
  {
    if (shaft_door.first == _current_floor_name)
    {
      shaft_door.second[0].open(dt);
      shaft_door.second[1].open(dt);
    }
    else
    {
      shaft_door.second[0].close(dt);
      shaft_door.second[1].close(dt);
    }
  }
}

void Lift::close_doors(double dt)
{
  _cabin_doors[0].close(dt);
  _cabin_doors[1].close(dt);
  for (auto& shaft_door : _lift_shaft_doors)
  {
    shaft_door.second[0].close(dt);
    shaft_door.second[1].close(dt);
  }
}

DoorStateEnum Lift::get_cabin_door_state()
{
  if (_cabin_doors[0].is_open() && _cabin_doors[1].is_open())
  {
    return DoorStateEnum::Open;
  }
  else if (_cabin_doors[0].is_closed() && _cabin_doors[1].is_closed())
  {
    return DoorStateEnum::Closed;
  }
  return DoorStateEnum::Moving;
}

DoorStateEnum Lift::get_shaft_door_state()
{
  Door& left_door = _lift_shaft_doors[_current_floor_name][0];
  Door& right_door = _lift_shaft_doors[_current_floor_name][1];
  if (left_door.is_open() && right_door.is_open())
  {
    return DoorStateEnum::Open;
  }
  else if (left_door.is_closed() && right_door.is_closed())
  {
    return DoorStateEnum::Closed;
  }
  return DoorStateEnum::Moving;
}

void Lift::update_door_state() // AND THIS!!!
{
  DoorStateEnum cabin_door_state = get_cabin_door_state();
  DoorStateEnum shaft_door_state = get_shaft_door_state();
  if (cabin_door_state == DoorStateEnum::Open && shaft_door_state == DoorStateEnum::Open)
  {
    _current_door_state = DoorStateEnum::Open;
  }
  else if (cabin_door_state == DoorStateEnum::Closed && shaft_door_state == DoorStateEnum::Closed)
  {
    _current_door_state = DoorStateEnum::Closed;
  }
  else
  {
    _current_door_state = DoorStateEnum::Moving;
  }
}

//===========================================================================
void Lift::set_lift_request(const std::string& desired_floor, const DoorStateEnum desired_door_state)
{
  auto it = _floor_name_to_elevation.find(desired_floor);
  if (it == _floor_name_to_elevation.end())
  {
    std::cout << "Received invalid floor name: " << desired_floor << ", ignoring..." << std::endl;
    return;
  }
  _lift_request = std::make_pair(desired_floor, desired_door_state);
}

std::string Lift::get_current_floor()
{
  return _current_floor_name;
}

std::string Lift::get_destination_floor()
{
  return _lift_request.first;
}

LiftCabinState Lift::get_cabin_state() // might change
{
  return _current_cabin_state;
}

DoorStateEnum Lift::get_overall_door_state() // might change
{
  return _current_door_state;
}

std::string Lift::get_name()
{
  return _lift_name;
}

std::vector<std::string> Lift::get_floor_names()
{
  return _floor_names;
}

} // namespace building_gazebo_plugins