#ifndef SRC__BUILDING_GAZEBO_PLUGINS__LIFT_HPP
#define SRC__BUILDING_GAZEBO_PLUGINS__LIFT_HPP

// Gazebo
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>

// stl
#include <utility>
#include <unordered_map>

#include "utils.hpp"
#include "door.hpp"

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
private:
  gazebo::physics::ModelPtr _model;
  std::string _lift_name;
  bool _load_complete;

  std::string _cabin_joint_name = "cabin_joint";
  gazebo::physics::JointPtr _cabin_joint_ptr;
  MotionParams _cabin_motion_params;

  std::array<std::string, 2> _cabin_door_joint_names = {"left_door_joint", "right_door_joint"};
  std::vector<Door> _cabin_doors;
  MotionParams _cabin_door_motion_params;

  std::vector<std::string> _floor_names;
  std::unordered_map<std::string, double> _floor_name_to_elevation;
  std::unordered_map<std::string, std::vector<Door>> _lift_shaft_doors;

  std::string _current_floor_name;
  LiftCabinState _current_cabin_state;
  DoorStateEnum _current_door_state;
  std::pair<std::string, DoorStateEnum> _lift_request;

  bool _load_lift_parameters(const sdf::ElementPtr& sdf);

  bool _load_floor_parameters(const sdf::ElementPtr& sdf);

  bool _load_all_lift_shaft_doors(const sdf::ElementPtr& sdf);

  std::string _find_current_closest_floor_name();

public:
  Lift(const gazebo::physics::ModelPtr& model_ptr, const sdf::ElementPtr& sdf);

  bool load_successful();

  void update(double dt);

  //===========================================================================
  void move_cabin(double dt);

  void stop_cabin();

  void update_cabin_state(); // might change

  //===========================================================================
  void open_doors(double dt);

  void close_doors(double dt);

  DoorStateEnum get_cabin_door_state(); // might change

  DoorStateEnum get_shaft_door_state();

  void update_door_state();

  //===========================================================================
  void set_lift_request(const std::string& desired_floor, const DoorStateEnum desired_door_state);

  std::string get_current_floor();

  std::string get_destination_floor();

  LiftCabinState get_cabin_state(); // might change

  DoorStateEnum get_overall_door_state(); // might change

  std::string get_name();

  std::vector<std::string> get_floor_names();
};

} // namespace building_gazebo_plugins

#endif // SRC__BUILDING_GAZEBO_PLUGINS__LIFT_HPP