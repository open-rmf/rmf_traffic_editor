#ifndef SRC__BUILDING_GAZEBO_PLUGINS__DOOR_HPP
#define SRC__BUILDING_GAZEBO_PLUGINS__DOOR_HPP

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>

#include "utils.hpp"

namespace building_gazebo_plugins {

class Door
{
public:
  bool _debuggable;

  Door(const bool debuggable,
       const gazebo::physics::JointPtr& joint,
       const MotionParams& params,
       const bool flip_direction = false);

  bool is_open() const;

  bool is_closed() const;

  void open(double dt);

  void close(double dt);

private:
  void _set_door_command(const double target, const double dt);

  gazebo::physics::JointPtr _joint;
  MotionParams _params;

  double _open_position;
  double _closed_position;
};

} // namespace building_gazebo_plugins

#endif // SRC__BUILDING_GAZEBO_PLUGINS__DOOR_HPP