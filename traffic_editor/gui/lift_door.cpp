#include "lift_door.h"

YAML::Node LiftDoor::to_yaml() const
{
  // This is in image space. I think it's safe to say nobody is clicking
  // with more than 1/1000 precision inside a single pixel.

  YAML::Node n;
  n["x"] = round(x * 1000.0) / 1000.0;
  n["y"] = round(y * 1000.0) / 1000.0;
  n["width"] = round(width * 1000.0) / 1000.0;
  n["door_type"] = static_cast<int>(door_type);
  // let's give yaw another decimal place because, I don't know, reasons (?)
  n["motion_axis_orientation"] =
      round(motion_axis_orientation * 10000.0) / 10000.0;
  return n;
}

void LiftDoor::from_yaml(const std::string& _name, const YAML::Node &data)
{
  if (!data.IsMap())
    throw std::runtime_error("LiftDoor::from_yaml() expected a map");
  x = data["x"].as<double>();
  y = data["y"].as<double>();
  width = data["width"].as<double>();
  door_type = static_cast<DoorType>(data["door_type"].as<int>());
  motion_axis_orientation = data["motion_axis_orientation"].as<double>();
  name = _name;
}
