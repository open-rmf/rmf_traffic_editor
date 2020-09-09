#include <traffic_editor/crowd_sim/model_type.h>

using namespace crowd_sim;

//==============================================
YAML::Node ModelType::to_yaml() const
{
  YAML::Node model_node = YAML::Node(YAML::NodeType::Map);
  model_node.SetStyle(YAML::EmitterStyle::Flow);
  model_node["typename"] = get_name();
  model_node["animation"] = get_animation();
  model_node["animation_speed"] = get_animation_speed();
  model_node["gazebo"] = _gazebo_conf.to_yaml();
  model_node["ign"] = _ign_conf.to_yaml();
  return model_node;
}

//==============================================
void ModelType::from_yaml(const YAML::Node& input)
{
  _name = input["typename"].as<std::string>();
  _animation = input["animation"].as<std::string>();
  _animation_speed = input["animation_speed"].as<double>();
  _gazebo_conf.from_yaml(input["gazebo"]);
  _ign_conf.from_yaml(input["ign"]);
}

//==============================================
YAML::Node ModelType::GazeboConf::to_yaml() const
{
  YAML::Node result = YAML::Node(YAML::NodeType::Map);
  result.SetStyle(YAML::EmitterStyle::Flow);
  result["filename"] = filename;
  result["pose"] = YAML::Node(YAML::NodeType::Sequence);
  result["pose"] = initial_pose;
  return result;
}

//==============================================
void ModelType::GazeboConf::from_yaml(const YAML::Node& input)
{
  filename = input["filename"].as<std::string>();
  const YAML::Node& pose_node = input["pose"];
  size_t i = 0;
  for (YAML::const_iterator it = pose_node.begin();
    it != pose_node.end() && i < initial_pose.size();
    it++)
  {
    initial_pose[i++] = (*it).as<double>();
  }
}

//==============================================
YAML::Node ModelType::IgnConf::to_yaml() const
{
  YAML::Node result = YAML::Node(YAML::NodeType::Map);
  result.SetStyle(YAML::EmitterStyle::Flow);
  result["model_file_path"] = filename;
  result["pose"] = YAML::Node(YAML::NodeType::Sequence);
  result["pose"] = initial_pose;
  return result;
}

//==============================================
void ModelType::IgnConf::from_yaml(const YAML::Node& input)
{
  filename = input["model_file_path"].as<std::string>();
  const YAML::Node& pose_node = input["pose"];
  size_t i = 0;
  for (YAML::const_iterator it = pose_node.begin();
    it != pose_node.end() && i < initial_pose.size();
    it++)
  {
    initial_pose[i++] = (*it).as<double>();
  }
}
