/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "edge.h"
using std::string;


Edge::Edge()
: start_idx(0),
  end_idx(0),
  type(UNDEFINED),
  selected(false)
{
}

Edge::Edge(const int _start_idx, const int _end_idx, const Type _type)
: start_idx(_start_idx),
  end_idx(_end_idx),
  type(_type),
  selected(false)
{
  create_required_parameters();
}

Edge::~Edge()
{
}

void Edge::from_yaml(const YAML::Node& data, const Type edge_type)
{
  if (!data.IsSequence())
    throw std::runtime_error("Edge::from_yaml expected a sequence");
  start_idx = data[0].as<double>();
  end_idx = data[1].as<double>();
  type = edge_type;
  // load the parameters
  if (data.size() >= 2)
  {
    for (YAML::const_iterator it = data[2].begin(); it != data[2].end(); ++it)
    {
      Param p;
      p.from_yaml(it->second);
      params[it->first.as<string>()] = p;
    }
  }

  create_required_parameters();
}

YAML::Node Edge::to_yaml() const
{
  YAML::Node y;
  y.push_back(start_idx);
  y.push_back(end_idx);

  YAML::Node params_node(YAML::NodeType::Map);
  for (const auto& param : params)
    params_node[param.first] = param.second.to_yaml();
  y.push_back(params_node);
  y.SetStyle(YAML::EmitterStyle::Flow);
  return y;
}

bool Edge::is_bidirectional() const
{
  auto it = params.find("bidirectional");
  if (it == params.end() || it->second.type != Param::BOOL)
    return false;
  return it->second.value_bool;
}

void Edge::set_param(const std::string& name, const std::string& value)
{
  auto it = params.find(name);
  if (it == params.end())
  {
    printf("tried to set unknown parameter [%s]\n", name.c_str());
    return;  // unknown parameter
  }
  it->second.set(value);
}

template<typename T>
void Edge::create_param_if_needed(
  const std::string& name,
  const Param::Type& param_type,
  const T& param_value)
{
  auto it = params.find(name);
  if (it == params.end() || it->second.type != param_type)
    params[name] = param_value;
}

void Edge::create_required_parameters()
{
  // create required parameters if they don't exist yet on this edge
  if (type == MEAS)
  {
    auto it = params.find("distance");
    if (it == params.end() || it->second.type != Param::DOUBLE)
      params["distance"] = Param(1.0);
  }
  else if (type == WALL)
  {
    create_param_if_needed("texture_name", Param::STRING,
      std::string("default"));
    create_param_if_needed("alpha", Param::DOUBLE, 1.0);
    create_param_if_needed("texture_height", Param::DOUBLE, 2.5);
    create_param_if_needed("texture_width", Param::DOUBLE, 1.0);
    create_param_if_needed("texture_scale", Param::DOUBLE, 1.0);
  }
  else if (type == LANE)
  {
    create_param_if_needed("bidirectional", Param::BOOL, true);
    create_param_if_needed("orientation", Param::STRING, std::string());
    create_param_if_needed("graph_idx", Param::INT, 0);
    create_param_if_needed("demo_mock_floor_name", Param::STRING,
      std::string());
    create_param_if_needed("demo_mock_lift_name", Param::STRING, std::string());
    create_param_if_needed("speed_limit", Param::DOUBLE, 0.0);
    create_param_if_needed("mutex", Param::STRING, std::string());
  }
  else if (type == DOOR)
  {
    create_param_if_needed("name", Param::STRING, std::string("null")); // default door name
    create_param_if_needed("type", Param::STRING, std::string("hinged"));
    create_param_if_needed("motion_axis", Param::STRING, std::string("start"));
    create_param_if_needed("motion_direction", Param::INT, 1);
    create_param_if_needed("motion_degrees", Param::DOUBLE, 90.0);  // hinged
    create_param_if_needed("right_left_ratio", Param::DOUBLE, 1.0); // doubles
    create_param_if_needed("plugin", Param::STRING, std::string("normal"));
  }
  else if (type == HUMAN_LANE)
  {
    create_param_if_needed("width", Param::DOUBLE, 1.0);
    create_param_if_needed("bidirectional", Param::BOOL, true);
    create_param_if_needed("orientation", Param::STRING, std::string());
    create_param_if_needed("graph_idx", Param::INT, 9);
    create_param_if_needed("demo_mock_floor_name", Param::STRING,
      std::string());
    create_param_if_needed("demo_mock_lift_name", Param::STRING, std::string());
  }
}

std::string Edge::type_to_string() const
{
  if (type == LANE)
    return string("lane");
  else if (type == WALL)
    return string("wall");
  else if (type == MEAS)
    return string("measurement");
  else if (type == DOOR)
    return string("door");
  else if (type == HUMAN_LANE)
    return string("human_lane");
  return string("undefined");
}

QString Edge::type_to_qstring() const
{
  return QString::fromStdString(type_to_string());
}

void Edge::set_graph_idx(const int idx)
{
  if (type != LANE && type != HUMAN_LANE)
    return;// for now at least, only lanes have graph indices
  params["graph_idx"] = Param(idx);
}

int Edge::get_graph_idx() const
{
  if (type != LANE && type != HUMAN_LANE)
    return 0;// for now, only lanes have indices defined
  auto it = params.find("graph_idx");
  if (it == params.end() || it->second.type != Param::INT)
    return 0;// shouldn't get here
  return it->second.value_int;
}

double Edge::get_width() const
{
  if (type != HUMAN_LANE)
    return -1.0;
  auto it = params.find("width");
  if (it == params.end() || it->second.type != Param::DOUBLE)
    return -1.0;// shouldn't get here
  return it->second.value_double;
}
