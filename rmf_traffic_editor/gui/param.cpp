/*
 * Copyright (C) 2019-2021 Open Source Robotics Foundation
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

#include "param.h"
using std::string;


Param::Param()
: type(UNDEFINED), value_int(0), value_double(0.0), value_bool(false)
{
}

Param::Param(const Type& t)
: type(t), value_int(0), value_double(0.0), value_bool(false)
{
}

Param::Param(const std::string& s)
: type(STRING),
  value_int(0),
  value_double(0.0),
  value_string(s),
  value_bool(false)
{
}

Param::Param(const int& i)
: type(INT), value_int(i), value_double(0.0), value_bool(false)
{
}

Param::Param(const double& d)
: type(DOUBLE), value_int(0), value_double(d), value_bool(false)
{
}

Param::Param(const bool& b)
: type(BOOL), value_int(0), value_double(0.0), value_bool(b)
{
}

Param::~Param()
{
}

void Param::from_yaml(const YAML::Node& data)
{
  if (!data.IsSequence())
    throw std::runtime_error("Param::from_yaml expected a YAML sequence");
  type = static_cast<Type>(data[0].as<int>());
  if (type == STRING)
    value_string = data[1].as<string>();
  else if (type == INT)
    value_int = data[1].as<int>();
  else if (type == DOUBLE)
    value_double = data[1].as<double>();
  else if (type == BOOL)
    value_bool = data[1].as<bool>();
  else
    throw std::runtime_error("Param::from_yaml found an unknown type");
}

YAML::Node Param::to_yaml() const
{
  if (type == UNDEFINED)
    return YAML::Node();

  YAML::Node y;
  y.SetStyle(YAML::EmitterStyle::Flow);
  y.push_back(static_cast<int>(type));
  if (type == STRING)
    y.push_back(value_string);
  else if (type == INT)
    y.push_back(value_int);
  else if (type == DOUBLE)
    y.push_back(value_double);
  else if (type == BOOL)
    y.push_back(value_bool);
  else
    throw std::runtime_error("Param::to_yaml found an unknown type");
  return y;
}

void Param::set(const std::string& value)
{
  if (type == INT)
    value_int = stoi(value);
  else if (type == DOUBLE)
    value_double = stod(value);
  else if (type == STRING)
    value_string = value;
  else if (type == BOOL)
    value_bool = (value == "true") || (value == "True");
  else
    throw std::runtime_error("Param::set() found an unknown type");
}

QString Param::to_qstring() const
{
  if (type == DOUBLE)
    return QString::number(value_double);
  else if (type == BOOL)
    return value_bool ? QString("true") : QString("false");
  else if (type == STRING)
    return QString::fromStdString(value_string);
  else if (type == INT)
    return QString::number(value_int);
  else
    return QString("unknown type!");
}
