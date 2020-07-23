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

#ifndef PARAM_H
#define PARAM_H

#include <string>

#include <yaml-cpp/yaml.h>
#include <QString>


class Param
{
public:
  enum Type
  {
    UNDEFINED = 0,
    STRING,
    INT,
    DOUBLE,
    BOOL
  } type;

  Param();
  ~Param();
  Param(const std::string& s);
  Param(const int& i);
  Param(const double& d);
  Param(const bool& b);
  Param(const Type& t);

  void from_yaml(const YAML::Node& data);
  YAML::Node to_yaml() const;

  int value_int;
  double value_double;
  std::string value_string;
  bool value_bool;

  void set(const std::string& value);

  QString to_qstring() const;
};

#endif
