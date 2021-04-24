/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef TRAFFIC_EDITOR__CONSTRAINT_HPP
#define TRAFFIC_EDITOR__CONSTRAINT_HPP

#include <string>
#include <vector>

#include <QUuid>
#include <yaml-cpp/yaml.h>

class QGraphicsScene;

class Level;

//=============================================================================
/// A constraint between two features in two maps that we want to align.

class Constraint
{
public:
  Constraint();
  Constraint(const QUuid& a, const QUuid& b);

  const std::vector<QUuid>& ids() const { return _ids; }
  void set_ids(std::vector<QUuid>& ids) { _ids = ids; }
  void add_id(const QUuid& id);
  bool includes_id(const QUuid& id) const;

  bool selected() const { return _selected; }
  void setSelected(const bool selected) { _selected = selected; }

  void from_yaml(const YAML::Node& data);
  YAML::Node to_yaml() const;

  bool operator==(const Constraint& other);

private:
  std::vector<QUuid> _ids;
  bool _selected = false;
};

#endif  // TRAFFIC_EDITOR__CONSTRAINT_HPP
