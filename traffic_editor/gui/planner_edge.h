/*
 * Copyright (C) 2019-2020 Open Source Robotics Foundation
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

#ifndef PLANNER_EDGE_H
#define PLANNER_EDGE_H

#include <memory>
#include "planner_node.h"

namespace planner {

class Edge
{
public:
  std::shared_ptr<Node> start;
  std::shared_ptr<Node> end;

  // in the future we may add things like speed limits, etc.
  Edge();
  Edge(std::shared_ptr<Node> _start, std::shared_ptr<Node> _end);

  bool operator==(const Edge& rhs);
};

}  // namespace planner
#endif
