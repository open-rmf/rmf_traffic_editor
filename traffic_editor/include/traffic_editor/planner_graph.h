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

#ifndef PLANNER_GRAPH_H
#define PLANNER_GRAPH_H

#include <memory>
#include <vector>

#include "planner_edge.h"
#include "planner_node.h"
#include "traffic_editor/vertex.h"

namespace planner {

class Graph
{
public:
  std::vector<std::shared_ptr<Node>> nodes;
  std::vector<std::shared_ptr<Edge>> edges;

  void add_edge(const Vertex& start, const Vertex& end);

  int node_idx(const Node& n);

  void scale_all_nodes(const double scale_factor);

  void print() const;

  std::vector<std::shared_ptr<Node>> plan_path(
      const Node& start,
      const Node& goal);

  std::shared_ptr<Node> nearest_node(const double x, const double y);
};

}  // namespace planner

#endif

