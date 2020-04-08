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

#include "planner_graph.h"
using planner::Graph;
using std::make_shared;
using std::shared_ptr;

void Graph::add_edge(const Vertex& v_start, const Vertex& v_end)
{
  const int start_idx = node_idx(Node(v_start));
  const int end_idx = node_idx(Node(v_end));

  edges.push_back(make_shared<Edge>(nodes[start_idx], nodes[end_idx]));
}

int Graph::node_idx(const Node& n)
{
  // returns the current index of this node, adding it if needed
  for (size_t i = 0; i < nodes.size(); i++)
    if (nodes[i]->distance(n) < 0.0001)  // not sure of ideal tolerance
      return static_cast<int>(i);

  // if we get here, we need to add this node to our vector
  nodes.push_back(std::make_shared<Node>(n));
  return static_cast<int>(nodes.size() - 1);
}

void Graph::scale_all_nodes(const double scale_factor)
{
  for (auto& node : nodes)
  {
    node->x *= scale_factor;
    node->y *= scale_factor;
  }
}

void Graph::print() const
{
  printf("nodes:\n");
  for (const auto& node : nodes)
    printf("  (%.2f, %.2f)\n", node->x, node->y);

  printf("edges:\n");
  for (const auto& edge : edges)
    printf("  (%.2f, %.2f) -> (%.2f, %.2f)\n",
        edge->start->x,
        edge->start->y,
        edge->end->x,
        edge->end->y);
}

void Graph::plan_path(const Node& start, const Node& goal)
{
  printf("planning path from (%.2f, %.2f) -> (%.2f, %.2f)\n",
      start.x,
      start.y,
      goal.x,
      goal.y);
}
