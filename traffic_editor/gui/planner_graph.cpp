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

#include <algorithm>
#include <vector>
#include "planner_graph.h"
using planner::Graph;
using std::make_shared;
using std::shared_ptr;
using std::vector;

void Graph::add_edge(const Vertex& v_start, const Vertex& v_end)
{
  const int start_idx = node_idx(Node(v_start));
  const int end_idx = node_idx(Node(v_end));

  edges.push_back(make_shared<Edge>(nodes[start_idx], nodes[end_idx]));

  nodes[start_idx]->neighbors.push_back(nodes[end_idx]);
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

std::shared_ptr<planner::Node> Graph::nearest_node(
    const double x,
    const double y)
{
  double min_dist = 1e9;
  std::shared_ptr<planner::Node> min_node;
  Node test_node;
  test_node.x = x;
  test_node.y = y;

  for (const auto& n : nodes)
  {
    const double dist = n->distance(test_node);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_node = n;
    }
  }

  return min_node;
}

vector<shared_ptr<planner::Node>> Graph::plan_path(
    const Node& start_pos,
    const Node& goal_pos)
{
  // the inbound start_pos might not be exactly on a planner node, so we have
  // to start by finding the nearest planner node on this graph

  std::shared_ptr<Node> start_node = nearest_node(start_pos.x, start_pos.y);
  std::shared_ptr<Node> goal_node = nearest_node(goal_pos.x, goal_pos.y);

  start_node->estimated_cost = start_node->distance(*goal_node);

#if 0
  printf("planning path from (%.2f, %.2f) -> (%.2f, %.2f)\n",
      start_pos.x,
      start_pos.y,
      goal_pos.x,
      goal_pos.y);

  printf("  start node: (%.2f, %.2f)\n", start_node->x, start_node->y);
  printf("    end node: (%.2f, %.2f)\n", goal_node->x, goal_node->y);
  printf("\n");
  start_node->print("  start:");
#endif

  vector<shared_ptr<Node>> unvisited_nodes;
  unvisited_nodes.push_back(start_node);

  bool found_goal = false;

  for (int num_iter = 0; num_iter < 100000; num_iter++)
  {
    // printf("A* iteration %d:\n", num_iter);
    double lowest_estimated_cost = 1e9;
    shared_ptr<Node> lowest_estimated_cost_node;

    for (const auto& unvisited_node : unvisited_nodes)
    {
      if (unvisited_node->estimated_cost < lowest_estimated_cost)
      {
        lowest_estimated_cost = unvisited_node->estimated_cost;
        lowest_estimated_cost_node = unvisited_node;
      }
    }

    lowest_estimated_cost_node->visited = true;
    // lowest_estimated_cost_node->print("  lowest_cost:");

    if (lowest_estimated_cost_node == goal_node)
    {
      found_goal = true;
      break;
    }

    unvisited_nodes.erase(
        std::remove(
            unvisited_nodes.begin(),
            unvisited_nodes.end(),
            lowest_estimated_cost_node),
        unvisited_nodes.end());

    for (const auto& neighbor : lowest_estimated_cost_node->neighbors)
    {
      if (neighbor->visited)
        continue;  // we already have the optimal path to this node

      const double cost =
          lowest_estimated_cost_node->actual_cost +
          neighbor->distance(*lowest_estimated_cost_node);
      if (std::find(
            unvisited_nodes.begin(),
            unvisited_nodes.end(),
            neighbor) == unvisited_nodes.end())
        unvisited_nodes.push_back(neighbor);
      else if (cost > neighbor->actual_cost)
        continue;  // this path to this node is worse than the existing one

      neighbor->parent = lowest_estimated_cost_node;
      neighbor->actual_cost = cost;
      neighbor->estimated_cost = cost + neighbor->distance(*goal_node);
    }

    if (unvisited_nodes.empty())
      break;
  }

  if (!found_goal)
  {
    printf("unable to find plan!\n");
    return vector<shared_ptr<Node>>();
  }
  
  // now go backwards from the goal 
  vector<shared_ptr<Node>> path;
  for (std::shared_ptr<Node> n = goal_node; n->parent; n = n->parent)
    path.push_back(n);
  std::reverse(path.begin(), path.end());

  /*
  printf("path has %d elements:\n", static_cast<int>(path.size()));
  for (const auto& n : path)
    n->print("  ");
  */

  return path;
}
