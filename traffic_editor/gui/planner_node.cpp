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

#include "planner_node.h"
using planner::Node;

Node::Node()
{
}

Node::Node(const Vertex& v)
: x(v.x),
  y(v.y)
{
}

double Node::distance(const Node& n) const
{
  const double dx = n.x - x;
  const double dy = n.y - y;
  return sqrt(dx * dx + dy * dy);
}

void Node::print(const std::string& prefix) const
{
  printf("%s(%.2f, %.2f) actual: %.2f  estimated: %.2f\n",
      prefix.c_str(),
      x,
      y,
      actual_cost,
      estimated_cost);
}

bool Node::operator==(const Node& rhs)
{
  // lots of opportunity to make this go faster, if it's ever the bottleneck.

  const double dx = rhs.x - x;
  const double dy = rhs.y - y;
  const double dist = sqrt(dx * dx + dy * dy);

  const double thresh = 0.000001;  // just to deal with floating-point rounding
  return dist < thresh;
}
