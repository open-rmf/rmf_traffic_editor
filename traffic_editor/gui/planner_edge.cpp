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

#include "planner_edge.h"
using planner::Edge;
using planner::Node;

Edge::Edge()
{
}

Edge::Edge(
    std::shared_ptr<Node> _start,
    std::shared_ptr<Node> _end)
: start(_start),
  end(_end)
{
}

bool Edge::operator==(const Edge& rhs)
{
  // lots of opportunity to make this go faster, if it's ever the bottleneck.

  const double thresh = 0.000001;  // just to deal with floating-point rounding

  const double sdx = rhs.start->x - start->x;
  const double sdy = rhs.start->y - start->y;
  const double sdist = sqrt(sdx * sdx + sdy * sdy);

  const double edx = rhs.end->x - end->x;
  const double edy = rhs.end->y - end->y;
  const double edist = sqrt(edx * edx + edy * edy);

  if (sdist > thresh || edist > thresh)
    return false;

  return true;
}
