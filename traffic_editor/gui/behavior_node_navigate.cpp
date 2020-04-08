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

#include "behavior_node_navigate.h"
#include "building.h"

using std::string;
using std::shared_ptr;
using std::vector;


BehaviorNodeNavigate::BehaviorNodeNavigate(const YAML::Node& y)
: BehaviorNode()
{
  destination_name = y[1].as<string>();
}

BehaviorNodeNavigate::~BehaviorNodeNavigate()
{
}

void BehaviorNodeNavigate::print() const
{
  printf("      navigate: [%s]\n", destination_name.c_str());
}

std::unique_ptr<BehaviorNode> BehaviorNodeNavigate::clone() const
{
  return std::make_unique<BehaviorNodeNavigate>(*this);
}

void BehaviorNodeNavigate::tick(
    const double dt_seconds,
    ModelState& state,
    Building& building,
    const std::vector<std::unique_ptr<Model> >& /*active_models*/)
{
  // look up the scale of this level's drawing
  const double meters_per_pixel =
      building.level_meters_per_pixel(state.level_name);

  if (!destination_found)
  {
    destination_found = true;
    populate_model_state_from_vertex_name(
        destination_state,
        destination_name,
        building);

    const int graph_idx = 4;  // todo
    shared_ptr<planner::Graph> graph = building.planner_graph(
        graph_idx,
        state.level_name);
    graph->print();

    planner::Node goal_node;
    populate_planner_node_from_vertex_name(
        goal_node,
        destination_name,
        building);

    planner::Node start_node;
    start_node.x = state.x * meters_per_pixel;
    start_node.y = state.y * meters_per_pixel;

    path = graph->plan_path(start_node, goal_node);
    printf("\n");
  }

  if (path.empty())
    return;  // nothing to do

  const shared_ptr<const planner::Node> next_node = path.front();

  const double state_x_meters = state.x * meters_per_pixel;
  const double state_y_meters = state.y * meters_per_pixel;

  const double error_x = next_node->x - state_x_meters;
  const double error_y = next_node->y - state_y_meters;
  const double error_2d = sqrt(error_x * error_x + error_y * error_y);

  // for now assume we always rotate in place to face the next node, either
  // forwards or backwards
  const double bearing = -atan2(error_y, error_x);
  const double forwards_yaw_error = angle_difference(bearing, state.yaw);
  const double backwards_yaw_error =
      angle_difference(angle_sum(bearing, M_PI), state.yaw);
  double error_yaw = 0;
  if (fabs(forwards_yaw_error) < fabs(backwards_yaw_error))
    error_yaw = forwards_yaw_error;
  else
    error_yaw = backwards_yaw_error;
  const double max_yaw_rate = 1.0;
  double yaw_rate = max_yaw_rate;  // yaw speed controller later...
  if (error_yaw < 0)
    yaw_rate *= -1.0;
  state.yaw = angle_sum(state.yaw, dt_seconds * yaw_rate);

  const double max_speed = 0.5;  // SI units = meters/sec
  double speed = max_speed * ((M_PI_2 - fabs(error_yaw)) / M_PI_2);

  if (error_yaw == backwards_yaw_error)
    speed *= -1;

  if (error_2d < speed)
    path.erase(path.begin());  // todo: don't use vector...

  const double x_dot = speed * cos(state.yaw);
  const double y_dot = speed * sin(state.yaw);

  state.x += dt_seconds * x_dot / meters_per_pixel;
  state.y -= dt_seconds * y_dot / meters_per_pixel;

#if 0
  static int print_count = 0;
  if (print_count++ % 10 == 0)
    printf("  (%.2f, %.2f, %.2f)  err = (%.2f, %.2f, %.2f) |%.2f| spd = (%.2f, %.2f)\n",
        state_x_meters,
        state_y_meters,
        state.yaw,
        error_x,
        error_y,
        error_yaw,
        error_2d,
        speed,
        yaw_rate);
#endif

  prev_error = error_2d;
}

bool BehaviorNodeNavigate::is_complete() const
{
  return path.empty();
}
