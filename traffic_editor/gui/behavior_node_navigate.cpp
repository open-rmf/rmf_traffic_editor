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

#include "traffic_editor/behavior_node_navigate.h"
#include "traffic_editor/building.h"

using std::string;
using std::shared_ptr;
using std::vector;
using std::make_unique;
using std::make_shared;


BehaviorNodeNavigate::BehaviorNodeNavigate(const YAML::Node& y)
: BehaviorNode()
{
  destination_name = y[1].as<string>();
  nav_graph_idx = y[2].as<int>();
}

BehaviorNodeNavigate::~BehaviorNodeNavigate()
{
}

void BehaviorNodeNavigate::print() const
{
  printf("      navigate: [%s]\n", destination_name.c_str());
}

std::unique_ptr<BehaviorNode> BehaviorNodeNavigate::instantiate(
    const YAML::Node& params,
    const std::string& _model_name) const
{
  auto b = make_unique<BehaviorNodeNavigate>(*this);
  b->destination_name = interpolate_string_params(destination_name, params);
  b->model_name = _model_name;
  return b;
}

void BehaviorNodeNavigate::tick(
    const double dt_seconds,
    ModelState& model_state,
    Building& building,
    const std::vector<std::unique_ptr<Model> >& /*active_models*/,
    const std::vector<std::string>& /*inbound_signals*/,
    std::vector<std::string>& /*outbound_signals*/)
{
  // look up the scale of this level's drawing
  const double meters_per_pixel =
      building.level_meters_per_pixel(model_state.level_name);

  if (!destination_found)
  {
    printf(
        "planning path to [%s] on level [%s]\n",
        destination_name.c_str(),
        model_state.level_name.c_str());

    destination_found = true;
    populate_model_state_from_vertex_name(
        destination_state,
        destination_name,
        building);

    shared_ptr<planner::Graph> graph = building.planner_graph(
        nav_graph_idx,
        model_state.level_name);
    // graph->print();

    planner::Node goal_node;
    populate_planner_node_from_vertex_name(
        goal_node,
        destination_name,
        building);

    planner::Node start_node;
    start_node.x = model_state.x * meters_per_pixel;
    start_node.y = model_state.y * meters_per_pixel;

    path = graph->plan_path(start_node, goal_node);
    // printf("\n");
  }

  if (path.empty())
    return;  // nothing to do

  const shared_ptr<planner::Node> next_node = path.front();

  const double state_x_meters = model_state.x * meters_per_pixel;
  const double state_y_meters = model_state.y * meters_per_pixel;

  if (controller_state == ControllerState::AWAITING_LANE)
  {
    if (!previous_node)  // happens the first tick of this behavior node
    {
      previous_node = make_shared<planner::Node>();
      previous_node->x = state_x_meters;
      previous_node->y = state_y_meters;
    }

    planner::Edge next_edge(previous_node, next_node);
    if (building.request_lane_edge(
          model_state.level_name,
          next_edge,
          model_name,
          is_first_motion))
    {
      building.release_lane_edge(
          model_state.level_name,
          current_edge,
          model_name);

      controller_state = ControllerState::NAVIGATING;
      is_first_motion = false;
      current_edge = next_edge;
    }
    else
      return;  // wait until next tick to request it again
  }

  const double error_x = next_node->x - state_x_meters;
  const double error_y = next_node->y - state_y_meters;
  const double error_2d = sqrt(error_x * error_x + error_y * error_y);

  // for now assume we always rotate in place to face the next node, either
  // forwards or backwards
  const double bearing = -atan2(error_y, error_x);
  const double forwards_yaw_error = angle_difference(bearing, model_state.yaw);
  const double backwards_yaw_error =
      angle_difference(angle_sum(bearing, M_PI), model_state.yaw);
  double error_yaw = 0;
  if (fabs(forwards_yaw_error) < fabs(backwards_yaw_error))
    error_yaw = forwards_yaw_error;
  else
    error_yaw = backwards_yaw_error;
  const double max_yaw_rate = 1.0;
  double yaw_rate = max_yaw_rate;  // yaw speed controller later...
  if (error_yaw < 0)
    yaw_rate *= -1.0;
  model_state.yaw = angle_sum(model_state.yaw, dt_seconds * yaw_rate);

  const double max_speed = 0.5;  // SI units = meters/sec
  double speed = max_speed * ((M_PI_2 - fabs(error_yaw)) / M_PI_2);

#if 0
  // reduce speed as needed to avoid rear-ender collisions
  double dist_to_next_model = building.distance_to_nearest_model_on_path(
      model_name,
      model_state,
      path);
  printf(
      "%s nearest model on path: %.2f\n",
      model_name.c_str(),
      dist_to_next_model);
#endif

  if (error_yaw == backwards_yaw_error)
    speed *= -1;

  if (error_2d < dt_seconds * speed)
  {
    // teleport fully to the next node, so that lane reservations work
    model_state.x = next_node->x / meters_per_pixel;
    model_state.y = next_node->y / meters_per_pixel;

    previous_node = next_node;
    controller_state = ControllerState::AWAITING_LANE;
    path.erase(path.begin());  // todo: don't use vector...
  }

  const double x_dot = speed * cos(model_state.yaw);
  const double y_dot = speed * sin(model_state.yaw);

  model_state.x += dt_seconds * x_dot / meters_per_pixel;
  model_state.y -= dt_seconds * y_dot / meters_per_pixel;

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
