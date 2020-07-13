#include <cmath>
#include <algorithm>

#include <building_sim_common/utils.hpp>

namespace building_gazebo_plugins {

//==============================================================================
double compute_ds(
  double s_target,
  double v_actual,
  const double v_max,
  const double accel_nom,
  const double accel_max,
  const double dt)
{
  double sign = 1.0;
  if (s_target < 0.0)
  {
    // Limits get confusing when we need to go backwards, so we'll flip signs
    // here so that we pretend the target is forwards
    s_target *= -1.0;
    v_actual *= -1.0;
    sign = -1.0;
  }

  // We should try not to shoot past the targstd::vector<event::ConnectionPtr> connections;et
  double next_s = s_target / dt;

  // Test velocity limit
  next_s = std::min(next_s, v_max);

  // Test acceleration limit
  next_s = std::min(next_s, accel_nom * dt + v_actual);

  if (v_actual > 0.0 && s_target > 0.0)
  {
    // This is what our deceleration should be if we want to begin a constant
    // deceleration from now until we reach the goal
    double deceleration = pow(v_actual, 2) / s_target;
    deceleration = std::min(deceleration, accel_max);

    if (accel_nom <= deceleration)
    {
      // If the smallest constant deceleration for reaching the goal is
      // greater than
      next_s = -deceleration * dt + v_actual;
    }
  }

  // Flip the sign the to correct direction before returning the value
  return sign * next_s;
}

//==============================================================================
double compute_desired_rate_of_change(
  double _s_target,
  double _v_actual,
  const MotionParams& _motion_params,
  const double _dt)
{
  double sign = 1.0;
  if (_s_target < 0.0)
  {
    // Limits get confusing when we need to go backwards, so we'll flip signs
    // here so that we pretend the target is forwards
    _s_target *= -1.0;
    _v_actual *= -1.0;
    sign = -1.0;
  }

  // We should try not to shoot past the target
  double v_next = _s_target / _dt;

  // Test velocity limit
  v_next = std::min(v_next, _motion_params.v_max);

  // Test acceleration limit
  v_next = std::min(v_next, _motion_params.a_nom * _dt + _v_actual);

  if (_v_actual > 0.0 && _s_target > 0.0)
  {
    // This is what our deceleration should be if we want to begin a constant
    // deceleration from now until we reach the goal
    double deceleration = pow(_v_actual, 2) / (2.0 * _s_target);
    deceleration = std::min(deceleration, _motion_params.a_max);

    if (_motion_params.a_nom <= deceleration)
    {
      // If the smallest constant deceleration for reaching the goal is
      // greater than the nominal acceleration, then we should begin
      // decelerating right away so that we can smoothly reach the goal while
      // decelerating as close to the nominal acceleration as possible.
      v_next = -deceleration * _dt + _v_actual;
    }
  }

  // Flip the sign to the correct direction before returning the value
  return sign * v_next;
}

//==============================================================================
bool get_element_required(
  const sdf::ElementPtr& _sdf,
  const std::string& _element_name,
  sdf::ElementPtr& _element)
{
  if (!_sdf->HasElement(_element_name))
  {
    std::cerr << "Element [" << _element_name << "] not found" << std::endl;
    return false;
  }
  _element = _sdf->GetElement(_element_name);
  return true;
}

} // namespace building_gazebo_plugins
