#ifndef SRC__BUILDING_SIM_COMMON__UTILS_HPP
#define SRC__BUILDING_SIM_COMMON__UTILS_HPP

#include <cmath>
#include <iostream>

namespace building_sim_common {

// TODO(MXG): Refactor the use of this function to replace it with
// compute_desired_rate_of_change()
double compute_ds(
  double s_target,
  double v_actual,
  double v_max,
  double accel_nom,
  double accel_max,
  double dt);

struct MotionParams
{
  double v_max = 0.2;
  double a_max = 0.1;
  double a_nom = 0.08;
  double dx_min = 0.01;
  double f_max = 10000000.0;
};

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

  // Flip the sign the to correct direction before returning the value
  return sign * v_next;
}

//==============================================================================
template<typename SdfPtrT, typename SdfElementPtrT>
bool get_element_required(
  SdfPtrT& _sdf,
  const std::string& _element_name,
  SdfElementPtrT& _element)
{
  if (!_sdf->HasElement(_element_name))
  {
    std::cerr << "Element [" << _element_name << "] not found" << std::endl;
    return false;
  }
  // using GetElementImpl() because for sdf::v9 GetElement() is not const
  _element = _sdf->GetElementImpl(_element_name);
  return true;
}

/*
double compute_desired_rate_of_change(
    double _s_target,
    double _v_actual,
    const MotionParams& _motion_params,
    const double _dt);

bool get_element_required(
    const sdf::ElementPtr& _sdf,
    const std::string& _element_name,
    sdf::ElementPtr& _element);
*/

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T, typename SdfPtrT>
bool get_sdf_attribute_required(SdfPtrT& sdf, const std::string& attribute_name,
  T& value)
{
  if (sdf->HasAttribute(attribute_name))
  {
    if (sdf->GetAttribute(attribute_name)->Get(value))
    {
      std::cout << "Using specified attribute value [" << value
                << "] for property [" << attribute_name << "]"
                << std::endl;
      return true;
    }
    else
    {
      std::cerr << "Failed to parse sdf attribute for [" << attribute_name
                << "]" << std::endl;
    }
  }
  else
  {
    std::cerr << "Attribute [" << attribute_name << "] not found" << std::endl;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T, typename SdfPtrT>
bool get_sdf_param_required(SdfPtrT& sdf, const std::string& parameter_name,
  T& value)
{
  if (sdf->HasElement(parameter_name))
  {
    if (sdf->GetElement(parameter_name)->GetValue()->Get(value))
    {
      std::cout << "Using specified value [" << value << "] for property ["
                << parameter_name << "]" << std::endl;
      return true;
    }
    else
    {
      std::cerr << "Failed to parse sdf value for [" << parameter_name << "]"
                <<std::endl;
    }
  }
  else
  {
    std::cerr << "Property [" << parameter_name << "] not found" << std::endl;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T, typename SdfPtrT>
void get_sdf_param_if_available(SdfPtrT& sdf, const std::string& parameter_name,
  T& value)
{
  if (sdf->HasElement(parameter_name))
  {
    if (sdf->GetElement(parameter_name)->GetValue()->Get(value))
    {
      std::cout << "Using specified value [" << value << "] for property ["
                << parameter_name << "]" << std::endl;
    }
    else
    {
      std::cerr << "Failed to parse sdf value for [" << parameter_name
                << "]" << std::endl;
    }
  }
  else
  {
    std::cout << "Using default value [" << value << "] for property ["
              << parameter_name << "]" << std::endl;
  }
}

} // namespace building_sim_common

#endif // SRC__BUILDING_SIM_COMMON__UTILS_HPP
