#ifndef SRC__BUILDING_GAZEBO_PLUGINS__UTILS_HPP
#define SRC__BUILDING_GAZEBO_PLUGINS__UTILS_HPP

#include <sdf/Element.hh>

namespace building_gazebo_plugins {

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

double compute_desired_rate_of_change(
    double _s_target,
    double _v_actual,
    const MotionParams& _motion_params,
    const double _dt);

bool get_element_required(
    const sdf::ElementPtr& _sdf,
    const std::string& _element_name,
    sdf::ElementPtr& _element);

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
bool get_sdf_attribute_required(const sdf::ElementPtr& sdf, const std::string& attribute_name, T& value)
{
  if(sdf->HasAttribute(attribute_name))
  {
    if(sdf->GetAttribute(attribute_name)->Get(value))
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
template<typename T>
bool get_sdf_param_required(const sdf::ElementPtr& sdf, const std::string& parameter_name, T& value)
{
  if(sdf->HasElement(parameter_name))
  {
    if(sdf->GetElement(parameter_name)->GetValue()->Get(value))
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
template<typename T>
void get_sdf_param_if_available(const sdf::ElementPtr& sdf, const std::string& parameter_name, T& value)
{
  if(sdf->HasElement(parameter_name))
  {
    if(sdf->GetElement(parameter_name)->GetValue()->Get(value))
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

} // namespace building_gazebo_plugins

#endif // SRC__BUILDING_GAZEBO_PLUGINS__UTILS_HPP
