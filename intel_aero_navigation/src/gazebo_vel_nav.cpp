#include <intel_aero_navigation/gazebo_vel_nav.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SetPhysicsProperties.h>


namespace intel_aero_navigation {


#define VN_INFO(fmt, ...) ROS_INFO("[GazeboVelNav] " fmt, ##__VA_ARGS__)
#define VN_ERROR(fmt, ...) ROS_ERROR("[GazeboVelNav] " fmt, ##__VA_ARGS__)
#define VN_FATAL(fmt, ...) ROS_FATAL("[GazeboVelNav] " fmt, ##__VA_ARGS__)
#define VN_DEBUG(fmt, ...) ROS_DEBUG("[GazeboVelNav] " fmt, ##__VA_ARGS__)


template<typename T>
void getParamStrict(const ros::NodeHandle& nh, std::string param_name, T& param)
{
  if (!nh.getParam(param_name, param)) {
    VN_FATAL("failed to get ROS param \"%s\"", param_name.c_str());
    exit(EXIT_FAILURE);
  }
}


GazeboVelNav::GazeboVelNav(std::string name, ros::NodeHandle nh_, ros::NodeHandle pnh_):
  VelNav(name, nh_, pnh_)
{
  // for changing simulation gravity settings
  set_client = nh.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
  get_client = nh.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
}

bool GazeboVelNav::systemInitialized()
{
  gazebo_msgs::GetPhysicsProperties::Request get_req;
  gazebo_msgs::GetPhysicsProperties::Response get_resp;

  if (!get_client.call(get_req, get_resp)) {
    VN_ERROR("failed to get physics properties: unable to determine the state of simulation gravity");
    return false;
  }

  double gravity_mag = sqrt(pow(get_resp.gravity.x, 2.0) +
                            pow(get_resp.gravity.y, 2.0) +
                            pow(get_resp.gravity.z, 2.0));

  if (gravity_mag > 1e-10) {
    VN_INFO("current simulation gravity vector magnitude = %f", gravity_mag);
    return false;
  }

  return true;
}


void GazeboVelNav::initializeSystem()
{
  gazebo_msgs::GetPhysicsProperties::Request get_req;
  gazebo_msgs::GetPhysicsProperties::Response get_resp;

  if (!get_client.call(get_req, get_resp)) {
    VN_ERROR("failed to set gravity to zero: unable to fetch current gravity parameters");
    return;
  }

  gazebo_msgs::SetPhysicsProperties::Request set_req;
  gazebo_msgs::SetPhysicsProperties::Response set_resp;

  set_req.time_step = get_resp.time_step;
  set_req.max_update_rate = get_resp.max_update_rate;
  set_req.gravity.x = 0.0;
  set_req.gravity.y = 0.0;
  set_req.gravity.z = 0.0;
  set_req.ode_config = get_resp.ode_config;

  if (set_client.call(set_req, set_resp))
    VN_INFO("successfully set gravity to zero");
  else
    VN_ERROR("failed to set gravity to zero: waypoint navigation may not function as expected");
}


} // namespace intel_aero_navigation
