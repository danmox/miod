#include <intel_aero_navigation/gazebo_vel_nav.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <tf2/utils.h>


namespace intel_aero_navigation {


#define VN_INFO(fmt, ...) ROS_INFO("[GazeboVelNav] " fmt, ##__VA_ARGS__)
#define VN_ERROR(fmt, ...) ROS_ERROR("[GazeboVelNav] " fmt, ##__VA_ARGS__)
#define VN_FATAL(fmt, ...) ROS_FATAL("[GazeboVelNav] " fmt, ##__VA_ARGS__)


template<typename T>
void getParamStrict(const ros::NodeHandle& nh, std::string param_name, T& param)
{
  if (!nh.getParam(param_name, param)) {
    VN_FATAL("failed to get ROS param \"%s\"", param_name.c_str());
    exit(EXIT_FAILURE);
  }
}


GazeboVelNav::GazeboVelNav(std::string name, ros::NodeHandle nh_, ros::NodeHandle pnh_):
  NavBase(name, nh_, pnh_)
{
  getParamStrict(pnh, "linear_vel_des", linear_vel_des);
  getParamStrict(pnh, "angular_vel_des", angular_vel_des);

  // for changing simulation gravity settings
  set_client = nh.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
  get_client = nh.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");

  vel_pub = nh.advertise<geometry_msgs::Twist>("vel_cmd", 10);
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


// assumptions:
// 1) goal.pose.orientation points along the segment to be traversed
void GazeboVelNav::sendCommand(const geometry_msgs::PoseStamped& goal)
{
  double goal_vec[3];
  goal_vec[0] = goal.pose.position.x - robot_pose->pose.position.x;
  goal_vec[1] = goal.pose.position.y - robot_pose->pose.position.y;
  goal_vec[2] = goal.pose.position.z - robot_pose->pose.position.z;

  double mag = sqrt(goal_vec[0]*goal_vec[0] +
                    goal_vec[1]*goal_vec[1] +
                    goal_vec[2]*goal_vec[2]);

  // segment heading error
  double desired_heading = tf2::getYaw(goal.pose.orientation);
  double robot_heading = tf2::getYaw(robot_pose->pose.orientation);
  double heading_error = desired_heading - robot_heading;
  if (heading_error > 2.0*M_PI)
    heading_error -= 2.0*M_PI;
  if (heading_error < -2.0*M_PI)
    heading_error += 2.0*M_PI;

  geometry_msgs::Twist velocity_command;
  velocity_command.angular.z = angular_vel_des * heading_error;

  if (mag > position_tol) {
    velocity_command.linear.x = linear_vel_des * goal_vec[0] / mag;
    velocity_command.linear.y = linear_vel_des * goal_vec[1] / mag;
    velocity_command.linear.z = linear_vel_des * goal_vec[2] / mag;
  }
  vel_pub.publish(velocity_command);
}


} // namespace intel_aero_navigation
