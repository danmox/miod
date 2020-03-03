#include <intel_aero_navigation/vel_nav.h>
#include <tf2/utils.h>


namespace intel_aero_navigation {


#define VN_INFO(fmt, ...) ROS_INFO("[VelNav] " fmt, ##__VA_ARGS__)
#define VN_ERROR(fmt, ...) ROS_ERROR("[VelNav] " fmt, ##__VA_ARGS__)
#define VN_FATAL(fmt, ...) ROS_FATAL("[VelNav] " fmt, ##__VA_ARGS__)
#define VN_DEBUG(fmt, ...) ROS_DEBUG("[VelNav] " fmt, ##__VA_ARGS__)


template<typename T>
void getParamStrict(const ros::NodeHandle& nh, std::string param_name, T& param)
{
  if (!nh.getParam(param_name, param)) {
    VN_FATAL("failed to get ROS param \"%s\"", param_name.c_str());
    exit(EXIT_FAILURE);
  }
}


VelNav::VelNav(std::string name, ros::NodeHandle nh_, ros::NodeHandle pnh_):
  NavBase(name, nh_, pnh_)
{
  getParamStrict(pnh, "linear_vel_des", linear_vel_des);
  getParamStrict(pnh, "angular_vel_des", angular_vel_des);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
}

// assumptions:
// 1) goal.pose.orientation points along the segment to be traversed
void VelNav::sendCommand(const geometry_msgs::PoseStamped& goal)
{
  double goal_vec[3];
  goal_vec[0] = goal.pose.position.x - robot_pose.pose.position.x;
  goal_vec[1] = goal.pose.position.y - robot_pose.pose.position.y;
  goal_vec[2] = goal.pose.position.z - robot_pose.pose.position.z;

  double mag = sqrt(goal_vec[0]*goal_vec[0] +
                    goal_vec[1]*goal_vec[1] +
                    goal_vec[2]*goal_vec[2]);

  // segment heading error
  double desired_heading = tf2::getYaw(goal.pose.orientation);
  double robot_heading = tf2::getYaw(robot_pose.pose.orientation);
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


void VelNav::executeEndAction(const int action)
{
  waypoints.clear(); // no more velocity commands will be sent
                     // quad will remain in place
}


} // namespace intel_aero_navigation
