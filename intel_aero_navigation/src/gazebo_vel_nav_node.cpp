#include <intel_aero_navigation/gazebo_vel_nav.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_vel_nav_node");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  intel_aero_navigation::GazeboVelNav nav(ros::this_node::getName(), nh, pnh);

  ros::waitForShutdown();

  return 0;
}
