#include <intel_aero_navigation/waypoint_navigation.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_navigation_node");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  intel_aero_navigation::WaypointNavigation nav(ros::this_node::getName(), nh, pnh);

  ros::waitForShutdown();

  return 0;
}
