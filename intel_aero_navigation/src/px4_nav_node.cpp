#include <intel_aero_navigation/px4_nav.hpp>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "px4_navigation_node");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  intel_aero_navigation::PX4Nav nav(ros::this_node::getName(), nh, pnh);

  ros::waitForShutdown();

  return 0;
}
