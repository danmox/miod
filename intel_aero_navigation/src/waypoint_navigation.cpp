#include <intel_aero_navigation/waypoint_navigation.h>

namespace intel_aero_navigation {


WaypointNavigation::WaypointNavigation(ros::NodeHandle nh_, ros::NodeHandle pnh_)
{
  nh = nh_;
  pnh = pnh_;

  costmap_sub = nh.subscribe("costmap", 1, &WaypointNavigation::costmapCB, this);
}

void WaypointNavigation::costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  costmap = msg;
  ROS_INFO("[WaypointNavigation] received new costmap");
}


}
