#ifndef INTEL_AERO_NAVIGATION_WAYPOINT_NAVIGATION_H_
#define INTEL_AERO_NAVIGATION_WAYPOINT_NAVIGATION_H_


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_mapping/occ_grid.h>


namespace intel_aero_navigation {


class WaypointNavigation
{
  protected:
    ros::NodeHandle nh, pnh;
    ros::Subscriber costmap_sub;

    nav_msgs::OccupancyGrid::ConstPtr costmap;

    void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr&);

  public:
    WaypointNavigation(ros::NodeHandle, ros::NodeHandle);

};


}


#endif
