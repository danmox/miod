#ifndef INTEL_AERO_NAVIGATION_WAYPOINT_NAVIGATION_H_
#define INTEL_AERO_NAVIGATION_WAYPOINT_NAVIGATION_H_


#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <actionlib/server/simple_action_server.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>

#include <grid_mapping/grid.hpp>

#include <mutex>
#include <vector>
#include <stack>
#include <iterator>
#include <string.h>


namespace intel_aero_navigation {


class WaypointNavigation
{
  protected:
    std::string path_frame_id;

    ros::NodeHandle nh, pnh;
    ros::Subscriber costmap_sub, odom_sub;
    ros::Publisher path_pub;

    std::mutex costmap_mutex, odom_mutex;
    typedef grid_mapping::Grid<int8_t> Costmap;
    Costmap costmap;
    nav_msgs::OccupancyGrid::ConstPtr ros_costmap_ptr;
    geometry_msgs::Pose goal;
    nav_msgs::Odometry::ConstPtr odom;
    std::vector<geometry_msgs::Pose> path;
    std::vector<geometry_msgs::Pose>::iterator current_waypoint;

    actionlib::SimpleActionServer<intel_aero_navigation::WaypointNavigationAction> nav_server;

    std::vector<int> AStar(const geometry_msgs::Pose, const geometry_msgs::Pose) const;
    bool obstacleFree(const std::vector<int>&) const;
    void publishPath(const geometry_msgs::Pose&) const;

  public:
    WaypointNavigation(std::string, ros::NodeHandle, ros::NodeHandle);

    void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr&);
    void odomCB(const nav_msgs::Odometry::ConstPtr&);
    void goalCB();
    void preemptCB();
};


}


#endif
