#ifndef INTEL_AERO_NAVIGATION_WAYPOINT_NAVIGATION_H_
#define INTEL_AERO_NAVIGATION_WAYPOINT_NAVIGATION_H_


#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>

#include <actionlib/server/simple_action_server.h>
#include <grid_mapping/grid.hpp>
#include <intel_aero_navigation/mavros_uav.h>

#include <mutex>
#include <vector>
#include <stack>
#include <iterator>
#include <string.h>
#include <thread>


namespace intel_aero_navigation {


class WaypointNavigation
{
  protected:
    MavrosUAV aero; // wrapper class interface to UAV

    // parameters
    std::string path_frame_id;
    double waypoint_tol;

    ros::NodeHandle nh, pnh;
    ros::Subscriber costmap_sub, odom_sub;
    ros::Publisher path_pub;

    std::mutex costmap_mutex, odom_mutex; // need thread safety for odom, costmap
    typedef grid_mapping::Grid<int8_t> Costmap;
    Costmap costmap; // graph for planning (wrapping nav_msgs::OccupancyGrid)
    nav_msgs::OccupancyGrid::ConstPtr ros_costmap_ptr; // costmap from costmap_2d
    geometry_msgs::PoseStamped goal;
    nav_msgs::Odometry::ConstPtr odom;

    std::thread takeoff_thread;
    bool takeoff_complete;

    // path of waypoints
    std::vector<geometry_msgs::PoseStamped> path;
    std::vector<geometry_msgs::PoseStamped>::iterator path_it;

    // action server
    actionlib::SimpleActionServer<intel_aero_navigation::WaypointNavigationAction> nav_server;

    // methods
    std::vector<int> AStar(const grid_mapping::Point, const grid_mapping::Point) const;
    bool obstacleFree(const std::vector<int>&) const;
    void publishPath(const geometry_msgs::PoseStamped&) const;

  public:
    WaypointNavigation(std::string, ros::NodeHandle, ros::NodeHandle);

    void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr&);
    void odomCB(const nav_msgs::Odometry::ConstPtr&);
    void goalCB();
    void preemptCB();
};


}


#endif
