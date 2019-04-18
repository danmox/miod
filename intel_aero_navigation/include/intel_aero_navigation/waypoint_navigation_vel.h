#ifndef INTEL_AERO_NAVIGATION_WAYPOINT_NAVIGATION_VEL_H_
#define INTEL_AERO_NAVIGATION_WAYPOINT_NAVIGATION_VEL_H_


#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>

#include <actionlib/server/simple_action_server.h>
#include <grid_mapping/grid.hpp>
#include <intel_aero_navigation/mavros_uav.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>
#include <vector>
#include <stack>
#include <iterator>
#include <string.h>
#include <thread>


namespace intel_aero_navigation {


class WaypointNavigationVel
{
  protected:
    MavrosUAV aero; // wrapper class interface to UAV

    // parameters
    std::string local_frame, world_frame, costmap_frame;
    double position_tol, heading_tol;

    ros::NodeHandle nh, pnh;
    ros::Subscriber costmap_sub, odom_sub;
    ros::Publisher path_pub;
    ros::ServiceClient s_client, g_client;
    ros::Publisher vel_pub;

    tf2_ros::Buffer tf2_buff;
    tf2_ros::TransformListener tf2_listener;
    geometry_msgs::TransformStamped world_to_local, costmap_to_local;

    std::mutex costmap_mutex, odom_mutex; // need thread safety for odom, costmap
    typedef grid_mapping::Grid<int8_t> Costmap;
    Costmap costmap; // graph for planning (wrapping nav_msgs::OccupancyGrid)
    nav_msgs::OccupancyGrid::ConstPtr ros_costmap_ptr; // costmap from costmap_2d
    bool processed_costmap;
    nav_msgs::Odometry::ConstPtr odom;

    // action goal information
    int end_behavior; // what the quad does when it has reached the goal
    std::vector<geometry_msgs::PoseStamped> waypoints; // stored for reference

    std::thread takeoff_thread, landing_thread;
    bool takeoff_command_issued;

    // path of waypoints to follow
    std::vector<geometry_msgs::PoseStamped> path; // what the quad follows
    std::vector<geometry_msgs::PoseStamped>::iterator path_it;

    // action server
    actionlib::SimpleActionServer<intel_aero_navigation::WaypointNavigationAction> nav_server;

    // methods
    std::vector<int> AStar(const grid_mapping::Point, const grid_mapping::Point) const;
    bool obstacleFree(const std::vector<int>&) const;
    void publishPath(const geometry_msgs::PoseStamped&) const;
    void planPath(const geometry_msgs::PoseStamped&);

  public:
    WaypointNavigationVel(std::string, ros::NodeHandle, ros::NodeHandle);

    void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr&);
    void odomCB(const nav_msgs::Odometry::ConstPtr&);
    void goalCB();
    void preemptCB();
};


// return the quaternion corresponding to a yaw aligned with the line segment
// connecting points p1 and p2: see waypoint_navigation.cpp
geometry_msgs::Quaternion pathSegmentQuaternion(const geometry_msgs::Point& p1,
                                                const geometry_msgs::Point& p2);


}


#endif
