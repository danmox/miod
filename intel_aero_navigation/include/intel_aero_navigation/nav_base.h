#ifndef INTEL_AERO_NAVIGATION_NAV_BASE
#define INTEL_AERO_NAVIGATION_NAV_BASE


#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>

#include <actionlib/server/simple_action_server.h>
#include <grid_mapping/grid.hpp>

#include <tf2_ros/transform_listener.h>

#include <vector>
#include <deque>
#include <string.h>


namespace intel_aero_navigation {


class NavBase
{
  protected:
    double position_tol, heading_tol;

    ros::NodeHandle nh, pnh;
    ros::Subscriber costmap_sub, pose_sub;
    ros::Publisher path_pub;

    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;
    std::string local_frame;

    typedef grid_mapping::Grid<int8_t> Costmap;
    Costmap* costmap_ptr;
    geometry_msgs::PoseStamped robot_pose;
    bool received_robot_pose;

    // path of waypoints to follow
    std::deque<geometry_msgs::PoseStamped> waypoints;
    int end_action;

    // action server
    actionlib::SimpleActionServer<intel_aero_navigation::WaypointNavigationAction> nav_server;

    // methods
    std::vector<int> AStar(const grid_mapping::Point, const grid_mapping::Point) const;
    bool obstacleFree(const std::vector<int>&) const;
    void publishPath() const;
    void planPath();

    virtual bool systemInitialized() = 0;
    virtual void initializeSystem() = 0;
    virtual void sendCommand(const geometry_msgs::PoseStamped&) = 0;
    virtual void executeEndAction(const int action) = 0;

 public:
    NavBase(std::string, ros::NodeHandle&, ros::NodeHandle&);
    ~NavBase();

    void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr&);
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr&);
    void goalCB();
    void preemptCB();
};


} // namespace intel_aero_navigation


#endif
