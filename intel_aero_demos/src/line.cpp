#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>
#include <actionlib/client/simple_action_client.h>

#include <vector>
#include <string.h>
#include <sstream>

/*
  Line

  In this simulation, one task agent remains fixed on the ground while a second
  task agent moves away from it; meanwhile, a network agent bridges the gap
  between them.

  Use line.launch to initialize this demo.
*/


int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_demo_node");
  ros::NodeHandle nh, pnh("~");

  double desired_altitude, start_x, start_y, end_x;
  int source_node;
  if (!nh.getParam("/desired_altitude", desired_altitude) ||
      !nh.getParam("/start_x", start_x) ||
      !nh.getParam("/start_y", start_y) ||
      !nh.getParam("/end_x", end_x) ||
      !nh.getParam("/source_node", source_node)) {
    ROS_FATAL("[line_demo] failed to fetch parameters from server");
    exit(EXIT_FAILURE);
  }
  std::string nav_nodelet("px4_waypoint_navigation_nodelet");
  if (!pnh.getParam("nav_nodelet", nav_nodelet)) {
    ROS_WARN("[line_demo] using default value for nav_nodelet (%s)", nav_nodelet.c_str());
  }

  typedef intel_aero_navigation::WaypointNavigationAction NavAction;
  typedef actionlib::SimpleActionClient<NavAction> NavClient;
  std::stringstream ss;
  ss << "/aero" << source_node << "/" << nav_nodelet;
  std::string name = ss.str();
  NavClient nav_client(name, true);
  ROS_INFO("[line_demo] waiting for %s action server to start", name.c_str());
  nav_client.waitForServer();
  ROS_INFO("[line_demo] %s action server ready", name.c_str());

  geometry_msgs::Pose start;
  start.position.x = start_x;
  start.position.y = start_y;
  start.position.z = desired_altitude;
  start.orientation.w = 1.0;
  geometry_msgs::Pose end;
  end.position.x = end_x;
  end.position.y = start_y;
  end.position.z = desired_altitude;
  end.orientation.w = 1.0;

  intel_aero_navigation::WaypointNavigationGoal goal_msg;
  goal_msg.header.frame_id = "world";
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.waypoints.push_back(start);
  goal_msg.waypoints.push_back(end);
  goal_msg.waypoints.push_back(start);
  goal_msg.end_action = intel_aero_navigation::WaypointNavigationGoal::LAND;

  ROS_INFO("[line_demo] sending waypoints");
  nav_client.sendGoal(goal_msg);

  nav_client.waitForResult();

  return 0;
}
