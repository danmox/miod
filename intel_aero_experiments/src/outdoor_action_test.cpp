#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>
#include <actionlib/client/simple_action_client.h>

#include <vector>
#include <string.h>
#include <sstream>


typedef intel_aero_navigation::WaypointNavigationAction NavAction;
typedef actionlib::SimpleActionClient<NavAction> NavClient;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_test");
  ros::NodeHandle nh, pnh("~");

  std::string action_server;
  pnh.param<std::string>("action_server", action_server, "px4_waypoint_navigation_nodelet");
  NavClient nav_client(nh, action_server.c_str(), true);
  ROS_INFO("waiting for %s action server to start", action_server.c_str());
  nav_client.waitForServer();
  ROS_INFO("%s action server ready", action_server.c_str());

  intel_aero_navigation::WaypointNavigationGoal goal_msg;
  goal_msg.header.frame_id = "world";
  goal_msg.header.stamp = ros::Time::now();
  geometry_msgs::Pose goal;
  goal.position.x = 24.0;
  goal.position.y = 4.0;
  goal.position.z = 4.0;
  goal_msg.waypoints.push_back(goal);
  goal.position.x = 24.0;
  goal.position.y = 12.0;
  goal.position.z = 4.0;
  goal_msg.waypoints.push_back(goal);
  goal.position.x = 5.0;
  goal.position.y = 12.0;
  goal.position.z = 4.0;
  goal_msg.waypoints.push_back(goal);
  goal.position.x = 5.0;
  goal.position.y = 4.0;
  goal.position.z = 4.0;
  goal_msg.waypoints.push_back(goal);
  goal_msg.end_action = intel_aero_navigation::WaypointNavigationGoal::LAND;

  ROS_INFO("sending goal");
  nav_client.sendGoal(goal_msg);
  ROS_INFO("waiting for result");
  nav_client.waitForResult();
  ROS_INFO("test complete");

  return 0;
}
