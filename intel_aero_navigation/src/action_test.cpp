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
  ros::NodeHandle nh;

  std::string sn = "waypoint_navigation_node";
  NavClient nav_client(nh, sn.c_str(), true);
  ROS_INFO("waiting for %s action server to start", sn.c_str());
  nav_client.waitForServer();
  ROS_INFO("%s action server ready", sn.c_str());

  intel_aero_navigation::WaypointNavigationGoal goal_msg;
  geometry_msgs::Pose goal;
  goal.orientation.w = 1.0;
  goal.position.x = 10;
  goal.position.y = 10;
  goal.position.z = 2;
  goal_msg.waypoints.push_back(goal);
  goal.position.x = 0;
  goal.position.y = 0;
  goal.position.z = 10;
  goal_msg.waypoints.push_back(goal);
  goal.position.x = 10;
  goal.position.y = -10;
  goal.position.z = 5;
  goal_msg.waypoints.push_back(goal);
  goal_msg.end_action = intel_aero_navigation::WaypointNavigationGoal::LAND;

  ROS_INFO("sending goal");
  nav_client.sendGoal(goal_msg);
  ROS_INFO("waiting for result");
  nav_client.waitForResult();
  ROS_INFO("test complete");

  return 0;
}
