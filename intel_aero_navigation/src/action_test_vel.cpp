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
  ros::init(argc, argv, "flight_navigation");
  ros::NodeHandle nh;

  std::string sn = "/aero1/waypoint_navigation_vel_nodelet";
  NavClient nav_client(sn.c_str(), true);
  ROS_INFO("[action_test] waiting for %s action server to start", sn.c_str());
  nav_client.waitForServer();
  ROS_INFO("[action_test] %s action server ready", sn.c_str());

  std::vector<geometry_msgs::Pose> flight_wp;
  geometry_msgs::Pose goal;
  goal.orientation.w = 1.0;
  goal.position.x = 10;
  goal.position.y = 10;
  goal.position.z = 2;
  flight_wp.push_back(goal);
  goal.position.x = 0;
  goal.position.y = 0;
  goal.position.z = 10;
  flight_wp.push_back(goal);
  goal.position.x = 10;
  goal.position.y = -10;
  goal.position.z = 5;
  flight_wp.push_back(goal);

  intel_aero_navigation::WaypointNavigationGoal goal_msg;
  goal_msg.waypoints = flight_wp;
  goal_msg.end_behavior = intel_aero_navigation::WaypointNavigationGoal::LAND;

  ROS_INFO("[action_test] sending goal");
  nav_client.sendGoal(goal_msg);
  ROS_INFO("[action_test] waiting for result");
  nav_client.waitForResult();

  return 0;
}
