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

  double x0 = 5.0;
  double y0 = 4.0;
  double xf = 22.0;
  double yf = 12.0;
  double step = 0.1;

  intel_aero_navigation::WaypointNavigationGoal goal_msg;
  goal_msg.header.frame_id = "world";
  goal_msg.header.stamp = ros::Time::now();
  geometry_msgs::Pose goal;
  goal.position.x = x0;
  goal.position.y = y0;
  goal.position.z = 3.0;
  goal.orientation.w = 1.0;
  goal_msg.waypoints.push_back(goal);

  while (goal.position.x < xf-step) {
    goal.position.x += step;
    goal_msg.waypoints.push_back(goal);
  }
  goal.position.x = xf;
  goal_msg.waypoints.push_back(goal);

  while (goal.position.y < yf-step) {
    goal.position.y += step;
    goal_msg.waypoints.push_back(goal);
  }
  goal.position.y = yf;
  goal_msg.waypoints.push_back(goal);

  while (goal.position.x > x0+step) {
    goal.position.x -= step;
    goal_msg.waypoints.push_back(goal);
  }
  goal.position.x = x0;
  goal_msg.waypoints.push_back(goal);

  while (goal.position.y > y0+step) {
    goal.position.y -= step;
    goal_msg.waypoints.push_back(goal);
  }
  goal.position.y = y0;
  goal_msg.waypoints.push_back(goal);

  goal_msg.end_action = intel_aero_navigation::WaypointNavigationGoal::LAND;

  ROS_INFO("sending goal");
  nav_client.sendGoal(goal_msg);
  ROS_INFO("waiting for result");
  nav_client.waitForResult();
  ROS_INFO("test complete");

  return 0;
}
