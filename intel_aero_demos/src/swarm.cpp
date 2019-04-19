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
  ros::init(argc, argv, "swarm_test");
  ros::NodeHandle nh;

  int number_of_agents;
  if (!nh.getParam("/number_of_agents", number_of_agents)) {
    ROS_ERROR("[swarm_test] unable to fet number_of_agents param");
    return -1;
  }

  double circle_radius = 10.0;
  double desired_altitude = 4.0;

  std::vector<std::string> action_names;
  for (int i = 0; i < number_of_agents; ++i) {
    std::stringstream ss;
    ss << "/aero" << i+1 << "/waypoint_navigation_vel_nodelet";
    action_names.push_back(ss.str());
  }

  std::vector<std::shared_ptr<NavClient>> nav_client_ptrs;
  for (int i = 0; i < number_of_agents; ++i) {
    std::shared_ptr<NavClient> ptr(new NavClient(action_names[i].c_str(), true));
    nav_client_ptrs.push_back(ptr);
    ROS_INFO("[swarm_test] waiting for %s action server to start",
             action_names[i].c_str());
    nav_client_ptrs[i]->waitForServer();
    ROS_INFO("[swarm_test] %s action server ready", action_names[i].c_str());
  }

  // spread agents around the a circle
  std::vector<geometry_msgs::Pose> wps;
  for (int i = 0; i < number_of_agents; ++i) {
    geometry_msgs::Pose goal;
    goal.position.x = circle_radius * cos(i*2.0*M_PI/number_of_agents);
    goal.position.y = circle_radius * sin(i*2.0*M_PI/number_of_agents);
    goal.position.z = desired_altitude;
    goal.orientation.w = 1.0;
    wps.push_back(goal);
  }

  // send goals
  for (int i = 0; i < number_of_agents; ++i) {
    intel_aero_navigation::WaypointNavigationGoal goal_msg;
    goal_msg.waypoints.push_back(wps[i]);
    goal_msg.end_behavior = intel_aero_navigation::WaypointNavigationGoal::HOVER;
    ROS_INFO("[swarm_test]: sending goal %d", i);
    nav_client_ptrs[i]->sendGoal(goal_msg);
  }

  for (int i = 0; i < number_of_agents; ++i) {
    ROS_INFO("[swarm_test] waiting for agent %d to finish", i);
    nav_client_ptrs[i]->waitForResult();
  }

  ROS_INFO("[swarm_test] test complete!");
  return 0;
}
