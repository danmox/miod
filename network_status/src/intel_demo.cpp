#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>
#include <actionlib/client/simple_action_client.h>

#include <vector>
#include <string.h>
#include <sstream>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_node");
  ros::NodeHandle nh, gnh("/");

  int number_of_robots, task_agent_id;
  if (!gnh.getParam("number_of_agents", number_of_robots) ||
      !gnh.getParam("task_agent_id", task_agent_id)) {
    ROS_FATAL("[demo_node] unable to fetch params from server");
    return -1;
  }

  typedef intel_aero_navigation::WaypointNavigationAction NavAction;
  typedef actionlib::SimpleActionClient<NavAction> NavClient;
  std::vector<std::shared_ptr<NavClient>> nav_clients;
  for (int i = 1; i <= number_of_robots; ++i) {
    std::stringstream ss;
    ss << "/aero" << i << "/waypoint_navigation_nodelet";
    std::string sn = ss.str();
    std::shared_ptr<NavClient> ac_ptr(new NavClient(sn.c_str(), true));
    ROS_INFO("[demo_node] waiting for %s action server to start", sn.c_str());
    ac_ptr->waitForServer();
    ROS_INFO("[demo_node] %s action server ready", sn.c_str());
    nav_clients.push_back(ac_ptr);
  }

  double desired_altitude = 2.0;
  std::vector<geometry_msgs::Pose> start_poses;
  geometry_msgs::Pose goal;
  goal.orientation.w = 1.0;
  goal.position.z = desired_altitude;

  // agent 1
  goal.position.x = 30;
  goal.position.y = 5;
  start_poses.push_back(goal);

  // agent 2
  goal.position.x = 55;
  goal.position.y = 18;
  start_poses.push_back(goal);

  // agent 3
  goal.position.x = 75;
  goal.position.y = 33;
  start_poses.push_back(goal);

  // agent 4
  goal.position.x = 63;
  goal.position.y = -10;
  start_poses.push_back(goal);

  // agent 5
  goal.position.x = 35;
  goal.position.y = -17;
  start_poses.push_back(goal);

  // agent 6
  goal.position.x = 27;
  goal.position.y = 0;
  start_poses.push_back(goal);

  ROS_INFO("[demo_node] starting quads");
  for (int i = 0; i < number_of_robots; ++i) {
    intel_aero_navigation::WaypointNavigationGoal goal_msg;
    goal_msg.waypoints.push_back(start_poses[i]);
    nav_clients[i]->sendGoal(goal_msg);
  }

  ROS_INFO("[demo_node] waiting for goals to be complete");
  for (int i = 0; i < number_of_robots; ++i) {
    nav_clients[i]->waitForResult();
  }
  ROS_INFO("[demo_node] completed takeoff");

  return 0;
}
