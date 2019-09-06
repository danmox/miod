#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>
#include <actionlib/client/simple_action_client.h>

#include <mavros_msgs/ParamSet.h>

#include <vector>
#include <string.h>
#include <sstream>

/*
  Patrol

  In this demo two quadrotors patrol the perimeter of a small collection of
  buildings while another group of agents support a wireless connection between.

  Use patrol.launch to run this demo.
*/


int main(int argc, char** argv)
{
  ros::init(argc, argv, "patrol_demo");
  ros::NodeHandle nh, gnh("/");

  int agent_count;
  double circle_radius;
  if (!gnh.getParam("task_agent_count", agent_count) ||
      !gnh.getParam("patrol_radius", circle_radius)) {
    ROS_FATAL("[patrol_demo] failed to fetch parameters from server");
    exit(EXIT_FAILURE);
  }

  typedef intel_aero_navigation::WaypointNavigationAction NavAction;
  typedef actionlib::SimpleActionClient<NavAction> NavClient;
  std::vector<std::shared_ptr<NavClient>> nav_clients;
  for (int i = 1; i <= agent_count; ++i) {
    std::stringstream ss;
    ss << "/aero" << i << "/waypoint_navigation_vel_nodelet";
    std::string sn = ss.str();
    std::shared_ptr<NavClient> ac_ptr(new NavClient(sn.c_str(), true));
    ROS_INFO("[patrol_demo] waiting for %s action server to start", sn.c_str());
    ac_ptr->waitForServer();
    ROS_INFO("[patrol_demo] %s action server ready", sn.c_str());
    nav_clients.push_back(ac_ptr);
  }

  std::vector<intel_aero_navigation::WaypointNavigationGoal> agent_goals;
  int num_points = 20;
  for (int j = 0; j < agent_count; ++j) {
    geometry_msgs::Pose goal;
    goal.orientation.w = 1.0;
    goal.position.z = 3.0;

    double angle_offset = j*2.0*M_PI/agent_count;
    std::vector<geometry_msgs::Pose> circle_points;
    for (int i = 0; i <= num_points; ++i) {
      goal.position.x = circle_radius*cos(i*2.0*M_PI/num_points + angle_offset);
      goal.position.y = circle_radius*sin(i*2.0*M_PI/num_points + angle_offset);
      circle_points.push_back(goal);
    }

    intel_aero_navigation::WaypointNavigationGoal goal_msg;
    goal_msg.waypoints = circle_points;
    goal_msg.end_action = intel_aero_navigation::WaypointNavigationGoal::LAND;
    agent_goals.push_back(goal_msg);
  }

  ROS_INFO("[patrol_demo] sending waypoints");
  for (int i = 0; i < agent_count; ++i) {
    nav_clients[i]->sendGoal(agent_goals[i]);
  }

  for (auto& client : nav_clients)
    client->waitForResult();

  return 0;
}
