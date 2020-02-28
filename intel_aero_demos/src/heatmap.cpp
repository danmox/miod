#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>
#include <actionlib/client/simple_action_client.h>

#include <mavros_msgs/ParamSet.h>

#include <vector>
#include <string.h>
#include <sstream>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "heatmap_demo");
  ros::NodeHandle nh, gnh("/");

  int agent_count;
  double heatmap_x_size;
  double heatmap_y_size;
  int heatmap_lines;
  if (!gnh.getParam("task_agent_count", agent_count) ||
      !gnh.getParam("heatmap_x_size", heatmap_x_size)  ||
      !gnh.getParam("heatmap_y_size", heatmap_y_size)  ||
      !gnh.getParam("heatmap_lines", heatmap_lines)) {
    ROS_FATAL("[heatmap_demo] failed to fetch parameters from server");
    exit(EXIT_FAILURE);
  }

  typedef intel_aero_navigation::WaypointNavigationAction NavAction;
  typedef actionlib::SimpleActionClient<NavAction> NavClient;
  std::vector<std::shared_ptr<NavClient>> nav_clients;
  for (int i = 1; i <= agent_count; ++i) {
    std::stringstream ss;
    ss << "/aero" << i << "/gazebo_vel_nav_nodelet";
    std::string sn = ss.str();
    std::shared_ptr<NavClient> ac_ptr(new NavClient(sn.c_str(), true));
    ROS_INFO("[heatmap_demo] waiting for %s action server to start", sn.c_str());
    ac_ptr->waitForServer();
    ROS_INFO("[heatmap_demo] %s action server ready", sn.c_str());
    nav_clients.push_back(ac_ptr);
  }

  std::vector<intel_aero_navigation::WaypointNavigationGoal> agent_goals;

  geometry_msgs::Pose goal;
  goal.orientation.w = 1.0;
  goal.position.z = 3.0;

  std::vector<geometry_msgs::Pose> heatmap_points;
  for (int i = 0; i <= heatmap_lines; ++i) {
    if (i % 2) {
      goal.position.x = -heatmap_x_size/2;
      goal.position.y = heatmap_y_size/2 - i*(heatmap_y_size/heatmap_lines);
      heatmap_points.push_back(goal);
      goal.position.x = heatmap_x_size/2;
      goal.position.y = heatmap_y_size/2 - i*(heatmap_y_size/heatmap_lines);
      heatmap_points.push_back(goal);
    } else {
      goal.position.x = heatmap_x_size/2;
      goal.position.y = heatmap_y_size/2 - i*(heatmap_y_size/heatmap_lines);
      heatmap_points.push_back(goal);
      goal.position.x = -heatmap_x_size/2;
      goal.position.y = heatmap_y_size/2 - i*(heatmap_y_size/heatmap_lines);
      heatmap_points.push_back(goal);
   }
  }
  intel_aero_navigation::WaypointNavigationGoal goal_msg;
  goal_msg.header.frame_id = "world";
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.waypoints = heatmap_points;
  goal_msg.end_action = intel_aero_navigation::WaypointNavigationGoal::LAND;
  agent_goals.push_back(goal_msg);


  ROS_INFO("[heatmap_demo] sending waypoints");
  nav_clients[0]->sendGoal(agent_goals[0]);
  nav_clients[0]->waitForResult();

  // for (int i = 0; i < agent_count; ++i) {
  //   nav_clients[i]->sendGoal(agent_goals[i]);
  // }

  // for (auto& client : nav_clients)
  //   client->waitForResult();

  // ROS_INFO("[patrol_demo] sending waypoints");
  // for (int i = 0; i < agent_count; ++i) {
  //   nav_clients[i]->sendGoal(agent_goals[i]);
  // }

  // for (auto& client : nav_clients)
  //   client->waitForResult();

  return 0;
}
