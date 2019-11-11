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
  ros::NodeHandle nh;

  double line_distance, desired_altitude, start_position;
  int source_node;
  if (!nh.getParam("/line_distance", line_distance) ||
      !nh.getParam("/desired_altitude", desired_altitude) ||
      !nh.getParam("/start_position", start_position) ||
      !nh.getParam("/source_node", source_node)) {
    ROS_FATAL("[line_demo] failed to fetch parameters from server");
    exit(EXIT_FAILURE);
  }

  typedef intel_aero_navigation::WaypointNavigationAction NavAction;
  typedef actionlib::SimpleActionClient<NavAction> NavClient;
  std::stringstream ss;
  ss << "/aero" << source_node << "/" << "gazebo_vel_nav_nodelet";
  std::string name = ss.str();
  NavClient nav_client(name, true);
  ROS_INFO("[line_demo] waiting for %s action server to start", name.c_str());
  nav_client.waitForServer();
  ROS_INFO("[line_demo] %s action server ready", name.c_str());

  geometry_msgs::Pose out, back;
  out.position.x = line_distance;
  out.position.z = desired_altitude;
  back.position.x = start_position;
  back.position.z = desired_altitude;

  intel_aero_navigation::WaypointNavigationGoal goal_msg;
  goal_msg.header.frame_id = "world";
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.waypoints.push_back(back);
  goal_msg.waypoints.push_back(out);
  goal_msg.waypoints.push_back(back);
  goal_msg.end_action = intel_aero_navigation::WaypointNavigationGoal::LAND;

  ROS_INFO("[line_demo] sending waypoints");
  nav_client.sendGoal(goal_msg);

  nav_client.waitForResult();

  return 0;
}
