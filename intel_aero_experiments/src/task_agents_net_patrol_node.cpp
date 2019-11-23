#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>
#include <actionlib/client/simple_action_client.h>

#include <vector>
#include <string.h>
#include <sstream>


typedef intel_aero_navigation::WaypointNavigationAction NavAction;
typedef actionlib::SimpleActionClient<NavAction> NavClient;


template<typename T>
void getParamStrict(const ros::NodeHandle& nh, std::string param_name, T& param)
{
  if (!nh.getParam(param_name, param)) {
    ROS_FATAL("[routing_visualization_node] failed to get ROS param \"%s\"", param_name.c_str());
    exit(EXIT_FAILURE);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_test");
  ros::NodeHandle nh, pnh("~");

  double net_x0, net_y0, net_xf, net_yf, altitude;
  XmlRpc::XmlRpcValue source_nodes;
  getParamStrict(nh, "/net_x0", net_x0);
  getParamStrict(nh, "/net_y0", net_y0);
  getParamStrict(nh, "/net_xf", net_xf);
  getParamStrict(nh, "/net_yf", net_yf);
  getParamStrict(nh, "/desired_altitude", altitude);
  getParamStrict(nh, "/source_nodes", source_nodes);

  std::string action_server;
  pnh.param<std::string>("action_server", action_server, "px4_waypoint_navigation_nodelet");
  ROS_INFO("using defaul value for action_server: %s", action_server.c_str());

  ROS_ASSERT(source_nodes.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(source_nodes.size() == 2);

  std::vector<std::shared_ptr<NavClient>> nav_clients;
  for (int i = 0; i < source_nodes.size(); ++i) {
    ROS_ASSERT(source_nodes[i].getType() == XmlRpc::XmlRpcValue::TypeInt);

    std::stringstream ss;
    ss << "/aero" << static_cast<int>(source_nodes[i]) << "/" << action_server;
    std::string as_name = ss.str();

    std::shared_ptr<NavClient> nav_client(new NavClient(nh, as_name.c_str(), true));
    nav_clients.push_back(nav_client);
    ROS_INFO("waiting for %s action server to start", as_name.c_str());
    nav_clients.back()->waitForServer();
    ROS_INFO("%s action server ready", as_name.c_str());
  }

  // ------ net --------------------
  // | p4 --------------------- p3 |
  // | |                        |  |
  // | |                        |  |
  // | p1 --------------------- p2 |
  // x------------------------------
  // ^ datum

  geometry_msgs::Pose p1;
  p1.position.x = net_x0;
  p1.position.y = net_y0;
  p1.position.z = altitude;
  p1.orientation.w = 1.0;

  geometry_msgs::Pose p2 = p1;
  p2.position.x = net_xf;

  geometry_msgs::Pose p3 = p2;
  p3.position.y = net_yf;

  geometry_msgs::Pose p4 = p3;
  p4.position.x = net_x0;

  // build opposing trajectories

  intel_aero_navigation::WaypointNavigationGoal traj1;
  traj1.header.frame_id = "world";
  traj1.header.stamp = ros::Time::now();
  traj1.end_action = intel_aero_navigation::WaypointNavigationGoal::LAND;
  traj1.waypoints.push_back(p1);
  traj1.waypoints.push_back(p2);
  traj1.waypoints.push_back(p3);
  traj1.waypoints.push_back(p4);

  intel_aero_navigation::WaypointNavigationGoal traj2 = traj1;
  traj2.waypoints.clear();
  traj2.waypoints.push_back(p3);
  traj2.waypoints.push_back(p4);
  traj2.waypoints.push_back(p1);
  traj2.waypoints.push_back(p2);

  ROS_INFO("sending goals");
  nav_clients[0]->sendGoal(traj1);
  nav_clients[1]->sendGoal(traj2);
  ROS_INFO("waiting for quads to complete trajectories");
  nav_clients[0]->waitForResult();
  nav_clients[1]->waitForResult();
  ROS_INFO("test complete");

  return 0;
}
