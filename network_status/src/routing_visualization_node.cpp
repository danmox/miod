#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <network_status/RatePair.h>
#include <routing_msgs/NetworkUpdate.h>
#include <string.h>
#include <sstream>
#include <vector>
#include <map>


routing_msgs::NetworkUpdate::Ptr net_cmd;
void networkUpdateCB(const routing_msgs::NetworkUpdate::Ptr& msg)
{
  net_cmd = msg;
}


std::map<int, geometry_msgs::Point> node_position;
std::map<int, bool> received_pose;
void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg, int id)
{
  node_position[id] = msg->pose.position;
  received_pose[id] = true;
}


template<typename T>
bool all(const T& cont, bool val)
{
  return std::find(cont.begin(), cont.end(), !val) == cont.end();
}


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
  ros::init(argc, argv, "routing_visualization_node");
  ros::NodeHandle nh, pnh("~");

  ros::Publisher viz_pub = nh.advertise<visualization_msgs::Marker>("network_visualization", 10);
  ros::Subscriber net_sub = nh.subscribe("network_update", 2, networkUpdateCB);

  XmlRpc::XmlRpcValue task_agent_ids, comm_agent_ids;
  std::string world_frame, pose_topic_prefix, pose_topic_suffix;
  getParamStrict(nh, "/comm_agent_ids", task_agent_ids);
  getParamStrict(nh, "/task_agent_ids", comm_agent_ids);
  getParamStrict(pnh, "world_frame", world_frame);
  getParamStrict(pnh, "pose_topic_prefix", pose_topic_prefix);
  getParamStrict(pnh, "pose_topic_suffix", pose_topic_suffix);
  int total_agents = task_agent_ids.size() + comm_agent_ids.size();

  // extract agent ids

  std::vector<int> agent_ids;

  ROS_ASSERT(task_agent_ids.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < task_agent_ids.size(); ++i) {
    ROS_ASSERT(task_agent_ids[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    agent_ids.push_back(static_cast<int>(task_agent_ids[i]));
  }

  ROS_ASSERT(comm_agent_ids.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < comm_agent_ids.size(); ++i) {
    ROS_ASSERT(comm_agent_ids[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    agent_ids.push_back(static_cast<int>(comm_agent_ids[i]));
  }

  // agent IP -> agent ID map

  std::map<std::string, int> ip_to_id;
  for (int i = 0; i < total_agents; ++i) {
    std::string ip = std::string("10.42.0.") + std::to_string(agent_ids[i]);
    ip_to_id[ip] = agent_ids[i];
  }

  // pose subscribers

  namespace stdph = std::placeholders;
  std::vector<ros::Subscriber> pose_subs;
  for (int i = 0; i < total_agents; ++i) {
    std::stringstream ss;
    ss << pose_topic_prefix << "aero" << agent_ids[i] << pose_topic_suffix;
    auto fcn = std::bind(&poseCB, stdph::_1, agent_ids[i]);
    pose_subs.push_back(nh.subscribe<geometry_msgs::PoseStamped>(ss.str(), 10, fcn));
  }

  // main loop

  ros::Rate rate(10);
  ROS_INFO("[routing_visualization_node] starting loop");
  while (ros::ok()) {

    rate.sleep();
    ros::spinOnce();

    bool received_all_poses = true;
    for (auto& val : received_pose)
      if (!val.second)
        received_all_poses = false;

    if (net_cmd && received_all_poses) {

      visualization_msgs::Marker marker;
      marker.header.frame_id = world_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "viz";
      marker.id = 1;
      marker.lifetime = ros::Duration(1.0);
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.5;
      marker.pose.orientation.w = 1.0;
      for (const auto& prt_entry : net_cmd->routes) {
        for (int j = 0; j < prt_entry.gateways.size(); ++j) {

          if (ip_to_id.count(prt_entry.node) == 0) {
            ROS_WARN("invalid node IP: %s", prt_entry.node.c_str());
            continue;
          }

          marker.points.push_back(node_position[ip_to_id[prt_entry.node]]);
          marker.points.push_back(node_position[ip_to_id[prt_entry.gateways[j].IP]]);

          std_msgs::ColorRGBA color;
          color.g = 1.0;
          color.a = prt_entry.gateways[j].prob;
          ROS_DEBUG("[routing_visualization_node] alpha = %.2f", color.a);

          marker.colors.push_back(color);
          marker.colors.push_back(color);
        }
      }

      viz_pub.publish(marker);

    } else {
      ROS_DEBUG("[routing_visualization_node] no network update or missing node poses; skipping this iteration");
    }

  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = world_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = "viz";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.color.a = 1.0;
  viz_pub.publish(marker);

  return 0;
}
