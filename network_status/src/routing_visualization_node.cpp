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


routing_msgs::NetworkUpdate::Ptr net_cmd;
void networkUpdateCB(const routing_msgs::NetworkUpdate::Ptr& msg)
{
  net_cmd = msg;
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

  // TODO change to odom subscribers?
  tf2_ros::Buffer tf2_buff;
  tf2_ros::TransformListener tf2_listener(tf2_buff);

  XmlRpc::XmlRpcValue task_ip_list, comm_ip_list;
  std::string world_frame;
  getParamStrict(nh, "/comm_agent_ips", task_ip_list);
  getParamStrict(nh, "/task_agent_ips", comm_ip_list);
  getParamStrict(pnh, "world_frame", world_frame);
  int number_of_agents = task_ip_list.size() + comm_ip_list.size();

  ros::Rate rate(10);
  ROS_INFO("[routing_visualization_node] starting loop");
  while (ros::ok()) {

    rate.sleep();
    ros::spinOnce();

    if (net_cmd) {

      // fetch agent locations
      bool publish_viz = true;
      std::vector<geometry_msgs::Point> node_location(number_of_agents);
      for (int i = 0; i < number_of_agents; ++i) {

        std::stringstream ss;
        ss << "aero" << i+1 << "/base_link";
        std::string node_frame = ss.str();

        geometry_msgs::TransformStamped trans;
        try {
          trans = tf2_buff.lookupTransform(world_frame, node_frame, ros::Time(0));
        } catch (tf2::TransformException &ex) {
          ROS_WARN("[routing_visualization_node] %s", ex.what());
          publish_viz = false;
          break;
        }

        geometry_msgs::Point point;
        point.x = trans.transform.translation.x;
        point.y = trans.transform.translation.y;
        point.z = trans.transform.translation.z;
        node_location[i] = point;
      }

      if (publish_viz) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = world_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "viz";
        marker.id = 1;
        marker.lifetime = ros::Duration(1.0);
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.5;
        for (const auto& prt_entry : net_cmd->routes) {
          for (int j = 0; j < prt_entry.gateways.size(); ++j) {
            marker.points.push_back(node_location[prt_entry.node[0]]);
            marker.points.push_back(node_location[prt_entry.gateways[j].IP[0]]);

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
        ROS_DEBUG("[routing_visualization_node] could not fetch all poses from tf: not publishing this iteration");
      }
    } else {
      ROS_DEBUG("[routing_visualization_node] no routing solution: not publishing this iteration");
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
