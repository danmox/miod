//#include <communication_predict/CommunicationPredict.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <string.h>
#include <sstream>

double distance(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
  return sqrt(pow(p2.x - p1.x, 2.0) + pow(p2.y - p1.y, 2.0)
              + pow(p2.z - p1.z, 2.0));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "network_visualization_node");
  ros::NodeHandle nh, pnh("~");

  ros::Publisher viz_pub = nh.advertise<visualization_msgs::Marker>("network_visualization", 10);

  tf2_ros::Buffer tf2_buff;
  tf2_ros::TransformListener tf2_listener(tf2_buff);

  int number_of_agents, task_agent_id;
  std::string world_frame;
  double dist_threshold(0.0);
  if (!pnh.getParam("number_of_agents", number_of_agents) ||
      !pnh.getParam("task_agent_id", task_agent_id) ||
      !pnh.getParam("world_frame", world_frame) ||
      !pnh.getParam("dist_threshold", dist_threshold)) {
    ROS_FATAL("[network_visualization_node] failed to fetch parameter(s) from server");
    return -1;
  }

  std_msgs::ColorRGBA green;
  green.g = 1.0;
  green.a = 1.0;

  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.a = 1.0;

  geometry_msgs::Vector3 scale;
  scale.x = 0.5;

  /*
  ROS_INFO("[network_visualization_node] giving other nodes some time to start");
  ros::Rate countdown(1);
  for (int i = 0; i < 10; ++i) {
    countdown.sleep();
    ros::spinOnce();
  }
  */

  ros::Rate rate(10);
  ROS_INFO("[network_visualization_node] starting loop");
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();

    bool publish_viz = true;
    std::vector<geometry_msgs::Point> node_location(number_of_agents);
    for (int i = 1; i <= number_of_agents; ++i) {
      std::stringstream ss;
      ss << "aero" << i << "/base_link";
      std::string node_frame = ss.str();

      geometry_msgs::TransformStamped trans;
      try {
        trans = tf2_buff.lookupTransform(world_frame, node_frame, ros::Time(0));
      } catch (tf2::TransformException &ex) {
        ROS_WARN("[network_visualization_node] %s", ex.what());
        publish_viz = false;
        break;
      }

      geometry_msgs::Point point;
      point.x = trans.transform.translation.x;
      point.y = trans.transform.translation.y;
      point.z = trans.transform.translation.z;
      node_location[i-1] = point;
    }

    if (publish_viz) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = world_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "viz";
      marker.id = 1;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::DELETEALL;
      marker.color.a = 1.0;
      viz_pub.publish(marker);

      marker.action = visualization_msgs::Marker::ADD;
      marker.scale = scale;
      for (int i = 0; i < number_of_agents; ++i) {
        for (int j = 0; j < number_of_agents; ++j) {
          if (i == j || distance(node_location[i], node_location[j]) > dist_threshold)
            continue;

          marker.points.push_back(node_location[i]);
          marker.points.push_back(node_location[j]);

          if (i == task_agent_id || j == task_agent_id) {
            marker.colors.push_back(green);
            marker.colors.push_back(green);
          } else {
            marker.colors.push_back(red);
            marker.colors.push_back(red);
          }
        }
      }

      viz_pub.publish(marker);
    } else {
      ROS_WARN("[network_visualization_node] not publishing this iteration");
    }
  }

  return 0;
}
