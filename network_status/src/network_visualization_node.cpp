#include <CommunicationPredict.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <network_status/RatePair.h>
#include <string.h>
#include <sstream>
#include <vector>


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
  ros::Publisher rates_pub = nh.advertise<network_status::RatePair>("channel_rates", 10);
  ros::Publisher task_pub = nh.advertise<std_msgs::Float64>("task_agent_rate", 10);

  tf2_ros::Buffer tf2_buff;
  tf2_ros::TransformListener tf2_listener(tf2_buff);

  int number_of_agents, task_agent_id;
  std::string world_frame;
  double dist_threshold(0.0);
  if (!nh.getParam("/number_of_agents", number_of_agents) ||
      !nh.getParam("/task_agent_id", task_agent_id) ||
      !pnh.getParam("world_frame", world_frame)) {
    ROS_FATAL("[network_visualization_node] failed to fetch parameter(s) from server");
    return -1;
  }

  bool use_map = true;
  CommunicationPredict comm_sim(use_map);

  geometry_msgs::Vector3 scale;
  scale.x = 0.5;

  // compute pair permutations
  std::vector<network_status::RatePair> perms;
  for (int i = 1; i < number_of_agents; ++i) {
    for (int j = i+1; j <= number_of_agents; ++j) {
      network_status::RatePair pair;
      pair.id1 = i;
      pair.id2 = j;
      perms.push_back(pair);
    }
  }

  ros::Rate rate(10);
  ROS_INFO("[network_visualization_node] starting loop");
  std_msgs::Float64 task_rate;
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
      marker.lifetime = ros::Duration(1.0);
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale = scale;
      for (auto& pair : perms) {
        int i = pair.id1-1; // subtract to convert from names to indices
        int j = pair.id2-1;

        marker.points.push_back(node_location[i]);
        marker.points.push_back(node_location[j]);

        comm_sim.Predict(node_location[i].x, node_location[i].y,
                         node_location[j].x, node_location[j].y,
                         pair.rate, pair.std);

        std_msgs::ColorRGBA color;
        color.a = pair.rate;
        ROS_DEBUG("[network_visualization_node] alpha = %.2f", pair.rate);

        if (i == task_agent_id-1 || j == task_agent_id-1) {
          if (pair.rate > task_rate.data)
            task_rate.data = pair.rate;

          color.g = 1.0;
          marker.colors.push_back(color);
          marker.colors.push_back(color);
        } else {
          color.r = 1.0;
          marker.colors.push_back(color);
          marker.colors.push_back(color);
        }
      }
      task_pub.publish(task_rate);
      task_rate.data = 0.0;

      viz_pub.publish(marker);
    } else {
      ROS_WARN("[network_visualization_node] not publishing this iteration");
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
