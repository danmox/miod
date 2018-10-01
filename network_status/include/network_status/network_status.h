#ifndef NETWORK_STATUS_NETWORK_STATUS_H
#define NETWORK_STATUS_NETWORK_STATUS_H

#include <communication_predict/CommunicationPredict.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace network_status {

class NetworkStatus
{
  public:
    NetworkStatus(ros::NodeHandle, ros::NodeHandle);

    void visualizationLoop();

  protected:
    ros::NodeHandle nh, pnh;
    ros::Publisher viz_pub;

    tf2_ros::Buffer tf2_buff;
    tf2_ros::TransformListener tf2_listener;

    std::vector<geometry_msgs::Point> node_locations;

    int number_of_robots, task_agent_id;

};

} // namespace network_status

#endif
