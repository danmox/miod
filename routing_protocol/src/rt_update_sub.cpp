#include <ros/ros.h>
#include <routing_msgs/PRTableEntry.h>
#include <routing_msgs/ProbGateway.h>
#include <routing_msgs/NetworkUpdate.h>
#include <fstream>
#include <vector>
#include <iostream>


void counterCallback(const routing_msgs::NetworkUpdate& msg)
{
  ROS_INFO("%s", msg.routes[0].node.c_str());
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "rt_update_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("rt_upd", 1000, counterCallback);
    ros::spin();
    return 0;
}
