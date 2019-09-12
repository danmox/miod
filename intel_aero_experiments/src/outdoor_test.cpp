#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <intel_aero_navigation/mavros_uav.h>

#include <string.h>


#define OT_INFO(fmt, ...) ROS_INFO("[outdoor_test] " fmt, ##__VA_ARGS__)


nav_msgs::Odometry::Ptr odom_ptr;
void odomCB(const nav_msgs::Odometry::Ptr& msg)
{
  odom_ptr = msg;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "outdoor_test");
  ros::NodeHandle nh, pnh("~");

  ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCB);

  intel_aero_navigation::MavrosUAV aero(nh, pnh);

  ros::Rate loop_rate(1);
  OT_INFO("waiting for odom");
  while (nh.ok() && !odom_ptr) {
    loop_rate.sleep();
    ros::spinOnce();
  }
  OT_INFO("received odometry");

  geometry_msgs::PoseStamped robot_pose;
  robot_pose.pose = odom_ptr->pose.pose;
  robot_pose.pose.position.z = 2.0;
  robot_pose.pose.position.x += 2.0;

  OT_INFO("sending takeoff command");
  aero.takeoff();

  loop_rate = ros::Rate(10);
  OT_INFO("publishing position waypoint for 20 seconds");
  ros::Time start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 20.0) {
    aero.sendLocalPositionCommand(robot_pose);
    loop_rate.sleep();
    ros::spinOnce();
  }

  OT_INFO("sending landing command");
  aero.land();

  return 0;
}
