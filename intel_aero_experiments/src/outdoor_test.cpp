#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <intel_aero_navigation/mavros_uav.h>

#include <string.h>


#define OT_INFO(fmt, ...) ROS_INFO("[outdoor_test] " fmt, ##__VA_ARGS__)
#define OT_WARN(fmt, ...) ROS_WARN("[outdoor_test] " fmt, ##__VA_ARGS__)


int main(int argc, char** argv)
{
  ros::init(argc, argv, "outdoor_test");
  ros::NodeHandle nh, pnh("~");

  double altitude = 2.0;
  if (!pnh.getParam("altitude", altitude)) {
    OT_WARN("using default altitude of %f", altitude);
  }

  intel_aero_navigation::MavrosUAV aero(nh, pnh);

  geometry_msgs::PoseStamped datum_pose;
  datum_pose.header.frame_id = "world";
  datum_pose.pose.position.x = 0.0;
  datum_pose.pose.position.y = 0.0;
  datum_pose.pose.position.z = altitude;
  datum_pose.pose.orientation.w = 1.0;

  OT_INFO("sending takeoff command");
  aero.takeoff();

  ros::Rate loop_rate(10);
  OT_INFO("publishing position waypoint for 40 seconds");
  ros::Time start;
  do {start = ros::Time::now();} while (start.toSec() == 0.0);
  while (ros::ok() && (ros::Time::now() - start).toSec() < 40.0) {
    aero.sendLocalPositionCommand(datum_pose);
    loop_rate.sleep();
    ros::spinOnce();
  }

  OT_INFO("sending landing command");
  aero.land();

  return 0;
}
