#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

mavros_msgs::HomePosition home;
void home_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
  home = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &state_cb);
  ros::Subscriber home_sub = nh.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 10, &home_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

  ros::Rate rate(20.0); //setpoint publishing rate MUST be faster than 2Hz

  // make FCU connection

  int wait_count = 0;
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
    wait_count++;
    if (wait_count > 40) {
      ROS_INFO("waiting for FCU connection");
      wait_count = 0;
    }
  }
  ROS_INFO("FCU connection made");

  // send a few setpoints before starting

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "aero1/map";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  ROS_INFO("Publishing setpoints before liftoff");
  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  // set the quad in offboard mode

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  ROS_INFO("Attempting to set the quad in offboard mode");
  while (ros::ok() && !(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Offboard enabled");

  // arm the quad for takeoff

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ROS_INFO("Attempting to arm quad");
  while (ros::ok() && !(arming_client.call(arm_cmd) && arm_cmd.response.success)) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Quad armed");

  // send position commands

  pose.header.frame_id = "aero1/map";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  ros::Time start = ros::Time::now();
  ROS_INFO("Publishing (0,0,2) waypoint command for 10s");
  while (ros::ok() && !(ros::Time::now() - start > ros::Duration(10.0))) {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  start = ros::Time::now();
  pose.pose.position.x = 2;
  ROS_INFO("Publishing (2,0,2) waypoint command for 10s");
  while (ros::ok() && !(ros::Time::now() - start > ros::Duration(10.0))) {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  start = ros::Time::now();
  pose.pose.position.y = 2;
  ROS_INFO("Publishing (2,2,2) waypoint command for 10s");
  while (ros::ok() && !(ros::Time::now() - start > ros::Duration(10.0))) {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  start = ros::Time::now();
  pose.pose.position.x = 0;
  ROS_INFO("Publishing (0,2,2) waypoint command for 10s");
  while (ros::ok() && !(ros::Time::now() - start > ros::Duration(10.0))) {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  start = ros::Time::now();
  pose.pose.position.y = 0;
  ROS_INFO("Publishing (0,0,2) waypoint command for 10s");
  while (ros::ok() && !(ros::Time::now() - start > ros::Duration(10.0))) {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  // land the quad

  mavros_msgs::CommandTOL land_cmd;
  land_cmd.request.yaw = 0;
  land_cmd.request.latitude = home.geo.latitude;
  land_cmd.request.longitude = home.geo.longitude;
  land_cmd.request.altitude = home.geo.altitude;

  ROS_INFO("Sending land command");
  while (ros::ok() && !(land_client.call(land_cmd) && land_cmd.response.success)) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
