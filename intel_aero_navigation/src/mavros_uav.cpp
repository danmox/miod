#include <intel_aero_navigation/mavros_uav.h>


namespace intel_aero_navigation {


MavrosUAV::MavrosUAV(ros::NodeHandle nh_, ros::NodeHandle pnh_) :
  nh(nh_),
  pnh(pnh_)
{
  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &MavrosUAV::stateCB, this);
  home_sub = nh.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 10, &MavrosUAV::homeCB, this);
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  arming_srv = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  mode_srv = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_srv = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
}


void MavrosUAV::stateCB(const mavros_msgs::State::ConstPtr& msg)
{
  state = *msg;
}


void MavrosUAV::homeCB(const mavros_msgs::HomePosition::ConstPtr& msg)
{
  home_position = *msg;
}


void MavrosUAV::takeoff(const geometry_msgs::PoseStamped cmd)
{
  ros::Rate rate(10.0); //setpoint publishing rate MUST be faster than 2Hz

  // make FCU connection

  int wait_count = 0;
  while (ros::ok() && !state.connected) {
    ros::spinOnce();
    rate.sleep();
    wait_count++;
    if (wait_count > 40) {
      ROS_INFO("[MavrosUAV] waiting for FCU connection");
      wait_count = 0;
    }
  }
  ROS_INFO("[MavrosUAV] FCU connection made");

  // send a few setpoints before starting

  ROS_INFO("[MavrosUAV] publishing setpoints before liftoff");
  for (int i = 50; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(cmd);
    ros::spinOnce();
    rate.sleep();
  }

  // set the quad in offboard mode

  mavros_msgs::SetMode mode_msg;
  mode_msg.request.custom_mode = "OFFBOARD";

  while (ros::ok() && !(mode_srv.call(mode_msg) && mode_msg.response.mode_sent)) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("[MavrosUAV] offboard enabled");

  // arm the quad for takeoff

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  while (ros::ok() && !(arming_srv.call(arm_cmd) && arm_cmd.response.success)) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("[MavrosUAV] quad armed");
}


void MavrosUAV::land(mavros_msgs::CommandTOL& cmd)
{
  ros::Rate rate(10.0);

  ROS_INFO("[MavrosUAV] sending land command");
  while (ros::ok() && !(land_srv.call(cmd) && cmd.response.success)) {
    ros::spinOnce();
    rate.sleep();
  }
}


void MavrosUAV::sendLocalPositionCommand(const geometry_msgs::PoseStamped& cmd)
{
  local_pos_pub.publish(cmd);
}


}
