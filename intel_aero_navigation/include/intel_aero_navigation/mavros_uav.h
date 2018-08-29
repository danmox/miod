#ifndef INTEL_AERO_NAVIGATION_MAVROS_UAV_H_
#define INTEL_AERO_NAVIGATION_MAVROS_UAV_H_


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>


namespace intel_aero_navigation {


class MavrosUAV
{
  protected:
    ros::NodeHandle nh, pnh;
    ros::Subscriber state_sub, home_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_srv, mode_srv, land_srv;

    mavros_msgs::State state;
    mavros_msgs::HomePosition home_position;

  public:
    MavrosUAV(ros::NodeHandle, ros::NodeHandle);

    void takeoff(const geometry_msgs::PoseStamped);
    void sendLocalPositionCommand(const geometry_msgs::PoseStamped&);
    void land(mavros_msgs::CommandTOL& cmd);

    void stateCB(const mavros_msgs::State::ConstPtr&);
    void homeCB(const mavros_msgs::HomePosition::ConstPtr&);
};


}


#endif
