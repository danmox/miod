#ifndef INTEL_AERO_NAVIGATION_MAVROS_UAV_H_
#define INTEL_AERO_NAVIGATION_MAVROS_UAV_H_


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>

#include <thread>


namespace intel_aero_navigation {


class MavrosUAV
{
  protected:
    ros::NodeHandle nh, pnh;
    ros::Subscriber state_sub, home_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_srv, mode_srv, land_srv;

    std::thread landing_thread, takeoff_thread;
    bool takeoff_command_issued, land_command_issued;

    mavros_msgs::State state;
    mavros_msgs::HomePosition home_position;

    void takeoffThread(const geometry_msgs::PoseStamped);
    void landingThread();

  public:
    MavrosUAV(ros::NodeHandle, ros::NodeHandle);

    void sendLocalPositionCommand(const geometry_msgs::PoseStamped&);
    void takeoff(const geometry_msgs::PoseStamped);
    void land();

    // get functions
    mavros_msgs::State getState() const { return state; }
    mavros_msgs::HomePosition getHomePosition() const { return home_position; }
    bool takeoffCommandIssued() const { return takeoff_command_issued; }
    bool landCommandIssued() const { return land_command_issued; }

    void stateCB(const mavros_msgs::State::ConstPtr&);
    void homeCB(const mavros_msgs::HomePosition::ConstPtr&);
};


}


#endif
