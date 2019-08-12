#ifndef INTEL_AERO_NAVIGATION_MAVROS_UAV_H_
#define INTEL_AERO_NAVIGATION_MAVROS_UAV_H_


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>

#include <thread>
#include <mutex>
#include <atomic>


namespace intel_aero_navigation {


class MavrosUAV
{
  protected:
    ros::NodeHandle nh, pnh;
    ros::Subscriber state_sub;
    ros::ServiceClient arming_srv, mode_srv, land_srv;

    ros::Publisher local_pos_pub, local_vel_pub;
    std::mutex pub_mutex;

    std::thread landing_thread, takeoff_thread;
    std::atomic<bool> takeoff_command_issued, land_command_issued;

    mavros_msgs::State state;
    std::mutex state_mutex;

    void takeoffThread();
    void landingThread();

  public:
    MavrosUAV(ros::NodeHandle, ros::NodeHandle);
    ~MavrosUAV();

    void sendLocalPositionCommand(const geometry_msgs::PoseStamped&);
    void sendLocalVelocityCommand(const geometry_msgs::Twist&);
    void takeoff();
    void land();

    mavros_msgs::State getState();
    bool takeoffCommandIssued() const;
    bool landCommandIssued() const;

    void stateCB(const mavros_msgs::State::ConstPtr&);
    void homeCB(const mavros_msgs::HomePosition::ConstPtr&);
};


} // namespace intel_aero_navigation


#endif
