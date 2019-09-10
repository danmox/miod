#ifndef INTEL_AERO_NAVIGATION_GAZEBO_VEL_NAV_H_
#define INTEL_AERO_NAVIGATION_GAZEBO_VEL_NAV_H_


#include <intel_aero_navigation/nav_base.h>


namespace intel_aero_navigation {


// TODO remove gazebo components
class GazeboVelNav : public NavBase
{
  protected:
    ros::ServiceClient set_client, get_client;
    ros::Publisher vel_pub;

    double linear_vel_des, angular_vel_des;

    virtual bool systemInitialized();
    virtual void initializeSystem();
    virtual void sendCommand(const geometry_msgs::PoseStamped& goal);
    virtual void executeEndAction(const int action);

  public:
    GazeboVelNav(std::string, ros::NodeHandle, ros::NodeHandle);

};


} // namespace intel_aero_navigation


#endif
