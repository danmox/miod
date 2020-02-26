#ifndef INTEL_AERO_NAVIGATION_GAZEBO_VEL_NAV_H_
#define INTEL_AERO_NAVIGATION_GAZEBO_VEL_NAV_H_


#include <intel_aero_navigation/vel_nav.h>


namespace intel_aero_navigation {


class GazeboVelNav : public VelNav
{
  protected:
    virtual bool systemInitialized();
    virtual void initializeSystem();

  public:
    GazeboVelNav(std::string, ros::NodeHandle, ros::NodeHandle);

};


} // namespace intel_aero_navigation


#endif
