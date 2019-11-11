#ifndef INTEL_AERO_NAVIGATION_PX4_NAV_HPP_
#define INTEL_AERO_NAVIGATION_PX4_NAV_HPP_


#include <intel_aero_navigation/nav_base.h>
#include <intel_aero_navigation/mavros_uav.h>


namespace intel_aero_navigation {


class PX4Nav : public NavBase
{
  protected:
    MavrosUAV aero; // wrapper class interface to UAV

    virtual bool systemInitialized() {return aero.takeoffCommandIssued();};
    virtual void initializeSystem() {aero.takeoff();};

    virtual void sendCommand(const geometry_msgs::PoseStamped& cmd)
    {
      aero.sendLocalPositionCommand(cmd);
    };

    virtual void executeEndAction(const int action)
    {
      if (action == intel_aero_navigation::WaypointNavigationGoal::HOVER) {
        ROS_DEBUG("[PX4Nav] end action is hovering in place; no action taken");
      } else if (action == intel_aero_navigation::WaypointNavigationGoal::LAND) {
        ROS_DEBUG("[PX4Nav] end action is landing; sending land command to aero");
        aero.land();
      }
    }

  public:
    PX4Nav(std::string name, ros::NodeHandle nh_, ros::NodeHandle pnh_):
      NavBase(name, nh_, pnh_),
      aero(nh_, pnh_)
    {
    }
};


}


#endif
