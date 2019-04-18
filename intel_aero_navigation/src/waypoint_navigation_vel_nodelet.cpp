#include <intel_aero_navigation/waypoint_navigation_vel.h>
#include <nodelet/nodelet.h>

namespace intel_aero_navigation {


class WaypointNavigationVelNodelet : public nodelet::Nodelet
{
  protected:
    ros::NodeHandle nh, pnh;
    std::shared_ptr<WaypointNavigationVel> nav_ptr;

  public:
    virtual void onInit()
    {
      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      nav_ptr.reset(new WaypointNavigationVel(getName(), nh, pnh));
    }
};


} // namespace intel_aero_navigation


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(intel_aero_navigation::WaypointNavigationVelNodelet, nodelet::Nodelet);
