#include <intel_aero_navigation/waypoint_navigation.h>
#include <nodelet/nodelet.h>

namespace intel_aero_navigation {


class WaypointNavigationNodelet : public nodelet::Nodelet
{
  protected:
    ros::NodeHandle nh, pnh;
    std::shared_ptr<WaypointNavigation> nav_ptr;

  public:
    virtual void onInit()
    {
      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      nav_ptr.reset(new WaypointNavigation(getName(), nh, pnh));
    }
};


} // namespace intel_aero_navigation


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(intel_aero_navigation::WaypointNavigationNodelet, nodelet::Nodelet);
