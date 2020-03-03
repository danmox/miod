#include <intel_aero_navigation/vel_nav.h>
#include <nodelet/nodelet.h>


namespace intel_aero_navigation {


class VelNavNodelet : public nodelet::Nodelet
{
  protected:
    ros::NodeHandle nh, pnh;
    std::shared_ptr<VelNav> nav_ptr;

  public:
    virtual void onInit()
    {
      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      nav_ptr.reset(new VelNav(getName(), nh, pnh));
    }
};


} // namespace intel_aero_navigation


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(intel_aero_navigation::VelNavNodelet, nodelet::Nodelet);
