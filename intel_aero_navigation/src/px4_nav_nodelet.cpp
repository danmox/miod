#include <intel_aero_navigation/px4_nav.hpp>
#include <nodelet/nodelet.h>


namespace intel_aero_navigation {


class PX4NavNodelet : public nodelet::Nodelet
{
  protected:
    ros::NodeHandle nh, pnh;
    std::shared_ptr<PX4Nav> nav_ptr;

  public:
    virtual void onInit()
    {
      nh = getNodeHandle(); // single threaded
      pnh = getPrivateNodeHandle(); // single threaded

      nav_ptr.reset(new PX4Nav(getName(), nh, pnh));
    }
};


} // namespace intel_aero_navigation


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(intel_aero_navigation::PX4NavNodelet, nodelet::Nodelet);
