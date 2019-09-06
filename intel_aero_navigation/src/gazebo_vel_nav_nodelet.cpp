#include <intel_aero_navigation/gazebo_vel_nav.h>
#include <nodelet/nodelet.h>


namespace intel_aero_navigation {


class GazeboVelNavNodelet : public nodelet::Nodelet
{
  protected:
    ros::NodeHandle nh, pnh;
    std::shared_ptr<GazeboVelNav> nav_ptr;

  public:
    virtual void onInit()
    {
      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      nav_ptr.reset(new GazeboVelNav(getName(), nh, pnh));
    }
};


} // namespace intel_aero_navigation


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(intel_aero_navigation::GazeboVelNavNodelet, nodelet::Nodelet);
