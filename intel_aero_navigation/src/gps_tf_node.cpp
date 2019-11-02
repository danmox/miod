#include <ros/ros.h>
#include <ros/console.h>

#include <robot_localization/navsat_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <string.h>
#include <functional>


namespace intel_aero_navigation {


using RobotLocalization::NavsatConversions::LLtoUTM;


class GPSTF
{
  private:
    ros::NodeHandle nh, pnh;
    ros::Publisher pose_pub;

    // relative time syncrhonizer for GPS, IMU messages
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>> gps_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Imu>> imu_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu> Policy;
    typedef message_filters::Synchronizer<Policy> ApproxSync;
    std::shared_ptr<ApproxSync> approx_sync;

    std::string base_frame, world_frame; // tf tree will be: world_frame -> base_link

    tf2::Transform utm_datum_tf;
    std::string datum_utm_zone;

    void syncCB(const sensor_msgs::NavSatFix::ConstPtr& gps_msg,
                const sensor_msgs::Imu::ConstPtr& imu_msg);

  public:
    GPSTF(const ros::NodeHandle&, ros::NodeHandle&);
};


template<typename T>
void getParamStrict(const ros::NodeHandle& nh, std::string param_name, T& param)
{
  if (!nh.getParam(param_name, param)) {
    ROS_FATAL("failed to get ROS param \"%s\"", param_name.c_str());
    exit(EXIT_FAILURE);
  }
}


GPSTF::GPSTF(const ros::NodeHandle& nh_, ros::NodeHandle& pnh_) :
  nh(nh_), pnh(pnh_)
{
  // node logger level

  bool debug = pnh.param("debug", false);
  if (debug)
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
      ros::console::notifyLoggerLevelsChanged();

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);

  // relative time synchronizer for GPS, IMU

  gps_sub.reset(new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh, "gps", 1));
  imu_sub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "imu", 1));
  approx_sync.reset(new ApproxSync(Policy(10), *gps_sub, *imu_sub));
  approx_sync->registerCallback(std::bind(&GPSTF::syncCB,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));

  // vital ROS params

  double datum_lat, datum_lon, datum_yaw;
  getParamStrict(pnh, "base_frame", base_frame);
  getParamStrict(pnh, "world_frame", world_frame);
  getParamStrict(pnh, "datum_lat", datum_lat);
  getParamStrict(pnh, "datum_lon", datum_lon);
  getParamStrict(pnh, "datum_yaw", datum_yaw);

  // compute datum transformation: utm -> datum

  double utm_x, utm_y;
  LLtoUTM(datum_lat, datum_lon, utm_y, utm_x, datum_utm_zone);
  ROS_DEBUG("datum UTM zone: %s", datum_utm_zone.c_str());

  utm_datum_tf.setOrigin(tf2::Vector3(utm_x, utm_y, 0.0));
  utm_datum_tf.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1), datum_yaw));
}


void GPSTF::syncCB(const sensor_msgs::NavSatFix::ConstPtr& gps_msg,
                   const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  static tf2_ros::TransformBroadcaster tf_broadcaster;
  static sensor_msgs::NavSatFix initial_gps_fix = *gps_msg;

  double utm_x, utm_y;
  std::string gps_utm_zone;
  LLtoUTM(gps_msg->latitude, gps_msg->longitude, utm_y, utm_x, gps_utm_zone);
  ROS_DEBUG("gps UTM zone: %s", gps_utm_zone.c_str());
  ROS_ASSERT(datum_utm_zone.compare(gps_utm_zone) == 0);

  // translation of utm -> base_link transformation
  tf2::Transform utm_base_link_tf;
  double relative_altitude = gps_msg->altitude - initial_gps_fix.altitude; // gps altitude in NED frame
  utm_base_link_tf.setOrigin(tf2::Vector3(utm_x, utm_y, relative_altitude));

  // rotation of utm -> base_link transformation
  tf2::Quaternion tf2_quat;
  tf2::fromMsg(imu_msg->orientation, tf2_quat);
  utm_base_link_tf.setRotation(tf2_quat);

  // utm_datum_tf is: utm -> datum (from the datum parameter)
  // utm_base_link_tf is:  utm -> base_link  (from the gps + imu message)
  // thus, datum -> base_link is: datum -> utm -> base_link

  tf2::Transform datum_base_link_tf;
  datum_base_link_tf = utm_datum_tf.inverse() * utm_base_link_tf;

  // take the latter of the two timestamps as the TF/odom timestamp
  ros::Time time_stamp = gps_msg->header.stamp > imu_msg->header.stamp ? gps_msg->header.stamp : imu_msg->header.stamp;

  // publish results to TF tree and as odom message

  geometry_msgs::TransformStamped datum_base_link_tfs;
  datum_base_link_tfs.header.stamp = time_stamp;
  datum_base_link_tfs.header.frame_id = world_frame;
  datum_base_link_tfs.child_frame_id = base_frame;
  datum_base_link_tfs.transform = tf2::toMsg(datum_base_link_tf);
  tf_broadcaster.sendTransform(datum_base_link_tfs);

  // also publish results as odom message

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = world_frame;
  pose.header.stamp = time_stamp;
  tf2::toMsg(datum_base_link_tf, pose.pose);
  // TODO covariance?
  pose_pub.publish(pose);
}


} // namespace intel_aero_navigation


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_tf_node");
  ros::NodeHandle nh, pnh("~");

  intel_aero_navigation::GPSTF gpstf(nh, pnh);

  ros::spin();

  return 0;
}
