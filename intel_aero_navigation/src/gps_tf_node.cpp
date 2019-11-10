#include <ros/ros.h>
#include <ros/console.h>

#include <robot_localization/navsat_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
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

    // tf tree will be: world_frame -> odom_frame -> base_frame
    std::string base_frame, odom_frame, world_frame;

    double datum_lat, datum_lon, datum_yaw;
    tf2::Transform datum_utm_tf;
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
  approx_sync->registerCallback(std::bind(&GPSTF::syncCB, this, std::placeholders::_1, std::placeholders::_2));

  // vital ROS params

  getParamStrict(pnh, "base_frame", base_frame);
  getParamStrict(pnh, "odom_frame", odom_frame);
  getParamStrict(pnh, "world_frame", world_frame);
  getParamStrict(pnh, "datum_lat", datum_lat);
  getParamStrict(pnh, "datum_lon", datum_lon);
  getParamStrict(pnh, "datum_yaw", datum_yaw);

  // compute transformation: datum (world_frame) -> utm

  double utm_x, utm_y;
  LLtoUTM(datum_lat, datum_lon, utm_y, utm_x, datum_utm_zone);
  ROS_DEBUG("datum UTM zone: %s", datum_utm_zone.c_str());

  datum_utm_tf.setOrigin(tf2::Vector3(utm_x, utm_y, 0.0));
  datum_utm_tf.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1), datum_yaw));
}


void GPSTF::syncCB(const sensor_msgs::NavSatFix::ConstPtr& gps_msg,
                   const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  static tf2_ros::Buffer tf2_buffer;
  static tf2_ros::TransformListener tf2_listener(tf2_buffer);
  static tf2_ros::TransformBroadcaster tf_broadcaster;

  // fetch transformation: base_frame -> odom_frame published by mavros

  geometry_msgs::TransformStamped base_odom_tf_msg;
  try {
    base_odom_tf_msg = tf2_buffer.lookupTransform(odom_frame, base_frame, ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "unable to fetch transform: base (%s) -> odom (%s): %s", base_frame.c_str(), odom_frame.c_str(), ex.what());
    return;
  }

  tf2::Transform base_odom_tf;
  tf2::fromMsg(base_odom_tf_msg.transform, base_odom_tf);

  // compute transformation: base_frame -> utm

  double utm_x, utm_y;
  std::string gps_utm_zone;
  LLtoUTM(gps_msg->latitude, gps_msg->longitude, utm_y, utm_x, gps_utm_zone);

  ROS_DEBUG("gps UTM zone: %s", gps_utm_zone.c_str());
  if (datum_utm_zone.compare(gps_utm_zone) != 0) {
    ROS_WARN_THROTTLE(1.0, "Won't publish transform: UTM zones not the same! GPS: {lat: %f, lon: %f, zone: %s}, datum: {lat: %f, lon: %f, zone: %s}", gps_msg->latitude, gps_msg->longitude, gps_utm_zone.c_str(), datum_lat, datum_lon, datum_utm_zone.c_str());
    return;
  }

  tf2::Quaternion base_utm_quat;
  tf2::fromMsg(imu_msg->orientation, base_utm_quat);

  tf2::Transform base_utm_tf;
  base_utm_tf.setOrigin(tf2::Vector3(utm_x, utm_y, base_odom_tf_msg.transform.translation.z));
  base_utm_tf.setRotation(base_utm_quat);

  // compute transformation: odom_frame -> world_frame

  tf2::Transform base_datum_tf = datum_utm_tf.inverse() * base_utm_tf;
  tf2::Transform odom_datum_tf = base_datum_tf * base_odom_tf.inverse();

  // publish results to TF tree and as odom message

  ros::Time time_stamp = gps_msg->header.stamp > imu_msg->header.stamp ? gps_msg->header.stamp : imu_msg->header.stamp;

  geometry_msgs::TransformStamped odom_datum_tf_msg;
  odom_datum_tf_msg.header.stamp = time_stamp;
  odom_datum_tf_msg.header.frame_id = world_frame;
  odom_datum_tf_msg.child_frame_id = odom_frame;
  odom_datum_tf_msg.transform = tf2::toMsg(odom_datum_tf);

  tf_broadcaster.sendTransform(odom_datum_tf_msg);

  // publish pose of robot in the shared world_frame

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = world_frame;
  pose.header.stamp = time_stamp;
  tf2::toMsg(base_datum_tf, pose.pose);
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
