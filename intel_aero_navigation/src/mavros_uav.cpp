#include <intel_aero_navigation/mavros_uav.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robot_localization/navsat_conversions.h>


namespace intel_aero_navigation {


#define MU_INFO(fmt, ...) ROS_INFO("[MavrosUAV] " fmt, ##__VA_ARGS__)
#define MU_WARN(fmt, ...) ROS_WARN("[MavrosUAV] " fmt, ##__VA_ARGS__)
#define MU_DEBUG(fmt, ...) ROS_DEBUG("[MavrosUAV] " fmt, ##__VA_ARGS__)


MavrosUAV::MavrosUAV(ros::NodeHandle nh_, ros::NodeHandle pnh_) :
  nh(nh_),
  pnh(pnh_),
  tf2_listener(tf2_buffer),
  takeoff_command_issued(false),
  land_command_issued(false)
{
  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &MavrosUAV::stateCB, this);
  local_vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);
  arming_srv = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  mode_srv = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_srv = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

  if (!nh.getParam("local_frame", local_frame)) {
    MU_WARN("using default UAV local_frame: %s", "aero1/odom");
    local_frame = "aero1/odom";
  }
}


MavrosUAV::~MavrosUAV()
{
  // keep the class from being destroyed while detached threads are running
  if (takeoff_thread.joinable()) takeoff_thread.join();
  if (landing_thread.joinable()) landing_thread.join();
}


void MavrosUAV::stateCB(const mavros_msgs::State::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(state_mutex);
  state = *msg;
}

mavros_msgs::State MavrosUAV::getState()
{
  std::lock_guard<std::mutex> lock(state_mutex);
  return state;
}


bool MavrosUAV::takeoffCommandIssued() const
{
  return takeoff_command_issued.load(std::memory_order_relaxed);
}


bool MavrosUAV::landCommandIssued() const
{
  return land_command_issued.load(std::memory_order_relaxed);
}


// NOTE: during takeoff local_pos_pub, local_vel_pub are blocked via the
// pub_mutex held by this thread; thus, any calls to sendLocal***Command(...)
// will have no effect (i.e. not interfere). The intended behavior is that
// calling threads immediately begin publishing desired commands after invoking
// takeoff() so that control of the UAV is gracefully transfered back to the
// governing program after takeoff has been completed and pub_mutex released.
void MavrosUAV::takeoffThread()
{
  ros::Rate rate(10.0); // setpoint publishing rate MUST be faster than 2Hz

  // make FCU connection
  ros::Time start = ros::Time::now();
  // TODO get state could still be default constructed here (but even if it is
  // it will be false so it shouldn't matter)
  while (ros::ok() && !getState().connected) {
    if ((ros::Time::now() - start).toSec() > 2.0) {
      MU_WARN("waiting for FCU connection");
      start = ros::Time::now();
    }
    ros::spinOnce();
    rate.sleep();
  }
  MU_INFO("FCU connection made");

  // prevent any other messages from being published for the remainder of takeoff
  std::lock_guard<std::mutex> lock(pub_mutex);

  // vertical flight speed limit governed by px4 param MPC_Z_VEL_MAX_UP
  geometry_msgs::Twist cmd;
  cmd.linear.z = 0.2;

  // send a few setpoints before starting
  MU_INFO("publishing setpoints before liftoff");
  for (int i = 30; ros::ok() && i > 0; --i) {
    local_vel_pub.publish(cmd);
    ros::spinOnce();
    rate.sleep();
  }

  // set the quad in offboard mode
  mavros_msgs::SetMode mode_msg;
  mode_msg.request.custom_mode = "OFFBOARD";
  while (ros::ok() && !(mode_srv.call(mode_msg) && mode_msg.response.mode_sent)) {
    local_vel_pub.publish(cmd); // setpoints must continually be published
    ros::spinOnce();
    rate.sleep();
  }
  MU_INFO("offboard enabled");

  // arm the quad for takeoff
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  while (ros::ok() && !(arming_srv.call(arm_cmd) && arm_cmd.response.success)) {
    local_vel_pub.publish(cmd); // setpoints must continually be published
    ros::spinOnce();
    rate.sleep();
  }
  MU_INFO("quad armed");

  // wait for the system to report the quad is active (i.e. 4: in the air)
  while (ros::ok() && getState().system_status != 4) {
    local_vel_pub.publish(cmd); // setpoints must continually be published
    ros::spinOnce();
    rate.sleep();
  }
  MU_INFO("takeoff complete");

  MU_DEBUG("exiting takeoff thread");
}


// see note on takeoffThread()
void MavrosUAV::takeoff()
{
  if (!takeoff_thread.joinable()) {
    MU_DEBUG("spawning takeoff thread");
    takeoff_thread = std::thread(&MavrosUAV::takeoffThread, this);
    land_command_issued = false; // false until land() is called
    takeoff_command_issued = true; // true until land() is called
  }
}


void MavrosUAV::landingThread()
{

  ros::Rate rate(10.0);
  geometry_msgs::PoseStamped pose;

  // make service call (this only starts the quad's descent)
  mavros_msgs::CommandTOL cmd;
  while (ros::ok() && !(land_srv.call(cmd) && cmd.response.success)) {
    local_pos_pub.publish(pose); // dummy msg keeps quad in offboard mode
    ros::spinOnce();
    rate.sleep();
  }
  MU_INFO("land command sent");

  // wait for quad to land (state becomes 3 after landing has been detected)
  while (ros::ok() && getState().system_status != 3) {
    local_pos_pub.publish(pose); // dummy msg keeps quad in offboard mode
    ros::spinOnce();
    rate.sleep();
  }
  MU_INFO("landing complete");

  MU_DEBUG("exiting landing thread");
}


void MavrosUAV::land()
{
  if (!landing_thread.joinable()) {
    MU_DEBUG("landing thread started");
    landing_thread = std::thread(&MavrosUAV::landingThread, this);
    land_command_issued = true; // true until takeoff(...) is called
    takeoff_command_issued = false; // false until takeoff(...) is called
  }
}


// TODO should there be some kind of multiplexing? velocity & position commands
// could be sent at the same time
void MavrosUAV::sendLocalVelocityCommand(const geometry_msgs::Twist& cmd)
{
  static ros::Time last_pub(0); // time since the last message was published

  // publish is thread safe but we don't want possibly conflicting messages being
  // sent to the quad while it is trying to take off
  if (pub_mutex.try_lock()) {

    // limit publishing to 1000Hz to prevent overloading the connection
    if ((ros::Time::now() - last_pub).toSec() > 1e-3) { // 1ms
      local_vel_pub.publish(cmd);
      last_pub = ros::Time::now();
    }

    pub_mutex.unlock();
  } else {
    MU_DEBUG("can't publish local position command: publisher busy");
  }
}


void MavrosUAV::sendLocalPositionCommand(const geometry_msgs::PoseStamped& cmd)
{
  static ros::Time last_pub(0); // time since the last message was published

  // transform the position command if it is not in the local frame

  // fetch latest transformation cmd.header.frame_id -> local_frame
  geometry_msgs::TransformStamped tfs;
  try {
    tfs = tf2_buffer.lookupTransform(local_frame, cmd.header.frame_id, ros::Time(0));
  } catch (tf2::TransformException &ex) {
    MU_WARN("unable to publish local position command: %s",ex.what());
    return;
  }

  // apply transform to pose message
  geometry_msgs::PoseStamped local_pose;
  tf2::doTransform(cmd, local_pose, tfs);

  // publish is thread safe but we don't want possibly conflicting messages being
  // sent to the quad while it is trying to take off
  if (pub_mutex.try_lock()) {

    // limit publishing to 1000Hz to prevent overloading the connection
    if ((ros::Time::now() - last_pub).toSec() > 1e-3) { // 1ms
      local_pos_pub.publish(local_pose);
      last_pub = ros::Time::now();
    }

    pub_mutex.unlock();
  } else {
    MU_DEBUG("can't publish local position command: publisher busy");
  }
}


using RobotLocalization::NavsatConversions::UTMtoLL;


void MavrosUAV::sendGlobalPositionCommand(const geometry_msgs::PoseStamped& cmd)
{
  static ros::Time last_pub(0); // time since the last message was published

  // fetch latest transform
  geometry_msgs::TransformStamped tfs;
  try {
    tfs = tf2_buffer.lookupTransform("utm", cmd.header.frame_id, ros::Time(0));
  } catch (tf2::TransformException &ex) {
    MU_WARN("unable to publish global position command: %s",ex.what());
    return;
  }

  // apply transform to pose message
  geometry_msgs::PoseStamped utm_pose;
  tf2::doTransform(cmd, utm_pose, tfs);

  // convert utm to LL
  double lat, lon;
  std::string utm_zone;
  UTMtoLL(utm_pose.pose.position.y, utm_pose.pose.position.x, utm_zone, lat, lon);

  // populate command message
  mavros_msgs::GlobalPositionTarget gpt;
  gpt.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT; // altitude measured relative to home position
  gpt.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX
    | mavros_msgs::GlobalPositionTarget::IGNORE_VY
    | mavros_msgs::GlobalPositionTarget::IGNORE_VZ
    | mavros_msgs::GlobalPositionTarget::IGNORE_AFX
    | mavros_msgs::GlobalPositionTarget::IGNORE_AFY
    | mavros_msgs::GlobalPositionTarget::IGNORE_AFZ
    | mavros_msgs::GlobalPositionTarget::IGNORE_YAW
    | mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
  gpt.latitude = lat;
  gpt.longitude = lon;
  gpt.altitude = cmd.pose.position.z; // meters above home

  // publish is thread safe but we don't want possibly conflicting messages being
  // sent to the quad while it is trying to take off
  if (pub_mutex.try_lock()) {

    // limit publishing to 1000Hz to prevent overloading the connection
    if ((ros::Time::now() - last_pub).toSec() > 1e-3) { // 1ms
      global_pos_pub.publish(gpt);
      last_pub = ros::Time::now();
    }

    pub_mutex.unlock();
  } else {
    MU_DEBUG("can't publish global position command: publisher busy");
  }
}


} // namespace intel_aero_navigation
