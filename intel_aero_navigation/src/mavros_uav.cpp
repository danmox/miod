#include <intel_aero_navigation/mavros_uav.h>


namespace intel_aero_navigation {


#define MU_INFO(fmt, ...) ROS_INFO("[MavrosUAV] " fmt, ##__VA_ARGS__)
#define MU_WARN(fmt, ...) ROS_WARN("[MavrosUAV] " fmt, ##__VA_ARGS__)
#define MU_DEBUG(fmt, ...) ROS_DEBUG("[MavrosUAV] " fmt, ##__VA_ARGS__)


MavrosUAV::MavrosUAV(ros::NodeHandle nh_, ros::NodeHandle pnh_) :
  nh(nh_),
  pnh(pnh_),
  takeoff_command_issued(false),
  land_command_issued(false)
{
  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &MavrosUAV::stateCB, this);
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  arming_srv = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  mode_srv = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_srv = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
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


void MavrosUAV::takeoffThread(const geometry_msgs::PoseStamped cmd)
{
  ros::Rate rate(10.0); // setpoint publishing rate MUST be faster than 2Hz

  // make FCU connection
  ros::Time start = ros::Time::now();
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

  // send a few setpoints before starting
  MU_INFO("publishing setpoints before liftoff");
  for (int i = 30; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(cmd);
    ros::spinOnce();
    rate.sleep();
  }

  // set the quad in offboard mode
  mavros_msgs::SetMode mode_msg;
  mode_msg.request.custom_mode = "OFFBOARD";
  while (ros::ok() && !(mode_srv.call(mode_msg) && mode_msg.response.mode_sent)) {
    local_pos_pub.publish(cmd); // setpoints must continually be published
    ros::spinOnce();
    rate.sleep();
  }
  MU_INFO("offboard enabled");

  // arm the quad for takeoff
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  while (ros::ok() && !(arming_srv.call(arm_cmd) && arm_cmd.response.success)) {
    local_pos_pub.publish(cmd); // setpoints must continually be published
    ros::spinOnce();
    rate.sleep();
  }
  MU_INFO("quad armed");

  // wait for the system to report the quad is active (i.e. 4: in the air)
  while (ros::ok() && getState().system_status != 4) {
    local_pos_pub.publish(cmd);
    ros::spinOnce();
    rate.sleep();
  }
  MU_INFO("takeoff complete");

  MU_DEBUG("exiting takeoff thread");
}


void MavrosUAV::takeoff(const geometry_msgs::PoseStamped pose)
{
  if (!takeoff_thread.joinable()) {
    MU_DEBUG("spawning takeoff thread");
    takeoff_thread = std::thread(&MavrosUAV::takeoffThread, this, pose);
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


void MavrosUAV::sendLocalPositionCommand(const geometry_msgs::PoseStamped& cmd)
{
  // publish is thread safe but we don't want possibly conflicting messages being
  // sent to the quad while it is trying to take off (doesn't matter for landing)
  if (pub_mutex.try_lock()) {
    local_pos_pub.publish(cmd);
    pub_mutex.unlock();
  } else {
    MU_DEBUG("can't publish local position command: publisher busy");
  }
}


} // namespace intel_aero_navigation
