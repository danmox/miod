#include <intel_aero_navigation/nav_base.h>

#include <nodelet/nodelet.h>
#include <ros/console.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>

#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <queue>
#include <cmath>


namespace intel_aero_navigation {


#define NB_INFO(fmt, ...) ROS_INFO("[NavBase] " fmt, ##__VA_ARGS__)
#define NB_WARN(fmt, ...) ROS_WARN("[NavBase] " fmt, ##__VA_ARGS__)
#define NB_DEBUG(fmt, ...) ROS_DEBUG("[NavBase] " fmt, ##__VA_ARGS__)
#define NB_FATAL(fmt, ...) ROS_FATAL("[NavBase] " fmt, ##__VA_ARGS__)
#define NB_ERROR(fmt, ...) ROS_ERROR("[NavBase] " fmt, ##__VA_ARGS__)


template<typename T>
void getParamStrict(const ros::NodeHandle& nh, std::string param_name, T& param)
{
  if (!nh.getParam(param_name, param)) {
    NB_FATAL("failed to get ROS param \"%s\"", param_name.c_str());
    exit(EXIT_FAILURE);
  }
}


NavBase::NavBase(std::string name, ros::NodeHandle& nh_, ros::NodeHandle& pnh_):
  costmap_ptr(nullptr),
  tf2_listener(tf2_buffer),
  nav_server(nh_, name, false),
  nh(nh_),
  pnh(pnh_)
{
  nav_server.registerGoalCallback(std::bind(&NavBase::goalCB, this));
  nav_server.registerPreemptCallback(std::bind(&NavBase::preemptCB, this));
  nav_server.start();

  getParamStrict(pnh, "odom_frame", odom_frame);
  getParamStrict(pnh, "position_tol", position_tol);
  getParamStrict(pnh, "heading_tol", heading_tol);

  // set nodelet verbosity to debug if desired
  bool debug = false;
  if (pnh.getParam("debug", debug) && debug)
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
      ros::console::notifyLoggerLevelsChanged();

  costmap_sub = nh.subscribe("costmap", 10, &NavBase::costmapCB, this);
  odom_sub = nh.subscribe("odom", 10, &NavBase::odomCB, this);
  path_pub = nh.advertise<nav_msgs::Path>("path", 10);
}

NavBase::~NavBase()
{
  if (costmap_ptr)
    delete costmap_ptr;
}

// TODO also have a map updates sub?
void NavBase::costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  // NOTE: it is assumed the pose of the robot is given with respect to the
  // same frame as the costmap (i.e. no transformations need to take place)
  if (!costmap_ptr) {
    costmap_ptr = new Costmap(msg);
    NB_INFO("received first costmap with resolution %.f and dimensions %d x %d", costmap_ptr->resolution, costmap_ptr->w, costmap_ptr->h);
  } else {
    costmap_ptr->insertMap(msg);
    NB_DEBUG("inserted costmap with resolution %.f and dimensions %d x %d", costmap_ptr->resolution, costmap_ptr->w, costmap_ptr->h);
  }

  // nothing more to do if any of the following are satisfied:
  // 1) the server is inactive
  // 2) odom is unavailable (can't check for collisions)
  // 4) the path is empty (don't need to check for collisions)
  if (!nav_server.isActive() || !robot_pose || waypoints.empty())
    return;

  // if the robot is not inside the costmap, find the point along its path when
  // it enters the map
  grid_mapping::Point first_point(robot_pose->pose.position);
  if (!costmap_ptr->inBounds(first_point))
    first_point = costmap_ptr->bbxIntersection(waypoints.front(), first_point);

  // check for collisions between the robot and its current waypoint
  // TODO should the rest of the waypoints also be checked for collisions?
  if (!obstacleFree(costmap_ptr->rayCast(first_point, waypoints.front()))) {
    grid_mapping::Point p2(waypoints.front());
    ROS_INFO_STREAM("[NavBase] collision detected between " << first_point << " and " << p2);
    NB_INFO("replanning");
    planPath();
  }

  return;
}


void NavBase::goalCB()
{
  // abort any existing goals
  if (nav_server.isActive()) {
    NB_INFO("goal aborted");
    nav_server.setAborted();
  }

  auto action_goal = nav_server.acceptNewGoal();
  NB_INFO("accepted new goal");

  // cannot plan without the current position of the quad
  if (!robot_pose) {
    NB_ERROR("no odom! aborting goal");
    nav_server.setAborted();
    return;
  }

  // cannot plan without a costmap
  if (!costmap_ptr) {
    NB_ERROR("no costmap! aborting goal");
    nav_server.setAborted();
    return;
  }

  if (nav_server.isPreemptRequested()) {
    NB_INFO("goal preempted");
    nav_server.setPreempted();
    return;
  }

  // convert waypoints to odom_frame (while planning must happen in the
  // costmap frame, local position commands sent to the px4 are interpreted as
  // being in odom_frame so we convert them here to be ready for future
  // publication)
  geometry_msgs::PoseStamped wp;
  wp.header = action_goal->header;
  wp.header.stamp = ros::Time(0); // gets the latest available transform
  waypoints.clear();
  for (auto& pose : action_goal->waypoints) {
    wp.pose = pose;
    try {
      tf2_buffer.transform(wp, odom_frame);
    } catch (tf2::TransformException &e) {
      NB_ERROR("could not transform waypoint from source frame \"%s\" to odom_frame \"%s\":", wp.header.frame_id.c_str(), odom_frame.c_str());
      NB_ERROR("%s", e.what());
      nav_server.setAborted();
      return;
    }
    waypoints.push_back(wp);
  }
  end_action = action_goal->end_action;

  planPath();

  NB_DEBUG("waypoints are:");
  for (const auto& wp : waypoints)
    NB_DEBUG("{%.2f, %.2f, %.2f}", wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
}


// takes the current set of waypoints and adds intermediate waypoints, if
// necessary, to ensure obstacle free navigation
//
// assumptions:
// 1) robot_pose is valid
// 1) the costmap has been initialized
void NavBase::planPath()
{
  // prepare waypoints:
  // 1) for planning the waypoints must be transformed to the costmap frame
  // 2) waypoints must be inside of the costmap
  // 3) waypoints must be in visitable space
  // 4) for planning convenience add the robot pose as the first waypoint (to be
  //    removed later)
  std::deque<geometry_msgs::PoseStamped> costmap_wps(waypoints);
  costmap_wps.push_front(*robot_pose);
  costmap_wps[0].header.stamp = ros::Time(0);
  bool abort_goal = false;
  for (auto wp_it = costmap_wps.begin(); wp_it != costmap_wps.end() ; ++wp_it) {

    // transform waypoints from odom_frame to the costmap frame
    try {
      tf2_buffer.transform(*wp_it, costmap_ptr->frame_id);
    } catch (tf2::TransformException &e) {
      NB_ERROR("could not transform waypoint from source frame \"%s\" to the costmap frame \"%s\":", wp_it->header.frame_id.c_str(), costmap_ptr->frame_id.c_str());
      NB_ERROR("%s", e.what());
      abort_goal = true;
      break;
    }

    // ensure waypoint is inside of costmap
    if (!costmap_ptr->inBounds(*wp_it)) {
      NB_INFO("waypoint out of bounds of costmap: expanding costmap");
      grid_mapping::Point origin, corner;
      origin.x = std::min(wp_it->pose.position.x, costmap_ptr->origin.x);
      origin.y = std::min(wp_it->pose.position.y, costmap_ptr->origin.y);
      corner.x = std::max(wp_it->pose.position.x, costmap_ptr->origin.x);
      corner.y = std::max(wp_it->pose.position.y, costmap_ptr->origin.y);
      costmap_ptr->expandMap(origin, corner);
    }

    // check if waypoints are visitable
    if (costmap_ptr->data[costmap_ptr->positionToIndex(*wp_it)] > 0 && wp_it != costmap_wps.begin()) {
      NB_ERROR("waypoint {%.1f,%.1f} is in occupied space", wp_it->pose.position.x, wp_it->pose.position.y);
      abort_goal = true;
      break;
    }
  }

  if (abort_goal) {
    NB_ERROR("aborting current goal");
    nav_server.setAborted();
    return;
  }

  // add additional points, if necessary, to list of waypoints to visit
  for (auto cwp_it = ++costmap_wps.begin(); cwp_it != costmap_wps.end(); ++cwp_it) {

    // minimal A* path in costmap indices
    std::vector<int> path_inds = AStar(*(cwp_it-1), *cwp_it);

    if (path_inds.size() < 3)
      continue;

    for (auto ind_it = path_inds.begin()+1; ind_it+1 != path_inds.end(); ++ind_it) {
      grid_mapping::Point pt = costmap_ptr->indexToPosition(*ind_it);
      geometry_msgs::PoseStamped intermediate_waypoint;
      intermediate_waypoint.header = cwp_it->header;
      intermediate_waypoint.pose.position.x = pt.x;
      intermediate_waypoint.pose.position.y = pt.y;
      intermediate_waypoint.pose.position.z = cwp_it->pose.position.z;
      costmap_wps.insert(cwp_it, intermediate_waypoint);
    }
  }

  // remove robot pose added for planning convenience
  costmap_wps.pop_front();

  // transform waypoints from the costmap frame back to odom_frame for execution
  for (geometry_msgs::PoseStamped& cwp : costmap_wps) {
    try {
      tf2_buffer.transform(cwp, odom_frame);
    } catch (tf2::TransformException &e) {
      NB_ERROR("could not transform costmap waypoint from frame \"%s\" to odom_frame \"%s\":", cwp.header.frame_id.c_str(), odom_frame.c_str());
      NB_ERROR("%s", e.what());
      NB_ERROR("aborting current goal");
      nav_server.setAborted();
      return;
    }
  }
  waypoints = costmap_wps;

  // visualize path in rviz
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = odom_frame;
  path_msg.header.stamp = ros::Time::now();
  path_msg.poses.push_back(*robot_pose);
  path_msg.poses.insert(path_msg.poses.end(), waypoints.begin(), waypoints.end());
  path_pub.publish(path_msg);
}


bool NavBase::obstacleFree(const std::vector<int>& cells) const
{
  return std::none_of(cells.begin(), cells.end(), [this](int i){return costmap_ptr->data[i] > 0;});
}


void NavBase::preemptCB()
{
  if (nav_server.isActive()) {
    NB_INFO("goal aborted");
    nav_server.setAborted();
  } else {
    NB_INFO("goal preempted");
    nav_server.setPreempted();
  }

  // TODO set waypoint to current state?
}


// TODO the heading of the waypoints are ignored
// actual tracking of the waypoints happens here
void NavBase::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  robot_pose.reset(new geometry_msgs::PoseStamped(pose));

  // by design one waypoint should always be left in the deque; an empty
  // waypoints deque means no goal has been received yet
  if (waypoints.empty())
    return;

  // to be implemented by inheriting class
  if (!systemInitialized())
    initializeSystem();

  geometry_msgs::PoseStamped curr_wp = waypoints.front();
  geometry_msgs::Point robot_pt = robot_pose->pose.position;

  // heading error
  double yaw_d = atan2(curr_wp.pose.position.y - robot_pt.y,
                       curr_wp.pose.position.x - robot_pt.x);
  double yaw = tf2::getYaw(robot_pose->pose.orientation);
  double yaw_error = yaw_d - yaw;
  if (yaw_error > 2.0*M_PI) // TODO should be >=?
    yaw_error -= 2.0*M_PI;
  if (yaw_error < -2.0*M_PI)
    yaw_error += 2.0*M_PI;

  // position error
  double pos_error = sqrt(pow(curr_wp.pose.position.x - robot_pt.x, 2.0) +
                          pow(curr_wp.pose.position.y - robot_pt.y, 2.0) +
                          pow(curr_wp.pose.position.z - robot_pt.z, 2.0));

  if (abs(yaw_error) > heading_tol && pos_error > position_tol) {
    // turn in place before moving
    NB_DEBUG("turning towards goal. pos_error = %.2f m, yaw_error = %.2f rad", pos_error, yaw_error);
    curr_wp = *robot_pose;
    curr_wp.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), yaw_d));
  } else if (pos_error > position_tol) {
    // maintain heading and move towards waypoint
    NB_DEBUG("moving towards goal. pos_error = %.2f m, yaw_error = %.2f rad", pos_error, yaw_error);
    curr_wp.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), yaw_d));
  } else if (waypoints.size() > 1) {
    // intermediate waypoint reached: advance to next
    NB_DEBUG("reached waypoint: {%.2f, %.2f, %.2f}", curr_wp.pose.position.x, curr_wp.pose.position.y, curr_wp.pose.position.z);
    waypoints.pop_front();
    NB_DEBUG("next waypoint: {%.2f, %.2f, %.2f}", waypoints.front().pose.position.x, waypoints.front().pose.position.y, waypoints.front().pose.position.z);
  } else if (nav_server.isActive()) {
    // last waypoint reached: set succeeded and execute end action
    NB_DEBUG("setting goal to succeeded");
    nav_server.setSucceeded(); // nav_server.isActive() == false now
    executeEndAction(end_action);
  }

  sendCommand(curr_wp); // to be implemented by inheriting class
}


double euclideanCost(int width, int start, int goal)
{
  int x1 = start % width;
  int y1 = start / width;
  int x2 = goal % width;
  int y2 = goal / width;

  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}


// assumes start and goal are inbounds of costmap
std::vector<int> NavBase::AStar(grid_mapping::Point start_pt,
                                grid_mapping::Point goal_pt) const
{
  int start = costmap_ptr->positionToIndex(start_pt);
  int goal = costmap_ptr->positionToIndex(goal_pt);

  typedef std::pair<double, int> q_el;
  std::priority_queue<q_el, std::vector<q_el>, std::greater<q_el>> frontier;
  std::unordered_map<int, int> came_from;
  std::unordered_map<int, double> cost_so_far;

  frontier.emplace(0.0, start);
  came_from.emplace(start, start);
  cost_so_far.emplace(start, 0.0);

  // perform search
  while (!frontier.empty()) {
    auto current = frontier.top().second;
    frontier.pop();

    if (current == goal)
      break;

    for (int next : costmap_ptr->neighborIndices(current)) {
      double movement_cost = euclideanCost(costmap_ptr->w, current, next);
      double obstacle_cost = costmap_ptr->data[next];
      double new_cost = cost_so_far[current] + movement_cost + obstacle_cost;

      if (cost_so_far.count(next) == 0 || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        came_from[next] = current;

        double weight = new_cost + euclideanCost(costmap_ptr->w, next, goal);
        frontier.emplace(weight, next);
      }
    }
  }

  // reconstruct path
  int current = goal;
  std::vector<int> path_indices;
  path_indices.push_back(current);
  while (current != start) {
    current = came_from[current];
    path_indices.push_back(current);
  }
  std::reverse(path_indices.begin(), path_indices.end());

  // prune path down to minimal set of points
  std::vector<int> minimal_indices(1, path_indices.front());
  auto curr_it = path_indices.begin();
  while (curr_it+1 != path_indices.end()) {

    // step along path until raycast between curr_it and next_it crosses
    // an obstacle or the end of the path is reached
    auto next_it = curr_it;
    for (; next_it+1 != path_indices.end(); ++next_it)
      if (!obstacleFree(costmap_ptr->rayCast(*curr_it, *(next_it+1))))
        break;

    // next_it is the furthest point along the path that can safely be reached
    minimal_indices.push_back(*next_it);
    curr_it = next_it;
  }

  return minimal_indices;
}

} // namespace intel_aero_navigation
