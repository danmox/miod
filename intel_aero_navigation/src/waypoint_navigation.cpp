#include <intel_aero_navigation/waypoint_navigation.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <queue>
#include <unordered_map>
#include <iostream>
#include <cmath>


namespace intel_aero_navigation {


WaypointNavigation::WaypointNavigation(std::string name, ros::NodeHandle nh_, ros::NodeHandle pnh_):
  costmap(grid_mapping::Point(0.0, 0.0), 0.2, 1, 1),
  tf2_listener(tf2_buff),
  aero(nh_, pnh_),
  nav_server(nh_, name, false),
  nh(nh_),
  pnh(pnh_),
  takeoff_command_issued(false),
  end_behavior(WaypointNavigationGoal::HOVER)
{
  nav_server.registerGoalCallback(std::bind(&WaypointNavigation::goalCB, this));
  nav_server.registerPreemptCallback(std::bind(&WaypointNavigation::preemptCB, this));
  nav_server.start();

  if (!pnh.getParam("costmap_frame", costmap_frame)) {
    ROS_WARN("[WaypointNavigation] failed to fetch parameter \"costmap_frame\" "
             "using default value of \"map\"");
    world_frame = "map";
  }
  if (!pnh.getParam("world_frame", world_frame)) {
    ROS_WARN("[WaypointNavigation] failed to fetch parameter \"world_frame\" "
             "using default value of \"map\"");
    world_frame = "world";
  }
  if (!pnh.getParam("local_frame", local_frame)) {
    ROS_WARN("[WaypointNavigation] failed to fetch parameter \"local_frame\" "
             "using default value of \"map\"");
    local_frame = "map";
  }
  costmap.frame_id = local_frame;
  if (!pnh.getParam("position_tol", position_tol)) {
    position_tol = 0.2;
    ROS_WARN("[WaypointNavigation] failed to fetch parameter \"position_tol\" "
             "using default value of %.2fm", position_tol);
  }
  if (!pnh.getParam("heading_tol", heading_tol)) {
    heading_tol = 0.2;
    ROS_WARN("[WaypointNavigation] failed to fetch parameter \"heading_tol\" "
             "using default value of %.2fm", heading_tol);
  }

  costmap_sub = nh.subscribe("costmap", 10, &WaypointNavigation::costmapCB, this);
  odom_sub = nh.subscribe("odom", 10, &WaypointNavigation::odomCB, this);
  path_pub = nh.advertise<nav_msgs::Path>("path", 10);

  // cache transforms needed to convert between local, world and costmap frames
  ROS_INFO("[WaypointNavigation] waiting for transforms to be available");
  ros::Rate rate(1);
  bool transforms_cached = false;
  while (!transforms_cached) {
    try {
      ros::Time t0(0);
      world_to_local = tf2_buff.lookupTransform(local_frame, world_frame, t0);
      costmap_to_local = tf2_buff.lookupTransform(local_frame, costmap_frame, t0);
      transforms_cached = true;
      ROS_INFO("[WaypointNavigation] cached transforms");
    } catch (tf2::TransformException &ex) {
      ROS_WARN("[WaypointNavigation] %s", ex.what());
    }
    ros::spinOnce();
    rate.sleep();
  }
}

void WaypointNavigation::costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  static bool processed_first_map = false; // set resolution from first map

  {
    std::lock_guard<std::mutex> lock(costmap_mutex);
    ros_costmap_ptr = msg;
    processed_costmap = false;
  }

  // set resolution from received map
  if (!processed_first_map) {
    costmap.resolution = ros_costmap_ptr->info.resolution;
    processed_first_map = true;
  }

  if (!nav_server.isActive())
    return;

  // fetch current pose of robot
  geometry_msgs::PoseStamped robot_pose;
  {
    std::lock_guard<std::mutex> lock(odom_mutex);
    robot_pose.header.frame_id = odom->header.frame_id;
    robot_pose.header.stamp = odom->header.stamp;
    robot_pose.pose = odom->pose.pose;
  }

  // lock remainder of thread for costmap use
  std::lock_guard<std::mutex> lock(costmap_mutex);

  // process costmap, shifting it into the local frame
  geometry_msgs::Pose origin;
  tf2::doTransform(ros_costmap_ptr->info.origin, origin, costmap_to_local);
  Costmap in_map(ros_costmap_ptr);
  in_map.origin = grid_mapping::Point(origin.position.x, origin.position.y);
  costmap.insertMap(in_map);
  processed_costmap = true;

  // check current path for collisions (locked due to costmap access)

  grid_mapping::Point first_point(robot_pose);
  if (!costmap.inBounds(first_point)) { // ensure 1st checkpoint is inbounds
    if (!path.empty()) {
      first_point = costmap.bbxIntersection(path[0], first_point);
    } else {
      return;
    }
  }

  // build list of checkpoints
  std::vector<int> check_pts;
  check_pts.push_back(costmap.positionToIndex(first_point));
  for (auto it = path_it; it != path.end(); ++it)
    check_pts.push_back(costmap.positionToIndex(*it));

  // check cells along path for obstacles
  bool obstacle_free = true;
  grid_mapping::Point pt1, pt2;
  for (auto it = check_pts.begin()+1; it != check_pts.end(); ++it) {
    if (!obstacleFree(costmap.rayCast(*(it-1), *it))) {
      pt1 = costmap.indexToPosition(*(it-1));
      pt2 = costmap.indexToPosition(*it);
      obstacle_free = false;
      break;
    }
  }

  if (!obstacle_free) {
    //ROS_INFO("[WaypointNavigation] collision detected! replanning path");
    ROS_INFO_STREAM("[WaypointNavigation] collision detected between " << pt1 << " and " << pt2);
    //planPath(robot_pose);
  }

  return;
}


void WaypointNavigation::goalCB()
{
  // abort any existing goals
  if (nav_server.isActive()) {
    ROS_INFO("[WaypointNavigation] goal aborted");
    nav_server.setAborted();
  }

  // accept new goal
  auto action_goal = nav_server.acceptNewGoal();
  end_behavior = action_goal->end_behavior;
  ROS_INFO("[WaypointNavigation] accepted new goal");

  // without knowing the quad's current position it is impossible to proceed
  // with planning
  if (!odom) {
    ROS_ERROR("[WaypointNavigation] no odom! aborting goal");
    nav_server.setAborted();
    return;
  }

  // fetch current pose of robot
  geometry_msgs::PoseStamped robot_pose;
  {
    std::lock_guard<std::mutex> lock(odom_mutex);
    robot_pose.header.frame_id = odom->header.frame_id;
    robot_pose.header.stamp = odom->header.stamp;
    robot_pose.pose = odom->pose.pose;
  }

  // if there are no waypoints execute the desired end behavior
  if (action_goal->waypoints.empty()) {
    if (end_behavior == WaypointNavigationGoal::LAND) {
      path.clear();
      nav_server.setSucceeded();
      aero.land();
    } else if (end_behavior == WaypointNavigationGoal::HOVER) {
      path.clear();
      robot_pose.pose.position.z = 2; // default hover height
      path.push_back(robot_pose);
      path_it = path.begin();
    }
    return;
  }

  // convert vector of pose msgs in the global frame to vector of timestamped
  // poses in the local frame
  geometry_msgs::PoseStamped waypoint;
  waypoint.header.frame_id = local_frame;
  waypoint.header.stamp = ros::Time::now();
  waypoints.clear();
  for (auto& pose : action_goal->waypoints) {
    waypoint.pose = pose;
    tf2::doTransform(waypoint, waypoint, world_to_local);
    waypoints.push_back(waypoint);
  }

  if (nav_server.isPreemptRequested()) {
    ROS_INFO("[WaypointNavigation] goal preempted");
    nav_server.setPreempted();
    return;
  }

  // lock remainder of thread for costmap use
  std::lock_guard<std::mutex> lock(costmap_mutex);

  if (!processed_costmap && ros_costmap_ptr) {
    geometry_msgs::Pose origin;
    tf2::doTransform(ros_costmap_ptr->info.origin, origin, costmap_to_local);
    Costmap in_map(ros_costmap_ptr);
    in_map.origin = grid_mapping::Point(origin.position.x, origin.position.y);
    costmap.insertMap(in_map);
    processed_costmap = true;
  }

  planPath(robot_pose);
}

// return the quaternion corresponding to a yaw aligned with the line segment
// connecting points p1 and p2
geometry_msgs::Quaternion pathSegmentQuaternion(const geometry_msgs::Point& p1,
                                                const geometry_msgs::Point& p2)
{
  double yaw = atan2(p2.y - p1.y, p2.x - p1.x);
  tf2::Quaternion yaw_quat;
  yaw_quat.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(yaw_quat);
}


// must process costmap before calling this function and have costmap_mutex held
// in the calling thread
void WaypointNavigation::planPath(const geometry_msgs::PoseStamped& robot_pose)
{
  // robot pose (odom) not in costmap: e.g. this could happen if the robot moves
  // away from it's initial position but no obstacles have been encountered and
  // thus the costmap is still empty
  if (!costmap.inBounds(robot_pose)) {
    ROS_WARN("[WaypointNavigation] odom out of bounds of costmap: setting "
             "straight line path to all waypoints");
    path = waypoints;
    path_it = path.begin();
    return;
  }

  // ensure waypoints are in bounds of costmap
  for (auto& pt : waypoints) {
    if (!costmap.inBounds(pt)) {
      ROS_INFO("[WaypointNavigation] waypoint out of bounds of costmap: "
               "expanding costmap");
      grid_mapping::Point new_origin = grid_mapping::min(pt, costmap.origin);
      grid_mapping::Point new_top_corner = grid_mapping::max(pt, costmap.topCorner());
      costmap.expandMap(new_origin, new_top_corner);
    }
  }

  // the path the quad follows consists of two kinds of points: position
  // waypoints, consisting of the set of goal points received in the action
  // message and the points computed by A* to avoid obstacles while traveling
  // between them, and orientation waypoints, additional waypoints added to
  // sharp corners in the path to allow the robot to turn and face along the
  // path segment before traversing it
  std::vector<geometry_msgs::Point> position_waypoints;
  geometry_msgs::Point starting_point;
  starting_point.x = robot_pose.pose.position.x;
  starting_point.y = robot_pose.pose.position.y;
  starting_point.z = waypoints.front().pose.position.z;
  position_waypoints.push_back(starting_point);

  // use A* to plan obstacle free paths between each waypoint
  for (geometry_msgs::PoseStamped& next_waypoint : waypoints) {

    std::vector<int> path_inds;
    path_inds = AStar(position_waypoints.back(), next_waypoint);

    // prune A* path down to minimal set of points
    auto curr_ind = path_inds.begin();
    while (curr_ind+1 != path_inds.end()) {

      // step along path until raycast between curr_ind and next_ind crosses
      // an obstacle or the end of the path is reached
      auto next_ind = curr_ind;
      for (; next_ind+1 != path_inds.end(); ++next_ind)
        if (!obstacleFree(costmap.rayCast(*curr_ind, *(next_ind+1))))
          break;

      // next_ind is the furthest point along the path that can safely be reached
      grid_mapping::Point pt = costmap.indexToPosition(*next_ind);
      geometry_msgs::Point waypoint;
      waypoint.x = pt.x;
      waypoint.y = pt.y;
      waypoint.z = next_waypoint.pose.position.z;
      position_waypoints.push_back(waypoint);
      curr_ind = next_ind;
    }
  }

  auto wp_it = position_waypoints.begin();

  // first waypoint
  geometry_msgs::PoseStamped waypoint;
  waypoint.header.frame_id = local_frame;
  waypoint.header.stamp = ros::Time::now();
  waypoint.pose.position = *wp_it;
  waypoint.pose.orientation = pathSegmentQuaternion(*wp_it, *(wp_it+1));
  path.clear();
  path.push_back(waypoint);

  // intermediate waypoints
  for (++wp_it; wp_it+1 != position_waypoints.end(); ++wp_it) {
    // position waypoint
    waypoint.pose.position = *wp_it;
    path.push_back(waypoint);
    // yaw waypoint
    waypoint.pose.orientation = pathSegmentQuaternion(*wp_it, *(wp_it+1));
    path.push_back(waypoint);
  }

  // end waypoint
  waypoint.pose.position = *wp_it;
  path.push_back(waypoint);

  // initialize tracker to first point
  path_it = path.begin();

  // send the path to rviz
  publishPath(robot_pose);
}

bool WaypointNavigation::obstacleFree(const std::vector<int>& cells) const
{
  for (int cell : cells) {
    if (costmap.data[cell] > 0)
      return false;
  }
  return true;
}


void WaypointNavigation::preemptCB()
{
  if (nav_server.isActive()) {
    ROS_INFO("[WaypointNavigation] goal aborted");
    nav_server.setAborted();
  } else {
    ROS_INFO("[WaypointNavigation] goal preempted");
    nav_server.setPreempted();
  }

  // TODO set waypoint to current state?
}


double distance(const geometry_msgs::PoseStamped& p1,
                const geometry_msgs::PoseStamped& p2)
{
  double dx = p1.pose.position.x - p2.pose.position.x;
  double dy = p1.pose.position.y - p2.pose.position.y;
  double dz = p1.pose.position.z - p2.pose.position.z;
  return sqrt(pow(dx, 2.0) + pow(dy, 2.0) + pow(dz, 2.0));
}


double headingError(const geometry_msgs::PoseStamped& p1,
                    const geometry_msgs::PoseStamped& p2)
{
  double yaw1 = tf2::getYaw(p1.pose.orientation);
  double yaw2 = tf2::getYaw(p2.pose.orientation);
  double error = yaw2 - yaw1;
  if (error > 2.0*M_PI)
    return error - 2.0*M_PI;
  if (error < -2.0*M_PI)
    return error + 2.0*M_PI;
  return error;
}


bool waypointReached(const geometry_msgs::PoseStamped& robot,
                     const geometry_msgs::PoseStamped& goal,
                     const double position_tol,
                     const double heading_tol)
{
  bool distance_achieved = distance(robot, goal) < position_tol;
  bool heading_achieved = std::abs(headingError(robot, goal)) < heading_tol;
  return distance_achieved && heading_achieved;
}


void WaypointNavigation::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  {
    std::lock_guard<std::mutex> lock(odom_mutex);
    odom = msg;
  }

  // nothing to do yet
  if (path.empty())
    return;

  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.frame_id = msg->header.frame_id;
  robot_pose.header.stamp = msg->header.stamp;
  robot_pose.pose = msg->pose.pose;

  // if the quad has visited all waypoints already and was requested to hover at
  // the last one
  if (!nav_server.isActive()) {
    if (end_behavior == WaypointNavigationGoal::HOVER) {
      aero.sendLocalPositionCommand(path.back());
      ROS_DEBUG("[WaypointNavigation] sending local position command");
    }
    return;
  }

  // send takeoff command if UAV is not armed
  if (!aero.takeoffCommandIssued())
    aero.takeoff(*path_it);

  geometry_msgs::PoseStamped current_waypoint;

  // check if the current waypoint has been reached
  if (waypointReached(robot_pose, *path_it, position_tol, heading_tol)) {
    ++path_it;
    if (path_it == path.end()) { // path completed, execute end behavior
      nav_server.setSucceeded(); // nav_server.isActive now returns false until new goal is received
      if (end_behavior == WaypointNavigationGoal::LAND) {
        aero.land();
        return;
      }
      current_waypoint = path.back();
    }
  } else {
    current_waypoint = *path_it;
  }

  aero.sendLocalPositionCommand(current_waypoint);
  ROS_DEBUG("[WaypointNavigation] sending local position command");
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
std::vector<int> WaypointNavigation::AStar(grid_mapping::Point start_pt,
                                           grid_mapping::Point goal_pt) const
{
  int start = costmap.positionToIndex(start_pt);
  int goal = costmap.positionToIndex(goal_pt);

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

    for (int next : costmap.neighborIndices(current)) {
      double movement_cost = euclideanCost(costmap.w, current, next);
      double obstacle_cost = costmap.data[next];
      double new_cost = cost_so_far[current] + movement_cost + obstacle_cost;

      if (cost_so_far.count(next) == 0 || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        came_from[next] = current;

        double weight = new_cost + euclideanCost(costmap.w, next, goal);
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

  return path_indices;
}

void WaypointNavigation::publishPath(const geometry_msgs::PoseStamped& pose) const
{
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = local_frame;

  path_msg.poses.push_back(pose);
  for (auto path_pose : path) {
    path_msg.poses.push_back(path_pose);
  }
  path_pub.publish(path_msg);
}


} // namespace intel_aero_navigation
