#include <intel_aero_navigation/waypoint_navigation.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <queue>
#include <unordered_map>
#include <iostream>


namespace intel_aero_navigation {


WaypointNavigation::WaypointNavigation(std::string name, ros::NodeHandle nh_, ros::NodeHandle pnh_):
  costmap(grid_mapping::Point(0.0, 0.0), 0.2, 1, 1),
  tf2_listener(tf2_buff),
  aero(nh_, pnh_),
  nav_server(nh_, name, false),
  nh(nh_),
  pnh(pnh_),
  takeoff_command_issued(false)
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
  if (!pnh.getParam("position_tol", waypoint_tol)) {
    waypoint_tol = 0.2;
    ROS_WARN("[WaypointNavigation] failed to fetch parameter \"position_tol\" "
             "using default value of %.2fm", waypoint_tol);
  }
  if (!pnh.getParam("yaw_tol", yaw_tol)) {
    yaw_tol = 0.2;
    ROS_WARN("[WaypointNavigation] failed to fetch parameter \"yaw_tol\" "
             "using default value of %.2fm", yaw_tol);
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

  std::vector<int> check_pts;
  check_pts.push_back(costmap.positionToIndex(robot_pose));
  for (auto path_pose : path) {
    check_pts.push_back(costmap.positionToIndex(path_pose));
  }

  bool obstacle_free = true;
  for (auto it = check_pts.begin()+1; it != check_pts.end(); ++it) {
    if (!obstacleFree(costmap.rayCast(*(it-1), *it))) {
      obstacle_free = false;
      break;
    }
  }

  if (obstacle_free)
    return;

  ROS_INFO("[WaypointNavigation] collision detected! replanning path");
  planPath(robot_pose);
}


void WaypointNavigation::goalCB()
{
  // abort any existing goals
  if (nav_server.isActive()) {
    ROS_INFO("[WaypointNavigation] goal aborted");
    nav_server.setAborted();
  }

  goal.header.frame_id = local_frame;
  goal.header.stamp = ros::Time::now();
  goal.pose = (nav_server.acceptNewGoal())->goal;
  tf2::doTransform(goal, goal, world_to_local);
  ROS_INFO("[WaypointNavigation] accepted new goal");

  if (nav_server.isPreemptRequested()) {
    ROS_INFO("[WaypointNavigation] goal preempted");
    nav_server.setPreempted();
    return;
  }

  // this is unlikely to happen
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

// must process costmap before calling this function and have costmap_mutex held
// in the calling thread
void WaypointNavigation::planPath(const geometry_msgs::PoseStamped& robot_pose)
{
  // robot pose (odom) not in costmap: e.g. this could happen if the robot moves
  // away from it's initial position but no obstacles have been encountered and
  // thus the costmap is still empty
  if (!costmap.inBounds(robot_pose)) {
    ROS_WARN("[WaypointNavigation] odom out of bounds of costmap: setting "
             "straight line path to goal");
    path.clear();
    path.push_back(goal);
    path_it = path.begin();
    return;
  }

  // goal pose not in bounds of costmap
  if (!costmap.inBounds(goal)) {
    ROS_INFO("[WaypointNavigation] goal out of bounds of costmap: expanding "
             "costmap");
    grid_mapping::Point new_origin = grid_mapping::min(goal, costmap.origin);
    grid_mapping::Point new_top_corner = grid_mapping::max(goal, costmap.topCorner());
    costmap.expandMap(new_origin, new_top_corner);
  }

  // plan path with A*
  // TODO what if goal is in occupied space?
  std::vector<int> path_indices = AStar(robot_pose, goal);

  // add current robot pose (with goal z) to path as first point
  geometry_msgs::PoseStamped first_waypoint = robot_pose;
  first_waypoint.pose.position.z = goal.pose.position.z;
  path.clear();
  path.push_back(first_waypoint);

  // prune A* path down to minimal set of points
  auto curr_index = path_indices.begin();
  while (curr_index != path_indices.end()) {
    auto next_index = curr_index;

    while (next_index+1 != path_indices.end()) {
      if (!obstacleFree(costmap.rayCast(*curr_index, *next_index))) {
        break;
      }
      ++next_index;
    }

    grid_mapping::Point curr_point = costmap.indexToPosition(*curr_index);
    grid_mapping::Point next_point = costmap.indexToPosition(*next_index);
    double yaw = atan2(next_point.y - curr_point.y, next_point.x - curr_point.x);

    // waypoint to align quad heading with trajectory (spin to face along traj)
    // TODO: only create this intermmediate waypoint if the current heading is way different
    geometry_msgs::PoseStamped waypoint = path.back();
    tf2::Quaternion yaw_quat;
    yaw_quat.setRPY(0.0, 0.0, yaw);
    waypoint.pose.orientation = tf2::toMsg(yaw_quat);
    path.push_back(waypoint);

    // position waypoint
    waypoint.pose.position.x = next_point.x;
    waypoint.pose.position.y = next_point.y;
    waypoint.pose.position.z = goal.pose.position.z;
    path.push_back(waypoint);

    curr_index = next_index+1;
  }
  path_it = path.begin();

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

  // TODO set goal to current state?
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


void WaypointNavigation::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  {
    std::lock_guard<std::mutex> lock(odom_mutex);
    odom = msg;
  }
  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.frame_id = odom->header.frame_id;
  robot_pose.header.stamp = odom->header.stamp;
  robot_pose.pose = odom->pose.pose;

  // no path or path complete; UAV will either do nothing (robot is unarmed)
  // or it will hover in place at the last completed goal
  if (!nav_server.isActive()) {
    aero.sendLocalPositionCommand(goal);
    return;
  }

  // if the UAV has not been started takeoff in place
  if (!takeoff_command_issued) {
    ROS_INFO("[WaypointNavigation] spawning takeoff thread");
    takeoff_thread = std::thread(&MavrosUAV::takeoff, aero, *path_it);
    takeoff_thread.detach();
    takeoff_command_issued = true;
  }

  geometry_msgs::PoseStamped current_waypoint;

  // check if the current waypoint has been reached
  if (distance(robot_pose, *path_it) < waypoint_tol &&
      headingError(robot_pose, *path_it) < yaw_tol) {
    ++path_it;
    if (path_it == path.end()) {
      nav_server.setSucceeded();
      current_waypoint = goal;
    }
  } else {
    current_waypoint = *path_it;
  }

  // does nothing if a takeoff command has not be successfully executed
  aero.sendLocalPositionCommand(current_waypoint);
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
