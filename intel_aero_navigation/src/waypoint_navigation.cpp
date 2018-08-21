#include <intel_aero_navigation/waypoint_navigation.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <queue>
#include <unordered_map>
#include <iostream>


namespace intel_aero_navigation {


WaypointNavigation::WaypointNavigation(std::string name, ros::NodeHandle nh_, ros::NodeHandle pnh_):
  costmap(grid_mapping::Point(0.0, 0.0), 0.1, 1, 1),
  nav_server(nh_, name, false),
  nh(nh_),
  pnh(pnh_)
{
  nav_server.registerGoalCallback(std::bind(&WaypointNavigation::goalCB, this));
  nav_server.registerPreemptCallback(std::bind(&WaypointNavigation::preemptCB, this));
  nav_server.start();

  if (!pnh.getParam("path_frame_id", path_frame_id)) {
    ROS_WARN("[WaypointNavigation] failed to fetch parameter \"frame_id\" using"
             "default value of \"map\"");
    path_frame_id = "map";
  }

  costmap_sub = nh.subscribe("costmap", 10, &WaypointNavigation::costmapCB, this);
  odom_sub = nh.subscribe("odom", 10, &WaypointNavigation::odomCB, this);
  path_pub = nh.advertise<nav_msgs::Path>("path", 10);
}

void WaypointNavigation::costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex);
  ros_costmap_ptr = msg;
}


void WaypointNavigation::goalCB()
{
  // abort any existing goals
  if (nav_server.isActive()) {
    ROS_INFO("[WaypointNavigation] goal aborted");
    nav_server.setAborted();
  }

  goal = (nav_server.acceptNewGoal())->goal;
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

  // convert stored costmap to grid_mapping::Grid
  {
    std::lock_guard<std::mutex> lock(costmap_mutex);
    costmap = Costmap(ros_costmap_ptr);
  }

  // fetch current pose of robot
  geometry_msgs::Pose robot_pose;
  {
    std::lock_guard<std::mutex> lock(odom_mutex);
    robot_pose = odom->pose.pose;
  }

  // robot pose (odom) not in costmap: e.g. this could happen if the robot moves
  // away from it's initial position but no obstacles have been encountered and
  // thus the costmap is still empty
  if (!costmap.inBounds(robot_pose.position.x, robot_pose.position.y)) {
    ROS_INFO("[WaypointNavigation] odom out of bounds of costmap: setting "
             "straight line path to goal");
    path.clear();
    path.push_back(goal);
    current_waypoint = path.begin();
    return;
  }

  grid_mapping::Point goal_pt(goal.position.x, goal.position.y);

  // goal pose not in bounds of costmap
  if (!costmap.inBounds(goal.position.x, goal.position.y)) {
    ROS_INFO("[WaypointNavigation] goal out of bounds of costmap: expanding "
             "costmap");
    grid_mapping::Point new_origin = grid_mapping::min(goal_pt, costmap.origin);
    grid_mapping::Point new_top_corner = grid_mapping::max(goal_pt, costmap.topCorner());
    costmap.expandMap(new_origin, new_top_corner);
  }

  // plan path with A*
  ROS_INFO("[WaypointNavigation] planning path through costmap with A*");
  std::vector<int> path_indices = AStar(robot_pose, goal);

  // prune A* path down to minimal set of points
  ROS_INFO("[WaypointNavigation] pruning path down to minimal set of points");
  path.clear();
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

    geometry_msgs::Pose waypoint;
    waypoint.position.x = next_point.x;
    waypoint.position.y = next_point.y;
    waypoint.position.z = goal.position.z;
    waypoint.orientation = tf::createQuaternionMsgFromYaw(yaw);
    path.push_back(waypoint);

    curr_index = next_index+1;
  }
  current_waypoint = path.begin();

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


void WaypointNavigation::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex);
  odom = msg;

  // TODO trajectory tracking
  // TODO obstacle checking
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
std::vector<int> WaypointNavigation::AStar(geometry_msgs::Pose start_pose,
                                           geometry_msgs::Pose goal_pose) const
{
  int start = costmap.positionToIndex(start_pose.position.x,
                                      start_pose.position.y);
  int goal = costmap.positionToIndex(goal_pose.position.x,
                                     goal_pose.position.y);

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

void WaypointNavigation::publishPath(const geometry_msgs::Pose& robot_pose) const
{
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = path_frame_id;

  geometry_msgs::PoseStamped start_pose;
  start_pose.pose = robot_pose;
  path_msg.poses.push_back(start_pose);
  for (auto pose : path) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    path_msg.poses.push_back(pose_stamped);
  }
  path_pub.publish(path_msg);
}


} // namespace intel_aero_navigation
