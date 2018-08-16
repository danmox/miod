#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <vector>
#include <unordered_map>


int costFunction(const NavigationOccupancyGrid &grid, int start, int goal)
{
  int w = grid.info.width;

  int x1 = start % w;
  int y1 = start / w;
  int x2 = goal % w;
  int y2 = goal / w;

  return abs(x2 - x1) + abs(y2 - y1);
}


std::vector<int> AStar(const NavigationOccupancyGrid &grid, int start, int goal)
{
  // check inputs
  std::vector<int> path;
  if (!grid.data[start] > FREE_THRESHOLD) { //TODO
    ROS_ERROR("AStar: start cell invalid!");
    return path;
  } else if (!grid.data[goal] > FREE_THRESHOLD) { //TODO
    ROS_ERROR("AStar: goal cell invalid!");
    return path;
  }

  // set up priority queue
  typedef std::pair<int, int> q_el;
  std::priority_queue<q_el, std::vector<q_el>, std::greater<q_el>> frontier;

  // keep track of cost and previous cell
  std::unordered_map<int, int> came_from, cost_so_far;

  frontier.emplace(0, start);
  came_from.emplace(start, start);
  cost_so_far.emplace(start, 0);

  // perform search
  while (!frontier.empty()) {
    auto current = frontier.top().second;
    frontier.pop();

    if (current == goal)
      break;

    for (int next_cell : grid.freeNeighbors(current)) {
      int new_cost = cost_so_far[current] + 1; // graph cost = 1

      if (!cost_so_far.count(next_cell) || new_cost < cost_so_far[next_cell]) {
        cost_so_far[next_cell] = new_cost;
        int weight = new_cost + costFunction(grid, next_cell, goal);
        frontier.emplace(weight, next_cell);
        came_from[next_cell] = current;
      }
    }
  }

  // reconstruct path
  int current = goal;
  path.push_back(current);
  while (current != start) {
    current = came_from[current];
    path.push_back(current);
  }
  reverse(path.begin(), path.end());

  return path;
}
