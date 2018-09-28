#ifndef GRID_MAPPING_GRID_HPP_
#define GRID_MAPPING_GRID_HPP_


#include "grid_mapping/grid.h"
#include <ros/ros.h>


namespace grid_mapping {


template <class T>
Grid<T>::Grid(Point origin_, double res, int w_, int h_, bool alloc_data) :
  GridBase(origin_, res, w_, h_)
{
  if (alloc_data)
    data = std::vector<T>(w*h, 0.0);
}


template <class T>
Grid<T>::Grid(const nav_msgs::OccupancyGrid::ConstPtr& msg) :
  GridBase(Point(msg->info.origin.position.x, msg->info.origin.position.y),
      msg->info.resolution, msg->info.width, msg->info.height)
{
  data.reserve(msg->data.size());
  for (auto cell : msg->data)
    data.push_back(static_cast<T>(cell));
}


// update existing map with other map info
template <class T>
void Grid<T>::update(const Grid* grid)
{
  int w_in = grid->w;
  int origin_offset = positionToIndex(grid->origin);
  for (int i = 0; i < grid->h; ++i) {
    int c = origin_offset + i*w;
    int c_in = i*w_in;
    for (int j = 0; j < w_in; ++j) {
      data[c+j] = updateCell(data[c+j], grid->data[c_in + j]);
    }
  }
}


// update a cell of the internal map during map insertion
// NOTE: default behavior as defined here is to overwrite the internal data!
// TODO: let user define update behavior via a passed function
template <class T>
T Grid<T>::updateCell(T current_value, T new_value)
{
  return new_value;
}


// Expand map to include p_min, p_max
template <class T>
void Grid<T>::expandMap(const Point p_min, const Point p_max)
{
  // determine new extents of map
  Point new_origin(origin);
  Point new_top_corner = topCorner();
  if (p_min.x < new_origin.x)
    new_origin.x = roundToMapRes(p_min.x);
  if (p_min.y < new_origin.y)
    new_origin.y = roundToMapRes(p_min.y);
  if (p_max.x >= new_top_corner.x)
    new_top_corner.x = roundToMapRes(p_max.x);
  if (p_max.y >= new_top_corner.y)
    new_top_corner.y = roundToMapRes(p_max.y);
  int w_new = round((new_top_corner.x - new_origin.x) / resolution) + 1;
  int h_new = round((new_top_corner.y - new_origin.y) / resolution) + 1;

  // overwrite old map with new map
  Grid<T> new_grid(new_origin, resolution, w_new, h_new);
  new_grid.update(this);
  origin = new_origin;
  w = w_new;
  h = h_new;
  data = new_grid.data;
}


// TODO this isn't very efficient
// TODO WHAT IF THE RESOLUTION ISN'T THE SAME!!!!
template <class T>
void Grid<T>::insertMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  Grid<T> in(msg);

  // ensure current map spans input map
  if (!inBounds(in.origin) || !inBounds(in.topCorner()))
    expandMap(in.origin, in.topCorner());

  // update local grid with in_grid data
  update(&in);
}


template <class T>
nav_msgs::OccupancyGrid Grid<T>::createROSMsg() const
{
  nav_msgs::OccupancyGrid msg;
  msg.header.stamp = ros::Time::now();
  msg.info.resolution = resolution;
  msg.info.width = w;
  msg.info.height = h;
  msg.info.origin.position.x = origin.x;
  msg.info.origin.position.y = origin.y;
  msg.data.resize(data.size());
  msg.data.insert(msg.data.begin(), data.begin(), data.end());

  return msg;
}


template <class H>
std::ostream& operator<<(std::ostream& out, const Grid<H>& grid)
{
  std::cout << std::endl;
  std::cout << "info:" << std::endl;
  std::cout << "  origin: " << grid.origin << std::endl;
  std::cout << "  w: " << grid.w << std::endl;
  std::cout << "  h: " << grid.h << std::endl;
  std::cout << "  resolution: " << grid.resolution << std::endl;
  std::cout << "data:" << std::endl;
  for (int i = 0; i < grid.data.size(); ++i) {
    if (i % grid.w != 0)
      std::cout << grid.data[i] << ", ";
    else
      std::cout << std::endl << "  " << grid.data[i] << ", ";
  }
  std::cout << std::endl;
}


} // namespace grid_mapping


#endif
