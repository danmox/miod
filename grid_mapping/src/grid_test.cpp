#include "grid_mapping/grid.hpp"
#include "grid_mapping/common.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <opencv2/opencv.hpp>


template <class T>
cv::Mat createGridImage(const grid_mapping::Grid<T>& grid)
{
  std::vector<int> map_data;
  std::transform(grid.data.begin(), grid.data.end(),
                 std::back_inserter(map_data),
                 [](double a){ return static_cast<int>(a*255.0/100.0); });
  cv::Mat img = cv::Mat(map_data).reshape(0, grid.h);
  img.convertTo(img, CV_8UC1);
  cv::flip(img, img, 0);
  return img;
}


int main(int argc, char** argv)
{
  if (argc != 2) {
    ROS_FATAL("Please provide a bagfile");
    exit(EXIT_FAILURE);
  }

  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);

  nav_msgs::OccupancyGrid::ConstPtr costmap;
  for (rosbag::MessageInstance m : rosbag::View(bag)) {
    nav_msgs::OccupancyGrid::ConstPtr msg = m.instantiate<nav_msgs::OccupancyGrid>();
    if (msg) {
      costmap = msg;
      break;
    }
  }
  if (!costmap) {
    ROS_ERROR("failed to load nav_msgs::OccupancyGrid from bagfile");
    return -1;
  }

  grid_mapping::Grid<int8_t> grid(costmap);
  grid_mapping::Point new_origin = grid.origin - grid.resolution;
  grid_mapping::Point new_ne_corner = grid.topCorner() + grid.resolution;
  grid.expandMap(new_origin, new_ne_corner);

  cv::Mat img = createGridImage(grid);
  displayImageComplement(img, "costmap");

  bag.close();

  return 0;
}
