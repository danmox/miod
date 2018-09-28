#include "grid_mapping/common.h"

#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <iterator>
#include <algorithm>

cv::Mat createGridImage(const nav_msgs::OccupancyGrid& grid)
{
  std::vector<int> map_data;
  std::transform(grid.data.begin(), grid.data.end(),
      std::back_inserter(map_data),
      [](unsigned char a){ return static_cast<int>(a*255.0/100.0); });
  cv::Mat img = cv::Mat(map_data).reshape(0, grid.info.height);
  img.convertTo(img, CV_8UC1);
  cv::flip(img, img, 0);
  return img;
}

void displayImage(const cv::Mat& img, const std::string name)
{
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::imshow(name, img);
  cv::waitKey(0);
}

void displayImageComplement(const cv::Mat& img, const std::string name)
{
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::imshow(name, 255-img);
  cv::waitKey(0);
}
