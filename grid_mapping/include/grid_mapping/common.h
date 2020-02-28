#ifndef GRID_MAPPING_COMMON_H_
#define GRID_MAPPING_COMMON_H_

#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <string>

cv::Mat createGridImage(const nav_msgs::OccupancyGrid& grid);
void displayImage(const cv::Mat& img, const std::string name);
void displayImageComplement(const cv::Mat& img, const std::string name);

#endif
