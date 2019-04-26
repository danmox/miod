#ifndef CHANNEL_SIMULATOR_CHANNEL_SIMULATOR_H
#define CHANNEL_SIMULATOR_CHANNEL_SIMULATOR_H

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/Pose.h>

#include <vector>


namespace channel_simulator {


struct ChannelState {
  double mean;
  double variance;
};


class ChannelSimulator
{
  protected:
    octomap::OcTree* tree;

    std::vector<double> computeSegments(octomap::point3d p1,
                                        octomap::point3d p2);
    bool rayIntersection(const octomap::point3d& origin,
                         const octomap::point3d& direction,
                         octomap::point3d& intersection);

  public:
    ChannelSimulator();

    void mapCB(const octomap_msgs::Octomap::ConstPtr& msg);
    ChannelState predict(const geometry_msgs::Pose& pose1,
                         const geometry_msgs::Pose& pose2);

};


} // namespace channel_simulator

#endif
