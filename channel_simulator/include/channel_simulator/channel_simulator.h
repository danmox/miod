#ifndef CHANNEL_SIMULATOR_CHANNEL_SIMULATOR_H
#define CHANNEL_SIMULATOR_CHANNEL_SIMULATOR_H

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/Point.h>

#include <vector>
#include <math.h>


namespace channel_simulator {


class ChannelModel {
  protected:
    double L0;       // transmit power (dBm)
    double PL0;      // transmit power (mW)
    double n;        // decay exponent
    double sigma_F2; // noise variance
    double N0;       // noise at receiver (dBm)
    double PN0;      // noise at receiver (mW)

  public:
    ChannelModel() :
      L0(-53.0), n(2.52), sigma_F2(40.0), N0(-70.0)
    {
      PL0 = dBm2mW(L0);
      PN0 = dBm2mW(N0);
    }

    double dBm2mW(const double dBm) { return pow(10.0, dBm/10.0); }
    void predict(const double d, double& mean, double& var)
    {
      mean = erf(sqrt(dBm2mW(L0 - 10*n*log10(d)) / PN0));
      var = pow((0.2 * d / (6.0 + d)), 2.0);
    }
};


class ChannelSimulator
{
  protected:
    octomap::OcTree* tree;
    ChannelModel model;

    std::vector<double> computeSegments(octomap::point3d p1,
                                        octomap::point3d p2);
    bool rayIntersection(const octomap::point3d& origin,
                         const octomap::point3d& direction,
                         octomap::point3d& intersection);

  public:
    ChannelSimulator();

    void mapCB(const octomap_msgs::Octomap::ConstPtr& msg);
    void predict(const geometry_msgs::Point& pose1,
                 const geometry_msgs::Point& pose2,
                 double& mean, double& var);
    bool receivedMap() { return tree == nullptr; }

};


} // namespace channel_simulator

#endif
