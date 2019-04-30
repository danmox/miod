#ifndef NETWORK_PLANNER_NETWORKPLANNER_H
#define NETWORK_PLANNER_NETWORKPLANNER_H


#include <ros/ros.h>
#include <CommunicationPredict.h>
#include <octomap/octomap.h>
#include <nav_msgs/Odometry.h>
#include <armadillo>
#include <vector>
#include <unordered_set>


namespace network_planner {


struct Flow
{
    std::unordered_set<int> sources;
    std::unordered_set<int> dests;
    double qos; // minimum bandwidth requirement
    double confidence; // probability with which requirements are satisfied
};


typedef std::vector<Flow> CommReqs;
typedef octomap::point3d_collection point_vec;


class NetworkPlanner
{
  protected:
    ros::NodeHandle nh, pnh;
    int num_task_agents, num_network_agents, num_agents;
    std::vector<ros::Subscriber> odom_subs;
    point_vec team_config;
    std::vector<bool> received_odom;
    CommunicationPredict channel_sim;

    void odomCB(const nav_msgs::Odometry::ConstPtr& msg, int idx);

  public:
    CommReqs comm_reqs;

    NetworkPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    bool Plan();
};


} // namespace network_planner


#endif
