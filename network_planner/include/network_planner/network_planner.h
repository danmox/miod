#ifndef NETWORK_PLANNER_NETWORKPLANNER_H
#define NETWORK_PLANNER_NETWORKPLANNER_H


#include <ros/ros.h>
#include <channel_simulator/channel_simulator.h>
#include <grid_mapping/grid.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <routing_msgs/NetworkUpdate.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>
#include <actionlib/client/simple_action_client.h>
#include <armadillo>
#include <vector>
#include <unordered_set>


namespace network_planner {


struct Flow
{
  public:
    std::unordered_set<int> srcs; // source nodes
    std::unordered_set<int> dests; // destinatio node
    double min_margin; // minimum bandwidth requirement
    double confidence; // probability with which requirements are satisfied
};


typedef std::vector<Flow> CommReqs;
typedef std::vector<arma::vec3> point_vec;
typedef grid_mapping::Grid<int8_t> Costmap;


class NetworkPlanner
{
  protected:
    // ROS parameters
    int task_count;          // number of task agents serviced
    int comm_count;          // number of supporting network agents
    int sample_count;        // number of random samples used in gradient step
    double sample_var;       // variance of samples drawn from a normal
                             // distribution centered at the agents' positions
    double desired_altitude; // fixed desired z of quadrotors
    double max_velocity;     // limit for largest step from gradient

    ros::NodeHandle nh, pnh;
    std::vector<ros::Subscriber> odom_subs;
    std::vector<ros::Publisher> vel_pubs;
    ros::Subscriber map_sub;
    ros::Publisher viz_pub, net_pub;

    int total_agents, num_dests;
    std::vector<int> comm_idcs;

    point_vec team_config;
    std::vector<bool> received_odom;

    channel_simulator::ChannelSimulator channel_sim;
    Costmap costmap;
    bool received_costmap;
    CommReqs comm_reqs;

    double update_duration; // estimated rate that updateNetworkConfig() runs at

    // navigation vars
    typedef intel_aero_navigation::WaypointNavigationAction NavAction;
    typedef actionlib::SimpleActionClient<NavAction> NavClient;
    std::vector<std::shared_ptr<NavClient>> nav_clients;

    // SOCP vars
    std::map<std::tuple<int,int,int>, int> ijk_to_idx;
    std::map<int, std::tuple<int,int,int>> idx_to_ijk;
    int num_flows, alpha_dim, y_dim;
    arma::vec y_col; // solution of robust routing problem [alpha_ij_k; slack]

    double computeV(const point_vec& config, bool debug);
    double computebik(const point_vec& config, bool debug);
    void probConstraints(const arma::mat& R_mean,
                         const arma::mat& R_var,
                         std::vector<arma::mat>& A_mats,
                         std::vector<arma::vec>& b_vecs,
                         std::vector<arma::vec>& c_vecs,
                         std::vector<arma::vec>& d_vecs,
                         int& idx,
                         bool divide_psi_inv_eps,
                         bool debug);
    void networkState(const point_vec& state, arma::mat& R_mean,
                      arma::mat& R_var);
    bool SOCP(const arma::mat& R_mean, const arma::mat& R_var,
              std::vector<arma::mat>& alpha_ij_k,
              double& slack, bool debug);

  public:

    NetworkPlanner() : costmap(grid_mapping::Point(0.0, 0.0), 0.2, 1, 1) {}
    NetworkPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    bool updateNetworkConfig();
    void odomCB(const nav_msgs::Odometry::ConstPtr& msg, int idx);
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void setCommReqs(const CommReqs& reqs);
    void runPlanningLoop();
    void initSystem();
};


// combinatorial functions
std::vector<int> extract_inds(int num, const int agents);
std::vector<int> compute_combinations(const int len, const int num_digits);


} // namespace network_planner


#endif
