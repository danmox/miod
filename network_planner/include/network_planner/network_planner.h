#ifndef NETWORK_PLANNER_NETWORKPLANNER_H
#define NETWORK_PLANNER_NETWORKPLANNER_H


#include <ros/ros.h>
#include <channel_simulator/channel_simulator.h>
#include <grid_mapping/grid.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <routing_msgs/NetworkUpdate.h>
#include <socp/RobustRoutingSOCP.h>
#include <intel_aero_navigation/WaypointNavigationAction.h>
#include <actionlib/client/simple_action_client.h>
#include <armadillo>
#include <vector>
#include <unordered_set>
#include <map>


namespace network_planner {


struct Flow
{
  public:
    std::unordered_set<int> srcs;  // source node
    std::unordered_set<int> dests; // destination node
    double min_margin; // minimum bandwidth requirement
    double confidence; // probability with which requirements are satisfied
};


typedef std::vector<arma::vec3> point_vec;
typedef grid_mapping::Grid<int8_t> Costmap;
typedef std::vector<Flow> CommReqs;
typedef std::vector<std::string> string_vec;


class NetworkPlanner
{
  protected:
    // ROS parameters
    int task_count;             // number of task agents serviced
    int comm_count;             // number of supporting network agents
    int sample_count;           // number of random samples used in gradient step
    double sample_var;          // variance of planning samples
    double desired_altitude;    // fixed desired z of quadrotors
    double max_velocity;        // limit for largest step from gradient
    double collision_distance;  // minimum distance between agents
    double minimum_update_rate; // minimum rate at which updateNetworkConfig can run
    std::string world_frame;

    ros::NodeHandle nh, pnh;
    std::vector<ros::Subscriber> pose_subs;
    ros::Subscriber map_sub;
    ros::Publisher viz_pub, net_pub, qos_pub;
    ros::ServiceClient socp_srv;

    int agent_count, num_dests, num_flows;
    std::vector<int> comm_idcs;
    std::vector<int> task_idcs;
    std::map<int,std::string> idx_to_ip;
    std::map<int,std::string> id_to_ip;
    std::map<int,int> id_to_idx;
    std::map<int,int> idx_to_id;
    string_vec agent_ips;       // list of the ip address of each agent

    point_vec team_config;
    std::vector<bool> received_odom;

    channel_simulator::ChannelSimulator channel_sim;
    Costmap costmap;
    bool received_costmap;
    CommReqs comm_reqs_id;

    // navigation vars
    typedef intel_aero_navigation::WaypointNavigationAction NavAction;
    typedef actionlib::SimpleActionClient<NavAction> NavClient;
    std::vector<std::shared_ptr<NavClient>> nav_clients;

    // SOCP vars
    std::map<std::tuple<int,int,int>, int> ijk_to_idx;
    std::map<int, std::tuple<int,int,int>> idx_to_ijk;
    int alpha_dim, y_dim;

    bool obstacleFree(const point_vec&);
    bool collisionFree(const point_vec&);
    double computeV(const point_vec& config,
                    const std::vector<arma::mat>& alpha,
                    bool debug);
    void probConstraints(const arma::mat& R_mean,
                         const arma::mat& R_var,
                         std::vector<arma::mat>& A_mats,
                         std::vector<arma::vec>& b_vecs,
                         std::vector<arma::vec>& c_vecs,
                         std::vector<arma::vec>& d_vecs,
                         int& idx,
                         bool divide_psi_inv_eps,
                         bool debug);
    void networkState(const point_vec& state,
                      arma::mat& R_mean,
                      arma::mat& R_var);
    bool SOCPsrv(const point_vec& config,
                 std::vector<arma::mat>& alpha,
                 double& slack, bool publish_qos, bool debug);
    bool SOCP(const point_vec& config,
              std::vector<arma::mat>& alpha,
              double& slack, bool publish_qos, bool debug);
    void publishNetworkUpdate(const std::vector<arma::mat>& alpha);

  public:

    NetworkPlanner() : costmap(grid_mapping::Point(0.0, 0.0), 0.2, 1, 1) {}
    NetworkPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    bool updateNetworkConfig();
    bool updateRouting();
    void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg, int idx);
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void setCommReqs(const CommReqs& reqs);
    void runPlanningLoop();
    void runRoutingLoop();
    void initSystem();
};


// combinatorial functions
std::vector<int> extract_inds(int num, const int agents);
std::vector<int> compute_combinations(const int len, const int num_digits);


} // namespace network_planner


#endif
