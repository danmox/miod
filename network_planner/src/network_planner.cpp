#include <network_planner/network_planner.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <armadillo_socp.h>

#include <functional>
#include <sstream>
#include <algorithm>
#include <tuple>
#include <map>

#include <boost/math/special_functions/erf.hpp>


namespace network_planner {


#define NP_DEBUG(fmt, ...) ROS_DEBUG("[NetworkPlanner] " fmt, ##__VA_ARGS__)
#define NP_INFO(fmt, ...) ROS_INFO("[NetworkPlanner] " fmt, ##__VA_ARGS__)
#define NP_WARN(fmt, ...) ROS_WARN("[NetworkPlanner] " fmt, ##__VA_ARGS__)
#define NP_ERROR(fmt, ...) ROS_ERROR("[NetworkPlanner] " fmt, ##__VA_ARGS__)
#define NP_FATAL(fmt, ...) ROS_FATAL("[NetworkPlanner] " fmt, ##__VA_ARGS__)


inline double PsiInv(double eps) {
  return sqrt(2.0) * boost::math::erf_inv(2 * eps - 1);
}


bool all(const std::vector<bool>& vec, bool val)
{
  return std::find(vec.begin(), vec.end(), !val) == vec.end();
}


template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}


double clamp(const double var, const double max_val)
{
  if (abs(var) > max_val)
    return sgn(var)*max_val;
  return var;
}


template<typename T>
void getParamStrict(const ros::NodeHandle& nh, std::string param_name, T& param)
{
  if (!nh.getParam(param_name, param)) {
    NP_FATAL("failed to get ROS param \"%s\"", param_name.c_str());
    exit(EXIT_FAILURE);
  }
}


NetworkPlanner::NetworkPlanner(ros::NodeHandle& nh_, ros::NodeHandle& pnh_) :
  nh(nh_),
  pnh(pnh_),
  channel_sim(nh_),
  update_duration(2.0), // conservative 2.0s initial update duration estimate
  received_costmap(false),
  costmap(grid_mapping::Point(0.0, 0.0), 0.2, 1, 1)
{
  // fetch parameters from ROS parameter server:
  getParamStrict(nh, "/task_agent_count", task_count);
  getParamStrict(nh, "/network_agent_count", comm_count);
  getParamStrict(nh, "/desired_altitude", desired_altitude);
  getParamStrict(pnh, "sample_count", sample_count);
  getParamStrict(pnh, "sample_variance", sample_var);
  getParamStrict(pnh, "max_velocity", max_velocity);

  // initialize agent count dependent variables
  total_agents = task_count + comm_count;
  team_config = point_vec(total_agents);
  received_odom = std::vector<bool>(total_agents, false);

  // TODO don't hard code this; pass parameter vectors
  // the first n agents are task agents and the remaining m agents are network
  // agents
  for (int i = task_count; i < total_agents; ++i)
    comm_idcs.push_back(i);

  // costmap for checking if candidate team configurations are valid
  map_sub = nh.subscribe("costmap", 10, &NetworkPlanner::mapCB, this);

  // subscribe to odom messages for team configuration
  namespace stdph = std::placeholders;
  for (int i = 1; i <= total_agents; ++i) {
    std::stringstream ss;
    ss << "/aero" << i << "/odom";
    std::string name = ss.str();
    auto fcn = std::bind(&NetworkPlanner::odomCB, this, stdph::_1, i-1);
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(name, 10, fcn);
    odom_subs.push_back(sub);
  }

  // publish velocity messages to update network team configuration
  for (int i = 0; i < comm_count; ++i) {
    std::string name = "/aero" + std::to_string(comm_idcs[i]+1) + "/vel_cmd";
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(name, 10);
    vel_pubs.push_back(pub);
  }

  // initialize navigation action clients
  for (int i = task_count+1; i < total_agents+1; ++i) { // names are 1 indexed
    std::stringstream ss;
    ss << "/aero" << i << "/gazebo_vel_nav_nodelet";
    std::string sn = ss.str();
    std::shared_ptr<NavClient> ac_ptr(new NavClient(sn.c_str(), true));
    nav_clients.push_back(ac_ptr);
    NP_INFO("connecting to action server %s", sn.c_str());
  }

  // publish visualization of gradient based planner
  viz_pub = nh.advertise<visualization_msgs::MarkerArray>("planner", 10);

  // publish network update messages
  net_pub = nh.advertise<routing_msgs::NetworkUpdate>("network_update", 10);
}


void NetworkPlanner::odomCB(const nav_msgs::Odometry::ConstPtr& msg, int idx)
{
  team_config[idx](0) = msg->pose.pose.position.x;
  team_config[idx](1) = msg->pose.pose.position.y;
  team_config[idx](2) = msg->pose.pose.position.z;
  received_odom[idx] = true;
}


void NetworkPlanner::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  NP_INFO("received costmap");
  costmap = Costmap(msg);
  received_costmap = true;
}


void NetworkPlanner::setCommReqs(const CommReqs& reqs)
{
  comm_reqs = reqs;
  num_flows = comm_reqs.size();

  num_dests = 0;
  for (const auto& flow : comm_reqs)
    num_dests += flow.dests.size();

  NP_INFO("received new communication requirements:");
  for (int k = 0; k < reqs.size(); ++k) {
    NP_INFO("  flow %d:", k+1);
    std::stringstream src_ss, dest_ss;
    for (const int src : reqs[k].srcs)
      src_ss << " " << src;
    std::string src_str = src_ss.str();
    NP_INFO("    sources:%s", src_str.c_str());
    for (const int dest : reqs[k].dests)
      dest_ss << " " << dest;
    std::string dest_str = dest_ss.str();
    NP_INFO("    destinations:%s", dest_str.c_str());
    NP_INFO("    margin = %.2f", reqs[k].min_margin);
    NP_INFO("    confidence = %.2f", reqs[k].confidence);
  }

  // TODO ignore more variables

  // build useful data structure for converting betweein i,j,k indices and
  // linear indices used for the optimization vars
  ijk_to_idx.clear();
  idx_to_ijk.clear();
  int ind = 0;
  for (int k = 0; k < num_flows; ++k) { // flow
    for (int i = 0; i < total_agents; ++i) { // source node

      // TODO this is right; however, it sometimes causes availability
      // constraints to be all 0 which causes the SOCP solver to fail; ignoring
      // it for now as taking them out still produces the expected solution
      /*
      // destination nodes don't rebroadcast
      if (comm_reqs[k].dests.count(i+1) > 0) { // ids in flow.dests are 1 indexed
        NP_DEBUG("ignoring alpha_%d*_%d", i+1, k+1);
        continue;
      }
      */

      for (int j = 0; j < total_agents; ++j) { // destination node
        if (i != j) {
          std::tuple<int,int,int> subs = std::make_tuple(i,j,k);
          idx_to_ijk[ind] = subs;
          ijk_to_idx[subs] = ind++;
        } else {
          NP_DEBUG("ignoring alpha_%d%d_%d", i+1, j+1, k+1);
        }
      }
    }
  }

  alpha_dim = ijk_to_idx.size(); // # optimization vars (excluding slack)
  y_dim = alpha_dim + 1;         // # optimization vars (including slack)
}


void NetworkPlanner::probConstraints(const arma::mat& R_mean,
                                     const arma::mat& R_var,
                                     std::vector<arma::mat>& A_mats,
                                     std::vector<arma::vec>& b_vecs,
                                     std::vector<arma::vec>& c_vecs,
                                     std::vector<arma::vec>& d_vecs,
                                     int& idx,
                                     bool divide_psi_inv_eps,
                                     bool debug)
{
  // probability constraints (second-order cones) ||Ay|| <= c^Ty + d
  for (int k = 0; k < num_flows; ++k) {

    double psi_inv_eps = PsiInv(comm_reqs[k].confidence);
    if (!divide_psi_inv_eps)
      psi_inv_eps = 1.0;
    if (debug) printf("psi_inv_eps = %f\n", psi_inv_eps);

    for (int i = 0; i < total_agents; ++i) {
      A_mats[idx] = arma::zeros<arma::mat>(y_dim, y_dim);
      b_vecs[idx] = arma::zeros<arma::vec>(y_dim);
      c_vecs[idx] = arma::zeros<arma::vec>(y_dim);
      d_vecs[idx] = arma::zeros<arma::vec>(1);

      // destination nodes do not have constraints
      if (comm_reqs[k].dests.count(i+1) > 0) // ids in flow.dests are 1 indexed
        continue;

      // components for \bar{b}_i^k and \tilde{b}_i^k
      for (int j = 0; j < total_agents; ++j) {

        // outgoing data
        auto it_out = ijk_to_idx.find(std::make_tuple(i,j,k));
        if (it_out != ijk_to_idx.end()) {
          A_mats[idx](it_out->second,it_out->second) = sqrt(R_var(i,j));
          c_vecs[idx](it_out->second) = R_mean(i,j) / psi_inv_eps;
        }

        // incoming data (NOTE: destination rebroadcast routing variables are
        // not included in ijk_to_idx/idx_to_ijk)
        auto it_in = ijk_to_idx.find(std::make_tuple(j,i,k));
        if (it_in != ijk_to_idx.end()) {
          A_mats[idx](it_in->second,it_in->second) = sqrt(R_var(j,i));
          c_vecs[idx](it_in->second) = -R_mean(j,i) / psi_inv_eps;
        }
      }

      // slack var
      c_vecs[idx](y_dim-1) = -1.0 / psi_inv_eps;

      // rate margin for the source node
      if (comm_reqs[k].srcs.count(i+1) > 0) // ids in flow.srcs are 1 indexed
        d_vecs[idx](0) = -comm_reqs[k].min_margin / psi_inv_eps;

      ++idx;
    }
  }

  if (debug) {
    for (int n = 0; n < idx; ++n) { // num prob constraints
      printf("\ncoefficient %d:", n+1);
      printf("\nalpha_ij_k    A_mats    B_mats\n");
        printf("----------   -------   -------\n");
      for (int ind = 0; ind < alpha_dim; ++ind) {
        int i,j,k;
        std::tie(i,j,k) = idx_to_ijk[ind];
        double A_val = A_mats[n](ind,ind);
        double c_val = c_vecs[n](ind);
        if (fabs(A_val) > 1e-4 || fabs(c_val) > 1e-4)
          printf("alpha_%d%d_%d   %7.4f   %7.4f\n", i+1, j+1, k+1, A_val, c_val);
        else
          printf("alpha_%d%d_%d   %7.1f   %7.1f\n", i+1, j+1, k+1, 0.0, 0.0);
      }
    }
  }
}


// TODO don't reallicate fixed constraints
bool NetworkPlanner::SOCP(const arma::mat& R_mean,
                          const arma::mat& R_var,
                          std::vector<arma::mat>& alpha_ij_k,
                          double& slack,
                          bool debug)
{
  //
  // form SOCP constraints of the form ||Ay + b|| <= c^Ty + d
  //

  if (debug) {
    printf("\noptimization vars:\n");
    for (int ind = 0; ind < alpha_dim; ++ind) {
      int i,j,k;
      std::tie(i,j,k) = idx_to_ijk[ind];
      printf("a_%d%d_%d(%d)\n", i+1, j+1, k+1, ind);
    }
    printf("\n");
  }

  // total number of constraints for problem

  int num_constraints =
    total_agents * num_flows - num_dests // probability margin constraints (for every i,k except for destination nodes)
    + total_agents                       // sum_jk alpha_ijk <= 1 (channel Tx availability, for every node)
    + total_agents                       // sum_ik alpha_ijk <= 1 (channel Rx availability, for every node)
    + 2 * alpha_dim                      // 0 <= alpha_ijk <= 1 (timeshare, for every opt var)
    + 1;                                 // s >= 0 (slack variable)

  // TODO better initialization
  // constraint matrices / column vectors
  std::vector<arma::mat> A_mats(num_constraints);
  std::vector<arma::vec> b_vecs(num_constraints);
  std::vector<arma::vec> c_vecs(num_constraints);
  std::vector<arma::vec> d_vecs(num_constraints);

  int idx = 0; // constraint index

  // probability constraints (second-order cones) ||Ay|| <= c^Ty + d
  // this function requires pre allocated vectors and advances idx according to
  // the number of constraints inserted
  probConstraints(R_mean, R_var, A_mats, b_vecs, c_vecs, d_vecs, idx, true, debug);

  // Tx: sum_jk alpha_ijk <= 1
  // Rx: sum_ik alpha_ijk <= 1
  // ||0y + 0|| <= c^Ty + 1

  for (int i = 0; i < total_agents; ++i) {

    // Tx
    A_mats[idx].set_size(0,0);                   // dummy
    b_vecs[idx] = arma::zeros<arma::vec>(0);     // dummy
    c_vecs[idx] = arma::zeros<arma::vec>(y_dim);
    d_vecs[idx] = arma::ones<arma::vec>(1);

    // Rx
    A_mats[idx+1].set_size(0,0);                   // dummy
    b_vecs[idx+1] = arma::zeros<arma::vec>(0);     // dummy
    c_vecs[idx+1] = arma::zeros<arma::vec>(y_dim);
    d_vecs[idx+1] = arma::ones<arma::vec>(1);

    for (int k = 0; k < num_flows; ++k) {
      for (int j = 0; j < total_agents; ++j) {

        // Tx
        auto it_tx = ijk_to_idx.find(std::make_tuple(i,j,k));
        if (it_tx != ijk_to_idx.end()) {
          c_vecs[idx](it_tx->second) = -1.0;
        }

        // Rx
        auto it_rx = ijk_to_idx.find(std::make_tuple(j,i,k));
        if (it_rx != ijk_to_idx.end()) {
          c_vecs[idx+1](it_rx->second) = -1.0;
        }

      }
    }

    idx += 2;
  }

  // timeshare constraints (routing variable bounds: 0 <= alpha_ij_k <= 1)

  for (int i = 0; i < alpha_dim; ++i) {
    // ||0.0|| <= alpha_ij_k + 0.0 (i.e. 0.0 <= alpha_ij_k)
    A_mats[idx].set_size(0,0);                   // dummy
    b_vecs[idx] = arma::zeros<arma::vec>(0);     // dummy
    c_vecs[idx] = arma::zeros<arma::vec>(y_dim);
    c_vecs[idx](i) = 1.0;
    d_vecs[idx] = arma::zeros<arma::vec>(1);
    ++idx;

    // ||0.0|| <= -alpha_ij_k + 1.0 (i.e. alpha_ij_k <= 1.0)
    A_mats[idx].set_size(0,0);                   // dummy
    b_vecs[idx] = arma::zeros<arma::vec>(0);     // dummy
    c_vecs[idx] = arma::zeros<arma::vec>(y_dim);
    c_vecs[idx](i) = -1.0;
    d_vecs[idx] = arma::ones<arma::vec>(1);
    ++idx;
  }

  // slack constraint ||0.0|| <= s + 0.0 (i.e.: s >= 0)

  A_mats[idx].set_size(0,0);                   // dummy
  b_vecs[idx] = arma::zeros<arma::vec>(0);     // dummy
  c_vecs[idx] = arma::zeros<arma::vec>(y_dim);
  c_vecs[idx](y_dim-1) = 1.0;
  d_vecs[idx] = arma::zeros<arma::vec>(1);

  NP_DEBUG("number of constraints %d", num_constraints);

  //
  // solve SOCP
  //

  // objective function (maximize slack)
  arma::vec f_obj = arma::zeros<arma::vec>(y_dim);
  f_obj(y_dim-1) = -1; // SOCP solver is a minimizer

  y_col.clear();
  std::vector<arma::vec> z_vecs; // dual??
  std::vector<double> w_vec;     // dual??
  std::vector<double> primal_ub(y_dim, 1.0);
  std::vector<double> primal_lb(y_dim, -1e-9);

  int ret = SolveSOCP(f_obj, A_mats, b_vecs, c_vecs, d_vecs,
                      y_col, z_vecs, w_vec, primal_ub, primal_lb,
                      1e-4, debug);

  if (ret != 0) {
    NP_WARN("SolveSOCP failed with return value: %d", ret);
    return false;
  }

  slack = y_col(y_dim-1);

  alpha_ij_k = std::vector<arma::mat>(num_flows);
  for (int k = 0; k < num_flows; ++k) {
    alpha_ij_k[k] = arma::zeros<arma::mat>(total_agents, total_agents);
    for (int i = 0; i < total_agents; ++i) {
      for (int j = 0; j < total_agents; ++j) {
        auto it = ijk_to_idx.find(std::make_tuple(i,j,k));
        if (it != ijk_to_idx.end())
          alpha_ij_k[k](i,j) = y_col(it->second);
      }
    }
  }

  if (debug) {
    // check constraints
    printf("\n");
    for (int i = 0; i < total_agents * num_flows - num_dests; ++i) {
      double lhs = arma::norm(A_mats[i] * y_col);
      arma::mat rhs = arma::trans(c_vecs[i]) * y_col + d_vecs[i];
      printf("constraint %d: %.2f <= %.2f\n", i+1, lhs, rhs(0,0));
    }

    // solution
    printf("\nslack = %f\n", slack);
    for (int k = 0; k < num_flows; ++k) {
      printf("\nalpha_ij_%d:\n",k+1);
      for (int i = 0; i < total_agents; ++i) {
        printf("   ");
        for (int j = 0; j < total_agents; ++j) {
          if (alpha_ij_k[k](i,j) >= 1e-4)
            printf("%6.4f   ", alpha_ij_k[k](i,j));
          else
            printf("%6d   ", 0);
        }
        printf("\n");
      }
      printf("\n");
    }

    // Rij
    R_mean.print("R_mean:");
    printf("\n");
    R_var.print("R_var:");
  }

  return true;
}


void NetworkPlanner::networkState(const point_vec& state,
                                  arma::mat& R_mean,
                                  arma::mat& R_var)
{
  R_mean = arma::zeros<arma::mat>(total_agents, total_agents);
  R_var = arma::zeros<arma::mat>(total_agents, total_agents);
  for (int i = 0; i < total_agents; ++i) {
    for (int j = 0; j < total_agents; ++j) {
      if (i == j) {
        R_mean(i,j) = R_var(i,j) = 0.0;
      } else {
        geometry_msgs::Point pi, pj;
        pi.x = state[i](0);
        pi.y = state[i](1);
        pi.z = desired_altitude;
        pj.x = state[j](0);
        pj.y = state[j](1);
        pj.z = desired_altitude;
        channel_sim.predict(pi, pj, R_mean(i,j), R_var(i,j));
      }
    }
  }
}


double NetworkPlanner::computebik(const point_vec& config, bool debug)
{
  // get predicted rates
  arma::mat R_mean;
  arma::mat R_var;
  networkState(config, R_mean, R_var);

  int num_constraints = total_agents * num_flows - num_dests;

  // compute probability constraints without dividing by psi_inv_eps
  int idx = 0;
  std::vector<arma::mat> A_mats(num_constraints);
  std::vector<arma::vec> b_vecs(num_constraints);
  std::vector<arma::vec> c_vecs(num_constraints);
  std::vector<arma::vec> d_vecs(num_constraints);
  probConstraints(R_mean, R_var, A_mats, b_vecs, c_vecs, d_vecs, idx, false, debug);

  // b_ik - m_ik excludes slack variable
  arma::vec y_tmp = y_col;
  y_tmp(y_dim-1) = 0.0;
  if (debug) y_tmp.print("y_tmp:");

  // compute b_ik - m_ik
  idx = 0;
  arma::vec bik_col(num_constraints);
  for (int k = 0; k < num_flows; ++k) {
    for (int i = 0; i < total_agents; ++i) {

      // destination nodes do not have constraints
      if (comm_reqs[k].dests.count(i+1) > 0) // ids in flow.dests are 1 indexed
        continue;

      // b_ik - m_ik = cT*y + d (NOTE: d is -m_ik)
      bik_col(idx) = arma::as_scalar((c_vecs[idx].t()) * y_tmp + d_vecs[idx]);
      ++idx;
    }
  }
  if (debug) bik_col.print("bik_col:");

  return bik_col.min();
}


double NetworkPlanner::computeV(const point_vec& config, bool debug)
{
  // get predicted rates
  arma::mat R_mean;
  arma::mat R_var;
  networkState(config, R_mean, R_var);

  int num_constraints = total_agents * num_flows - num_dests;

  // compute probability constraints without dividing by psi_inv_eps
  int idx = 0;
  std::vector<arma::mat> A_mats(num_constraints);
  std::vector<arma::vec> b_vecs(num_constraints);
  std::vector<arma::vec> c_vecs(num_constraints);
  std::vector<arma::vec> d_vecs(num_constraints);
  probConstraints(R_mean, R_var, A_mats, b_vecs, c_vecs, d_vecs, idx, false, debug);

  // v(alpha(x),x') excludes slack variable
  arma::vec y_tmp = y_col;
  y_tmp(y_dim-1) = 0.0;
  if (debug) y_tmp.print("y_tmp:");

  // compute v(alpha(x),x')
  idx = 0;
  arma::vec v_col(num_constraints);
  for (int k = 0; k < num_flows; ++k) {

    double psi_inv_eps = PsiInv(comm_reqs[k].confidence);

    for (int i = 0; i < total_agents; ++i) {

      // destination nodes do not have constraints
      if (comm_reqs[k].dests.count(i+1) > 0) // ids in flow.dests are 1 indexed
        continue;

      v_col(idx) = arma::as_scalar((c_vecs[idx].t()) * y_tmp + d_vecs[idx]);
      v_col(idx) /= arma::norm(A_mats[idx] * y_tmp);
      v_col(idx) -= psi_inv_eps;
      ++idx;
    }
  }
  if (debug) v_col.print("v_col:");

  return v_col.min();
}


bool collisionFree(const point_vec& config)
{
  for (int i = 0; i < config.size(); ++i) {
    for (int j = i+1; j < config.size(); ++j) {
      if (arma::norm(config[i] - config[j]) < 2.0) {
        return false;
      }
    }
  }
  return true;
}


bool obstacleFree(const point_vec& config, const Costmap& costmap)
{
  for (const arma::vec3& pt : config) {
    grid_mapping::Point pt2d(pt(0), pt(1));
    if (!costmap.inBounds(pt2d))
      continue;
    else if (costmap.data[costmap.positionToIndex(pt2d)] > 10)
      return false;
  }
  return true;
}


bool NetworkPlanner::updateNetworkConfig()
{
  // ensure the following conditions are met before planning:
  // 1) odom messages have been received for each agent
  // 2) the channel_sim has received a map (i.e. is ready to predict)
  // 3) a task specification has been received
  // 4) a costmap has been received so that candidate configs can be vetted
  if (!all(received_odom, true)) {
    NP_WARN("unable to plan: odometry not available for all agents");
    return false;
  }
  /*
  if (!channel_sim.mapSet()) {
    NP_WARN("unable to plan: channel simulator has not received a map yet");
    return false;
  }
  */
  if (comm_reqs.empty()) {
    NP_WARN("unable to plan: no task specification set");
    return false;
  }
  if (!received_costmap) {
    NP_WARN("unable to plan: no costmap received yet");
    return false;
  }

  //
  // find optimal routing variables for current team configuration
  //

  // get predicted rates

  arma::mat R_mean;
  arma::mat R_var;
  networkState(team_config, R_mean, R_var);

  // solve SOCP

  double slack;
  std::vector<arma::mat> alpha_ij_k;
  if (!SOCP(R_mean, R_var, alpha_ij_k, slack, false)) {
    NP_ERROR("unable to plan: no valid solution to SOCP found");
    return false;
  }
  printf("found solution to SOCP with slack = %f\n", slack);

  //
  // compute the gradient of the team config w.r.t the min comm margin
  //

  // compute set of perturbed team configurations

  int n = 2 * comm_count; // 2D
  arma::mat mean(n, 1, arma::fill::zeros);
  for (int i = 0; i < comm_count; ++i) {
    mean(2*i,0) = team_config[comm_idcs[i]](0);
    mean(2*i+1,0) = team_config[comm_idcs[i]](1);
  }

  arma::mat var = sample_var * arma::mat(n, 1, arma::fill::ones);

  arma::gmm_diag ndist;
  ndist.reset(n,1);
  ndist.set_means(mean);
  ndist.set_dcovs(var);
  arma::mat samples = ndist.generate(sample_count); // n x sample_count

  // visualize the sampled points

  visualization_msgs::MarkerArray plan_viz;
  visualization_msgs::Marker sample_points;
  sample_points.header.stamp = ros::Time(0);
  sample_points.header.frame_id = "world";
  sample_points.ns = "plan_viz";
  sample_points.id = 0;
  sample_points.type = visualization_msgs::Marker::POINTS;
  sample_points.action = visualization_msgs::Marker::ADD;
  sample_points.scale.x = 0.05; // meter
  sample_points.scale.y = 0.05; // meter
  sample_points.color.r = 1.0;
  sample_points.color.a = 0.5;
  for (int j = 0; j < sample_count; ++j) {
    for (int i = 0; i < comm_count; ++ i) {
      geometry_msgs::Point sampled_point;
      sampled_point.x = samples(2*i,j);
      sampled_point.y = samples(2*i+1,j);
      sampled_point.z = team_config[comm_idcs[i]](2);
      sample_points.points.push_back(sampled_point);
    }
  }
  plan_viz.markers.push_back(sample_points);

  // search for local configurations with better node margins

  point_vec x_star = team_config;
  double vax0 = computeV(team_config, false);
  double vax_star = vax0;
  printf("vax0 = %.3f\n", vax0);
  for (int j = 0; j < sample_count; ++j) {

    // form candidate config, perturbing only network agents
    point_vec x_prime = team_config;
    for (int i = 0; i < comm_count; ++i) {
      x_prime[comm_idcs[i]](0) = samples(2*i,j);
      x_prime[comm_idcs[i]](1) = samples(2*i+1,j);
    }

    // check if config is better
    double vax_min = computeV(x_prime, false); // TODO this is not efficient
    if (vax_min > vax_star) {
      vax_star = vax_min;
      x_star = x_prime;
    }
  }
  printf("vax_star = %.3f\n", vax_star);

  if (abs(vax0 - vax_star) < 1e-6) {
    NP_WARN("better configuration not found: vax0 = %.3f, vax_star = %.3f", vax0, vax_star);
    for (int i = 0; i < comm_count; ++i)
      vel_pubs[i].publish(geometry_msgs::Twist()); // zero command
    return true;
  }

  // optimal direction
  point_vec dist = team_config;
  for (int i = 0; i < dist.size(); ++i)
    dist[i] = x_star[i] - team_config[i];

  printf("team config:                 x_star:                      dist:\n");
  for (int i = 0; i < dist.size(); ++i) {
    printf("(%7.2f, %7.2f, %7.2f), (%7.2f, %7.2f, %7.2f), (%7.2f, %7.2f, %7.2f)\n",
           team_config[i](0), team_config[i](1), team_config[i](2),
           x_star[i](0), x_star[i](1), x_star[i](2), dist[i](0), dist[i](1), dist[i](2));
  }

  //
  // visualize commanded velocity directions
  //

  visualization_msgs::Marker vel_arrow;
  vel_arrow.header.stamp = ros::Time(0);
  vel_arrow.header.frame_id = "world";
  vel_arrow.ns = "plan_viz";
  vel_arrow.id = 0;
  vel_arrow.type = visualization_msgs::Marker::ARROW;
  vel_arrow.action = visualization_msgs::Marker::ADD;
  vel_arrow.color.b = 1.0;
  vel_arrow.color.a = 1.0;
  for (int i = 0; i < comm_count; ++i) {
    geometry_msgs::Point start;
    start.x = team_config[comm_idcs[i]](0);
    start.y = team_config[comm_idcs[i]](1);
    start.z = team_config[comm_idcs[i]](2);

    double length_scale = 3.0; // max norm == 1 so max arrow length will be 5m
    geometry_msgs::Point tip = start;
    tip.x += length_scale * dist[comm_idcs[i]](0);
    tip.y += length_scale * dist[comm_idcs[i]](1);
    tip.z += length_scale * dist[comm_idcs[i]](2); // this should do nothing

    ++vel_arrow.id; // messages with the same ns and id are overwritten

    double arrow_length = length_scale * arma::norm(dist[comm_idcs[i]]);
    vel_arrow.points.clear();
    vel_arrow.points.push_back(start);
    vel_arrow.points.push_back(tip);
    vel_arrow.scale.x = arrow_length / 10.0; // shaft diameter
    vel_arrow.scale.y = arrow_length / 5.0;  // head diameter
    vel_arrow.scale.z = arrow_length / 3.0;  // head length

    plan_viz.markers.push_back(vel_arrow);
  }
  viz_pub.publish(plan_viz);

  //
  // send velocity control messages
  //

  for (int i = 0; i < comm_count; ++i) {
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = clamp(dist[comm_idcs[i]](0) / update_duration, max_velocity);
    cmd_msg.linear.y = clamp(dist[comm_idcs[i]](1) / update_duration, max_velocity);
    vel_pubs[i].publish(cmd_msg);
  }

  // print out routing solution
  for (int k = 0; k < num_flows; ++k) {
    alpha_ij_k[k].elem(find(alpha_ij_k[k] < 0.01)).zeros(); // clear out small values
    alpha_ij_k[k].print(std::string("flow ") + std::to_string(k+1) + std::string(" routes:"));
  }

  //
  // send network update command
  //

  routing_msgs::NetworkUpdate net_cmd;
  for (int i = 0; i < total_agents; ++i) {

    // probabilistic routing table entry for node i (all outgoing routes)
    routing_msgs::PRTableEntry rt_entry;
    rt_entry.node[0] = i;

    // check for outgoing traffic to all other nodes
    for (int j = 0; j < total_agents; ++j) {

      // no self transmissions
      if (i == j)
        continue;

      // combine alpha_ij for all flows k
      double alpha_ij = 0;
      for (int k = 0; k < num_flows; ++k) {
        alpha_ij += alpha_ij_k[k](i,j);
      }

      // only include routing table entry for significant usage
      if (alpha_ij > 0.01) { // ignore small routing vars
        routing_msgs::ProbGateway gway;
        gway.IP[0] = j;
        gway.prob = alpha_ij;
        rt_entry.gateways.push_back(gway);
      }
    }

    net_cmd.routes.push_back(rt_entry);
  }
  net_pub.publish(net_cmd);

  return true;
}


void NetworkPlanner::runPlanningLoop()
{
  double alpha = 0.3; // weighted average degree
  ros::Time last_call = ros::Time::now();
  while (ros::ok()) {
    updateNetworkConfig();
    update_duration = alpha*update_duration + (1-alpha)*(ros::Time::now() - last_call).toSec();
    last_call = ros::Time::now();
    printf("update_duration = %.3f\n", update_duration);
  }
}


void NetworkPlanner::initSystem()
{
  NP_INFO("waiting for action servers to start");
  for (auto& client : nav_clients)
    client->waitForServer();
  NP_INFO("action servers started");

  NP_INFO("waiting for odom");
  if (!all(received_odom, true)) ros::Rate(1).sleep();
  NP_INFO("odom received");

  ros::Rate loop_rate(1);
  for (int i = 5; i > 0; --i) {
    ROS_INFO("starting network planner in %d...", i);
    ros::spinOnce();
    loop_rate.sleep();
  }

  NP_INFO("sending takeoff goals");
  for (int i = 0; i < comm_count; ++i) {
    geometry_msgs::Pose goal;
    goal.orientation.w = 1.0;
    goal.position.x = team_config[comm_idcs[i]](0);
    goal.position.y = team_config[comm_idcs[i]](1);
    goal.position.z = desired_altitude;

    intel_aero_navigation::WaypointNavigationGoal goal_msg;
    goal_msg.waypoints.push_back(goal);
    goal_msg.end_action = intel_aero_navigation::WaypointNavigationGoal::HOVER;
    // TODO this should not be hardcoded!!
    goal_msg.header.frame_id = "world";
    goal_msg.header.stamp = ros::Time::now();

    nav_clients[i]->sendGoal(goal_msg);
  }

  NP_INFO("waiting for agents to complete takeoff");
  for (const auto& client : nav_clients)
    client->waitForResult();

  NP_INFO("system ready");
}


} // namespace network_planner
