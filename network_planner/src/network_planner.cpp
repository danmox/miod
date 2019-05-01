#include <network_planner/network_planner.h>
#include <geometry_msgs/Point.h>
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
#define NP_FATAL(fmt, ...) {ROS_FATAL("[NetworkPlanner] " fmt, ##__VA_ARGS__); exit(EXIT_FAILURE);}


double PsiInv(double eps) {
  return sqrt(2.0) * boost::math::erf_inv(2 * eps - 1);
}


NetworkPlanner::NetworkPlanner(ros::NodeHandle& nh_, ros::NodeHandle& pnh_) :
  nh(nh_),
  pnh(pnh_),
  channel_sim(false), // don't use map
  received_costmap(false),
  costmap(grid_mapping::Point(0.0, 0.0), 0.2, 1, 1)
{
  namespace stdph = std::placeholders;

  if (!nh.getParam("/task_agent_count", num_task_agents) ||
      !nh.getParam("/network_agent_count", num_network_agents))
    NP_FATAL("failed to fetch params from server");

  // initialize agent count dependent variables
  num_agents = num_task_agents + num_network_agents;
  team_config = point_vec(num_agents);
  received_odom = std::vector<bool>(num_agents, false);

  // TODO don't hard code this
  // the first n agents are task agents and the remaining m agents are network
  // agents
  for (int i = num_task_agents; i < num_agents; ++i)
    network_agent_inds.push_back(i);

  // costmap for checking if candidate team configurations are valid
  map_sub = nh.subscribe("/aero5/costmap_2d/costmap/costmap", 10,
                         &NetworkPlanner::mapCB, this);

  // subscribe to odom messages for team configuration
  namespace stdph = std::placeholders;
  for (int i = 1; i <= num_agents; ++i) {
    std::stringstream ss;
    ss << "/aero" << i << "/odom";
    std::string name = ss.str();
    auto fcn = std::bind(&NetworkPlanner::odomCB, this, stdph::_1, i-1);
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(name, 10, fcn);
    odom_subs.push_back(sub);
  }

  // initialize navigation action clients
  for (int i = num_task_agents+1; i < num_agents+1; ++i) { // names are 1 indexed
    std::stringstream ss;
    ss << "/aero" << i << "/waypoint_navigation_vel_nodelet";
    std::string sn = ss.str();
    std::shared_ptr<NavClient> ac_ptr(new NavClient(sn.c_str(), true));
    nav_clients.push_back(ac_ptr);
    NP_INFO("connecting to action server %s", sn.c_str());
  }
}


void NetworkPlanner::odomCB(const nav_msgs::Odometry::ConstPtr& msg, int idx)
{
  geometry_msgs::Point pt = msg->pose.pose.position;
  team_config[idx] = octomap::point3d(pt.x, pt.y, pt.z);
  received_odom[idx] = true;
}


void NetworkPlanner::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  NP_INFO("received costmap");
  costmap = Costmap(msg);
  received_costmap = true;
}


bool all(const std::vector<bool>& vec, bool val)
{
  return std::find(vec.begin(), vec.end(), !val) == vec.end();
}


void NetworkPlanner::setCommReqs(const CommReqs& reqs)
{
  comm_reqs = reqs;

  // build useful data structure for converting betweein i,j,k indices and
  // linear indices used for the optimization vars
  num_flows = comm_reqs.size();
  ijk_to_idx.clear();
  idx_to_ijk.clear();
  int ind = 0;
  for (int i = 0; i < num_agents; ++i) {
    for (int j = 0; j < num_agents; ++j) {
      for (int k = 0; k < num_flows; ++k) {
        if (i != j) { // TODO ignore more variables
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
    for (int i = 0; i < num_agents; ++i) {
      A_mats[idx] = arma::zeros<arma::mat>(y_dim, y_dim);
      b_vecs[idx] = arma::zeros<arma::vec>(y_dim);
      c_vecs[idx] = arma::zeros<arma::vec>(y_dim);
      d_vecs[idx] = arma::zeros<arma::vec>(1);

      for (int j = 0; j < num_agents; ++j) {
        // outgoing data
        auto it = ijk_to_idx.find(std::make_tuple(i,j,k));
        if (it != ijk_to_idx.end()) {
          A_mats[idx](it->second,it->second) = sqrt(R_var(i,j));
          c_vecs[idx](it->second) = R_mean(i,j) / psi_inv_eps;
        }

        // incoming data if *it is not a destination of flow k
        if (comm_reqs[k].dests.count(i+1) == 0) { // flow.dests is 1 indexed
          it = ijk_to_idx.find(std::make_tuple(j,i,k));
          if (it != ijk_to_idx.end()) {
            A_mats[idx](it->second,it->second) = -sqrt(R_var(j,i));
            c_vecs[idx](it->second) = -R_mean(j,i) / psi_inv_eps;
          }
        }
      }

      c_vecs[idx](y_dim-1) = -1.0 / psi_inv_eps; // slack var

      if (comm_reqs[k].sources.count(i+1) > 0) // flow.sources is 1 indexed
        d_vecs[idx](0) = -comm_reqs[k].qos / psi_inv_eps; // rate margin

      ++idx;
    }
  }

  if (debug) {
    for (int n = 0; n < num_agents*num_flows; ++n) { // num prob constraints
      printf("\ncoefficient %d:", n+1);
      printf("\nalpha_ij_k    A_mats\n");
      printf("----------   -------\n");
      for (int ind = 0; ind < alpha_dim; ++ind) {
        int i,j,k;
        std::tie(i,j,k) = idx_to_ijk[ind];
        double val = A_mats[n](ind,ind);
        if (fabs(val) > 1e-4)
          printf("alpha_%d%d_%d   %7.4f\n", i+1, j+1, k+1, val);
        else
          printf("alpha_%d%d_%d   %7.1f\n", i+1, j+1, k+1, 0.0);
      }
    }
  }
}


bool NetworkPlanner::SOCP(const arma::mat& R_mean,
                          const arma::mat& R_var,
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
    num_agents * num_flows // probability QoS constraints (for every i,k)
    + num_agents           // sum_jk alpha_ijk <= 1 (channel availability, for every i)
    + 2 * alpha_dim          // 0 <= alpha_ijk <= 1 (timeshare, for every opt var)
    + 1;                     // s >= 0 (slack variable)

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

  // channel availability constraints (sum_jk alpha_ijk <= 1)
  // ||0|| <= -c^Ty + 1

  for (int i = 0; i < num_agents; ++i) {
    A_mats[idx].set_size(0,0);                   // dummy
    b_vecs[idx] = arma::zeros<arma::vec>(0);     // dummy
    c_vecs[idx] = arma::zeros<arma::vec>(y_dim);
    d_vecs[idx] = arma::ones<arma::vec>(1);

    for (int k = 0; k < num_flows; ++k) {
      for (int j = 0; j < num_agents; ++j) {
        auto it = ijk_to_idx.find(std::make_tuple(i,j,k));
        if (it != ijk_to_idx.end()) {
          c_vecs[idx](it->second) = -1.0;
        }
      }
    }

    ++idx;
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
  std::vector<arma::vec> z_vecs; //??
  std::vector<double> w_vec; //??
  std::vector<double> primal_ub(y_dim, 1.0);
  std::vector<double> primal_lb(y_dim, -1e-9);

  int ret = SolveSOCP(f_obj, A_mats, b_vecs, c_vecs, d_vecs,
                      y_col, z_vecs, w_vec, primal_ub, primal_lb,
                      1e-4, false);

  if (ret != 0) {
    NP_WARN("SolveSOCP failed with return value: %d", ret);
    return false;
  }

  slack = y_col(y_dim-1);

  std::vector<arma::mat> alpha_ij_k(num_flows);
  for (int k = 0; k < num_flows; ++k) {
    alpha_ij_k[k] = arma::zeros<arma::mat>(num_agents, num_agents);
    for (int i = 0; i < num_agents; ++i) {
      for (int j = 0; j < num_agents; ++j) {
        auto it = ijk_to_idx.find(std::make_tuple(i,j,k));
        if (it != ijk_to_idx.end())
          alpha_ij_k[k](i,j) = y_col(it->second);
      }
    }
  }

  if (debug) {
    // check constraints
    printf("\n");
    for (int i = 0; i < num_agents * num_flows; ++i) {
      double lhs = arma::norm(A_mats[i] * y_col, 2);
      arma::mat rhs = arma::trans(c_vecs[i]) * y_col + d_vecs[i];
      printf("constraint %d: %.2f <= %.2f\n", i+1, lhs, rhs(0,0));
    }

    // solution
    printf("\nslack = %f\n", slack);
    for (int k = 0; k < num_flows; ++k) {
      printf("\nalpha_ij_%d:\n",k+1);
      for (int i = 0; i < num_agents; ++i) {
        printf("   ");
        for (int j = 0; j < num_agents; ++j) {
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
  R_mean = arma::zeros<arma::mat>(num_agents, num_agents);
  R_var = arma::zeros<arma::mat>(num_agents, num_agents);
  for (int i = 0; i < num_agents; ++i) {
    for (int j = 0; j < num_agents; ++j) {
      if (i == j) {
        R_mean(i,j) = R_var(i,j) = 0.0;
      } else {
        double xi = state[i].x(), yi = state[i].y();
        double xj = state[j].x(), yj = state[j].y();
        channel_sim.Predict(xi, yi, xj, yj, R_mean(i,j), R_var(i,j));
      }
    }
  }
}


double NetworkPlanner::computeV(const point_vec& config, bool debug)
{
  // get predicted rates
  arma::mat R_mean;
  arma::mat R_var;
  networkState(config, R_mean, R_var);

  int num_constraints = num_agents * num_flows;

  // constraint matrices / column vectors
  std::vector<arma::mat> A_mats(num_constraints);
  std::vector<arma::vec> b_vecs(num_constraints);
  std::vector<arma::vec> c_vecs(num_constraints);
  std::vector<arma::vec> d_vecs(num_constraints);

  // compute probability constraints without dividing by psi_inv_eps
  int idx = 0;
  probConstraints(R_mean, R_var, A_mats, b_vecs, c_vecs, d_vecs, idx, false, debug);

  // v(alpha(x),x') excludes slack variable
  arma::vec y_tmp = y_col;
  y_tmp(y_dim-1) = 0.0;
  if (debug) y_tmp.print("y_tmp:");

  // compute v(alpha(x),x')
  arma::vec v_col(num_constraints);
  idx = 0;
  for (int k = 0; k < num_flows; ++k) {
    double psi_inv_eps = PsiInv(comm_reqs[k].confidence);
    for (int i = 0; i < num_agents; ++i) {
      v_col(idx) = arma::as_scalar((c_vecs[idx].t())*y_tmp + d_vecs[idx]);
      v_col(idx) /= arma::norm(A_mats[idx] * y_tmp, 2);
      v_col(idx) -= psi_inv_eps;
      ++idx;
    }
  }
  if (debug) v_col.print("v_col:");

  return v_col.min();
}


bool collisionFree(const point_vec& config)
{
  for (int i = 0; i < config.size(); ++i)
    for (int j = i+1; j < config.size(); ++j)
      if ((config[i] - config[j]).norm() < 2.0)
        return false;
  return true;
}


bool obstacleFree(const point_vec& config, const Costmap& costmap)
{
  for (const octomap::point3d& pt : config) {
    grid_mapping::Point pt2d(pt.x(), pt.y());
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
    NP_WARN("unable to plan: some agent positions not received yet");
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
  bool success = SOCP(R_mean, R_var, slack, false);
  NP_DEBUG("found solution to SOCP with slack = %f", slack);

  if (!success) {
    NP_ERROR("unable to plan: no valid solution to SOCP found");
    return false;
  }

  //
  // find next best network configuration
  //

  // compute steps an agent might take in each direction

  int num_steps = 4;
  double step_radius = 1.0; // meters
  point_vec steps(num_steps);
  for (int i = 0; i < num_steps; ++i) {
    steps[i].x() = step_radius*cos(i*2.0*M_PI/num_steps);
    steps[i].y() = step_radius*sin(i*2.0*M_PI/num_steps);
  }
  steps.push_back(octomap::point3d(0,0,0)); // hold in place is an option too
  ++num_steps; // now there are 5 actions each robot can take

  // find config with best margin across all candidate team configurations

  std::vector<int> perms = compute_combinations(num_network_agents, num_steps);
  NP_INFO("searching for best configuration in %ld perturbations", perms.size());
  double v_star = 0.0;
  point_vec optimal_config = team_config;
  for (int perm : perms) {
    std::vector<int> step_inds = extract_inds(perm, num_network_agents);

    // form candidate config, perturbing only network agents
    point_vec candidate_config = team_config;
    for (int i = 0; i < num_network_agents; ++i)
      candidate_config[network_agent_inds[i]] += steps[step_inds[i]];

    // ensure candidate config is obstacle and collision free
    if (!obstacleFree(candidate_config, costmap) || !collisionFree(candidate_config)) {
      NP_DEBUG("candidate configuration not valid; skipping");
      continue;
    }

    //compute v(alpha(x),x)
    double v_alpha_x = computeV(candidate_config, false);
    if (v_alpha_x > v_star) {
      v_star = v_alpha_x;
      optimal_config = candidate_config;
    }
  }
  printf("v_star = %f\n", v_star);
  printf("current_config:\n");
  for (const auto& pt : team_config)
    printf("  (%6.2f, %6.2f, %6.2f)\n", pt.x(), pt.y(), pt.z());
  printf("opimal_config:\n");
  for (const auto& pt : optimal_config)
    printf("  (%6.2f, %6.2f, %6.2f)\n", pt.x(), pt.y(), pt.z());

  //
  // reconfigure to optimal value
  //

  NP_INFO("sending goals");
  for (int i = 0; i < num_network_agents; ++i) { // optimal config includes task agents
    geometry_msgs::Pose goal;
    goal.orientation.w = 1.0;
    goal.position.x = optimal_config[i+num_task_agents].x();
    goal.position.y = optimal_config[i+num_task_agents].y();
    goal.position.z = 3.0; // TODO make param

    intel_aero_navigation::WaypointNavigationGoal goal_msg;
    goal_msg.waypoints.push_back(goal);
    goal_msg.end_behavior = intel_aero_navigation::WaypointNavigationGoal::HOVER;

    nav_clients[i]->sendGoal(goal_msg);
  }

  NP_INFO("waiting for network agents to reach goals");
  for (const auto& client : nav_clients)
    client->waitForResult();

  return true;
}


void NetworkPlanner::runPlanningLoop()
{
  while (ros::ok())
    updateNetworkConfig();
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

  NP_INFO("sending takeoff goals");
  for (int i = 0; i < num_network_agents; ++i) { // optimal config includes task agents
    geometry_msgs::Pose goal;
    goal.orientation.w = 1.0;
    goal.position.x = team_config[i+num_task_agents].x();
    goal.position.y = team_config[i+num_task_agents].y();
    goal.position.z = 3.0; // TODO make param

    intel_aero_navigation::WaypointNavigationGoal goal_msg;
    goal_msg.waypoints.push_back(goal);
    goal_msg.end_behavior = intel_aero_navigation::WaypointNavigationGoal::HOVER;

    nav_clients[i]->sendGoal(goal_msg);
  }

  NP_INFO("waiting for agents to complete takeoff");
  for (const auto& client : nav_clients)
    client->waitForResult();

  NP_INFO("system ready");
}


} // namespace network_planner
