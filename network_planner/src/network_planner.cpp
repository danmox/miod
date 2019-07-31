#include <network_planner/network_planner.h>
#include <geometry_msgs/Point.h>
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


NetworkPlanner::NetworkPlanner(ros::NodeHandle& nh_, ros::NodeHandle& pnh_) :
  nh(nh_),
  pnh(pnh_),
  channel_sim(true), // use map
  received_costmap(false),
  costmap(grid_mapping::Point(0.0, 0.0), 0.2, 1, 1)
{
  // fetch parameters from ROS parameter server:
  std::vector<std::string> param_names = {"/task_agent_count",
                                          "/network_agent_count",
                                          "sample_count",
                                          "sample_variance",
                                          "/desired_altitude",
                                          "max_step"};
  std::vector<bool> success;
  success.push_back(nh.getParam(param_names[0], task_count));
  success.push_back(nh.getParam(param_names[1], comm_count));
  success.push_back(pnh.getParam(param_names[2], sample_count));
  success.push_back(pnh.getParam(param_names[3], sample_var));
  success.push_back(nh.getParam(param_names[4], desired_altitude));
  success.push_back(pnh.getParam(param_names[5], max_step));
  if (!all(success, true)) {
    for (int i = 0; i < success.size(); ++i)
      if (!success[i])
        NP_ERROR("failed to get ROS param: %s", param_names[i].c_str());
    NP_FATAL("failed to fetch all params from server");
    exit(EXIT_FAILURE);
  }

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
  map_sub = nh.subscribe("/aero5/costmap_2d/costmap/costmap", 10,
                         &NetworkPlanner::mapCB, this);

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

  // initialize navigation action clients
  for (int i = task_count+1; i < total_agents+1; ++i) { // names are 1 indexed
    std::stringstream ss;
    ss << "/aero" << i << "/waypoint_navigation_vel_nodelet";
    std::string sn = ss.str();
    std::shared_ptr<NavClient> ac_ptr(new NavClient(sn.c_str(), true));
    nav_clients.push_back(ac_ptr);
    NP_INFO("connecting to action server %s", sn.c_str());
  }

  // publish visualization of gradient based planner
  viz_pub = nh.advertise<visualization_msgs::MarkerArray>("planner",10);
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


void NetworkPlanner::setCommReqs(const CommReqs& reqs)
{
  comm_reqs = reqs;

  // build useful data structure for converting betweein i,j,k indices and
  // linear indices used for the optimization vars
  num_flows = comm_reqs.size();
  ijk_to_idx.clear();
  idx_to_ijk.clear();
  int ind = 0;
  for (int i = 0; i < total_agents; ++i) {
    for (int j = 0; j < total_agents; ++j) {
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
    for (int i = 0; i < total_agents; ++i) {
      A_mats[idx] = arma::zeros<arma::mat>(y_dim, y_dim);
      b_vecs[idx] = arma::zeros<arma::vec>(y_dim);
      c_vecs[idx] = arma::zeros<arma::vec>(y_dim);
      d_vecs[idx] = arma::zeros<arma::vec>(1);

      for (int j = 0; j < total_agents; ++j) {
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
    for (int n = 0; n < total_agents*num_flows; ++n) { // num prob constraints
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
    total_agents * num_flows // probability QoS constraints (for every i,k)
    + total_agents           // sum_jk alpha_ijk <= 1 (channel avail., for all i)
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

  for (int i = 0; i < total_agents; ++i) {
    A_mats[idx].set_size(0,0);                   // dummy
    b_vecs[idx] = arma::zeros<arma::vec>(0);     // dummy
    c_vecs[idx] = arma::zeros<arma::vec>(y_dim);
    d_vecs[idx] = arma::ones<arma::vec>(1);

    for (int k = 0; k < num_flows; ++k) {
      for (int j = 0; j < total_agents; ++j) {
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
    for (int i = 0; i < total_agents * num_flows; ++i) {
      double lhs = arma::norm(A_mats[i] * y_col, 2);
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

  int num_constraints = total_agents * num_flows;

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
    for (int i = 0; i < total_agents; ++i) {
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
  if (!SOCP(R_mean, R_var, slack, false)) {
    NP_ERROR("unable to plan: no valid solution to SOCP found");
    return false;
  }
  NP_DEBUG("found solution to SOCP with slack = %f", slack);

  //
  // find next best network configuration
  //

  // compute set of perturbed team configurations

  int n = 2*comm_count;
  arma::mat mean(n,1,arma::fill::zeros);
  for (int i = 0; i < comm_count; ++i) {
    mean(2*i,0) = team_config[comm_idcs[i]].x();
    mean(2*i+1,0) = team_config[comm_idcs[i]].y();
  }
  arma::mat var = sample_var * arma::mat(n,1,arma::fill::ones);
  arma::gmm_diag ndist;
  ndist.reset(n,1);
  ndist.set_means(mean);
  ndist.set_dcovs(var);
  arma::mat samples = ndist.generate(sample_count);

  // form visualization of samples
  visualization_msgs::MarkerArray plan_viz;
  visualization_msgs::Marker sample_points;
  sample_points.header.stamp = ros::Time(0);
  sample_points.header.frame_id = "world";
  sample_points.ns = "plan_viz";
  sample_points.id = 0;
  sample_points.type = visualization_msgs::Marker::POINTS;
  sample_points.action = visualization_msgs::Marker::ADD;
  sample_points.scale.x = 0.02; // meter
  sample_points.scale.y = 0.02; // meter
  sample_points.color.r = 1.0;
  sample_points.color.a = 0.8;
  for (int j = 0; j < sample_count; ++j) {
    for (int i = 0; i < comm_count; ++ i) {
      geometry_msgs::Point sampled_point;
      sampled_point.x = samples(2*i,j);
      sampled_point.y = samples(2*i+1,j);
      sampled_point.z = team_config[comm_idcs[i]].z();
      sample_points.points.push_back(sampled_point);
    }
  }
  plan_viz.markers.push_back(sample_points);

  // approximate gradient using samples

  NP_INFO("computing gradient with %d samples", sample_count);
  point_vec gradient(total_agents);
  double v_alpha_x = computeV(team_config, false);
  int used = 0; // some configs might be skipped due to obstacles/collisions
  for (int j = 0; j < sample_count; ++j) { // samples stored as mat column vecs
    // form candidate config, perturbing only network agents
    point_vec x_prime = team_config;
    for (int i = 0; i < comm_count; ++i) {
      octomap::point3d perturbation(samples(2*i,j), samples(2*i+1,j), 0.0);
      x_prime[comm_idcs[i]] += perturbation;
    }

    // ensure candidate config is obstacle and collision free
    if (!obstacleFree(x_prime, costmap) || !collisionFree(x_prime)) {
      NP_DEBUG("candidate configuration not valid; skipping");
      continue;
    }

    // update gradient estimate
    double v_alpha_x_prime = computeV(x_prime, false);
    for (int i = 0; i < comm_count; ++i) {
      // TODO switch over to arma::vecs for math operations
      octomap::point3d diff = team_config[comm_idcs[i]] - x_prime[comm_idcs[i]];
      double xi_norm = diff.norm();
      if (xi_norm > 1e-3) {
        gradient[comm_idcs[i]] += diff * ((v_alpha_x - v_alpha_x_prime) / xi_norm);
      }
    }
    ++used;
  }
  NP_INFO("skipped %d configs while computing the gradient", sample_count-used);

  // normalize gradient vectors so that the largest step is max_step
  double largest_norm = 0.0;
  for (const octomap::point3d& pt : gradient)
    if (pt.norm() > largest_norm)
      largest_norm = pt.norm();
  NP_INFO("largets gradient norm is %.2f", largest_norm);
  for (octomap::point3d& pt : gradient)
    pt *= (max_step / largest_norm);

  // add gradient directions to planning visualization
  visualization_msgs::Marker gradient_arrow;
  gradient_arrow.header.stamp = ros::Time(0);
  gradient_arrow.header.frame_id = "world";
  gradient_arrow.ns = "plan_viz";
  gradient_arrow.id = 0;
  gradient_arrow.type = visualization_msgs::Marker::ARROW;
  gradient_arrow.action = visualization_msgs::Marker::ADD;
  gradient_arrow.color.b = 1.0;
  gradient_arrow.color.a = 1.0;
  for (int i = 0; i < comm_count; ++i) {
    geometry_msgs::Point start;
    start.x = team_config[comm_idcs[i]].x();
    start.y = team_config[comm_idcs[i]].y();
    start.z = team_config[comm_idcs[i]].z();

    geometry_msgs::Point tip = start;
    tip.x += gradient[comm_idcs[i]].x();
    tip.y += gradient[comm_idcs[i]].y();
    tip.z += gradient[comm_idcs[i]].z(); // this should do nothing

    ++gradient_arrow.id; // each id must be different to be displayed together

    double arrow_length = gradient[comm_idcs[i]].norm();
    NP_INFO("gradient_arrow %d length is %.3f", i+1, arrow_length);
    gradient_arrow.points.clear();
    gradient_arrow.points.push_back(start);
    gradient_arrow.points.push_back(tip);
    gradient_arrow.scale.x = arrow_length/10.0; // shaft diameter
    gradient_arrow.scale.y = arrow_length/5.0;  // head diameter
    gradient_arrow.scale.z = arrow_length/3.0;  // head length

    plan_viz.markers.push_back(gradient_arrow);
  }
  viz_pub.publish(plan_viz);

  // compute optimal move from gradient

  // compute gradient update step
  // TODO use Algorithm 1 from Jon Fink thesis: 5.2.1 here
  point_vec new_config(gradient.size());
  for (int i = 0; i < total_agents; ++i)
    new_config[i] = gradient[i] + team_config[i];

  printf("v_alpha_x = %f\n", v_alpha_x);
  printf("current_config:\n");
  for (const auto& pt : team_config)
    printf("  (%6.2f, %6.2f, %6.2f)\n", pt.x(), pt.y(), pt.z());
  printf("gradient:\n");
  for (const auto& pt : new_config)
    printf("  (%6.2f, %6.2f, %6.2f)\n", pt.x(), pt.y(), pt.z());

  //
  // reconfigure to optimal value
  //

  // TODO change to velocity control

  NP_INFO("sending goals");
  for (int i = 0; i < comm_count; ++i) {
    geometry_msgs::Pose goal;
    goal.orientation.w = 1.0;
    goal.position.x = new_config[comm_idcs[i]].x();
    goal.position.y = new_config[comm_idcs[i]].y();
    goal.position.z = desired_altitude;

    intel_aero_navigation::WaypointNavigationGoal goal_msg;
    goal_msg.waypoints.push_back(goal);
    goal_msg.end_behavior = intel_aero_navigation::WaypointNavigationGoal::HOVER;

    NP_INFO("sending goal to agend %d", comm_idcs[i]+1);
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
  for (int i = 0; i < comm_count; ++i) { // optimal config includes task agents
    geometry_msgs::Pose goal;
    goal.orientation.w = 1.0;
    goal.position.x = team_config[i+task_count].x();
    goal.position.y = team_config[i+task_count].y();
    goal.position.z = desired_altitude;

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
