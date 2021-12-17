#include <network_planner/network_planner.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <armadillo_socp.h>

#include <functional>
#include <sstream>
#include <algorithm>
#include <tuple>
#include <map>

#include <boost/math/special_functions/erf.hpp>


namespace network_planner {


#define NP_INFO(fmt, ...) ROS_INFO("[network_planner] " fmt, ##__VA_ARGS__)
#define NP_WARN(fmt, ...) ROS_WARN("[network_planner] " fmt, ##__VA_ARGS__)
#define NP_ERROR(fmt, ...) ROS_ERROR("[network_planner] " fmt, ##__VA_ARGS__)
#define NP_FATAL(fmt, ...) ROS_FATAL("[network_planner] " fmt, ##__VA_ARGS__)
#define NP_DEBUG(fmt, ...) ROS_DEBUG("[network_planner] " fmt, ##__VA_ARGS__)


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
  use_safety_bdry(true),
  received_costmap(false),
  costmap(grid_mapping::Point(0.0, 0.0), 0.2, 1, 1)
{
  // fetch parameters from ROS parameter server:
  std::string pose_topic_prefix, pose_topic_suffix, nav_nodelet;
  double min_x, min_y, max_x, max_y;
  getParamStrict(nh, "/desired_altitude", desired_altitude);
  getParamStrict(pnh, "sample_count", sample_count);
  getParamStrict(pnh, "sample_variance", sample_var);
  getParamStrict(pnh, "collision_distance", collision_distance);
  getParamStrict(pnh, "minimum_update_rate", minimum_update_rate);
  getParamStrict(pnh, "pose_topic_prefix", pose_topic_prefix);
  getParamStrict(pnh, "pose_topic_suffix", pose_topic_suffix);
  getParamStrict(pnh, "nav_nodelet", nav_nodelet);
  getParamStrict(pnh, "world_frame", world_frame);
  if (!nh.getParam("/safety_x_min", safety_p_min(0)) ||
      !nh.getParam("/safety_y_min", safety_p_min(1)) ||
      !nh.getParam("/safety_x_max", safety_p_max(0)) ||
      !nh.getParam("/safety_y_max", safety_p_max(1))) {
    use_safety_bdry = false;
    NP_WARN("failed to get safety boundary parameters; continuing without using a safety boundary");
  }

  // fetch task agent ids
  XmlRpc::XmlRpcValue task_agent_ids;
  getParamStrict(nh, "/task_agent_ids", task_agent_ids);
  ROS_ASSERT(task_agent_ids.getType() == XmlRpc::XmlRpcValue::TypeArray);
  int idx = 0;
  for (int i = 0; i < task_agent_ids.size(); ++i) {
    ROS_ASSERT(task_agent_ids[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    int id = static_cast<int>(task_agent_ids[i]);
    std::string IP = std::string("10.42.0.") + std::to_string(id);

    agent_ips.push_back(IP);
    task_idcs.push_back(idx);
    id_to_idx[id]  = idx;
    id_to_ip[id]   =  IP;
    idx_to_ip[idx] =  IP;
    idx_to_id[idx] =  id;

    idx++;
  }
  task_count = task_agent_ids.size();

  // fetch comm agent ids
  XmlRpc::XmlRpcValue comm_agent_ids;
  getParamStrict(nh, "/comm_agent_ids", comm_agent_ids);
  ROS_ASSERT(comm_agent_ids.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < comm_agent_ids.size(); ++i) {
    ROS_ASSERT(comm_agent_ids[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    int id = static_cast<int>(comm_agent_ids[i]);
    std::string IP = std::string("10.42.0.") + std::to_string(id);

    agent_ips.push_back(IP);
    comm_idcs.push_back(idx);
    id_to_idx[id]  = idx;
    id_to_ip[id]   =  IP;
    idx_to_ip[idx] =  IP;
    idx_to_id[idx] =  id;

    idx++;
  }
  comm_count = comm_agent_ids.size();

  // initialize agent count dependent variables
  agent_count = task_count + comm_count;
  team_config = point_vec(agent_count);
  received_odom = std::vector<bool>(agent_count, false);

  // costmap for checking if candidate team configurations are valid
  map_sub = nh.subscribe("costmap", 10, &NetworkPlanner::mapCB, this);

  // subscribe to pose messages for team configuration
  namespace stdph = std::placeholders;
  for (int i = 0; i < agent_count; ++i) {
    std::stringstream ss;
    ss << pose_topic_prefix << "aero" << idx_to_id.at(i) << pose_topic_suffix;
    auto fcn = std::bind(&NetworkPlanner::poseCB, this, stdph::_1, i);
    pose_subs.push_back(nh.subscribe<geometry_msgs::PoseStamped>(ss.str(), 10, fcn));
  }

  // initialize navigation action clients
  for (int i = 0; i < comm_count; ++i) {
    std::stringstream ss;
    ss << "/aero" << idx_to_id.at(comm_idcs[i]) << "/" << nav_nodelet;
    std::string sn = ss.str();
    std::shared_ptr<NavClient> ac_ptr(new NavClient(sn.c_str(), true));
    nav_clients.push_back(ac_ptr);
    NP_INFO("connecting to action server %s", sn.c_str());
  }

  // publish visualization of gradient based planner
  viz_pub = nh.advertise<visualization_msgs::MarkerArray>("planner", 10);

  // publish network update messages
  net_pub = nh.advertise<routing_msgs::NetworkUpdate>("network_update", 10);

  // publish rate margin of source nodes
  qos_pub = nh.advertise<std_msgs::Float64MultiArray>("qos", 10);

  // SOCP service
  socp_srv = nh.serviceClient<socp::RobustRoutingSOCP>("RobustRoutingSOCP");

  // publish slack at each iteration
  slack_pub = nh.advertise<std_msgs::Float64>("slack", 10);
}


void NetworkPlanner::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg, int idx)
{
  team_config[idx](0) = msg->pose.position.x;
  team_config[idx](1) = msg->pose.position.y;
  team_config[idx](2) = msg->pose.position.z;
  received_odom[idx] = true;
}


void NetworkPlanner::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  NP_INFO("received costmap");
  costmap = Costmap(msg);
  received_costmap = true;
}


void NetworkPlanner::setCommReqs(const Flows& reqs)
{
  flows = reqs;
  num_flows = flows.size();

  // TODO remove num_dests?
  num_dests = flows.size();

  NP_INFO("received new communication requirements:");
  for (int k = 0; k < reqs.size(); ++k) {
    NP_INFO("  flow %d:", k+1);
    NP_INFO("    source: %ld", reqs[k].src);
    NP_INFO("    destination: %ld", reqs[k].dest);
    NP_INFO("    rate = %.2f", reqs[k].rate);
    NP_INFO("    confidence = %.2f", reqs[k].qos);
  }

  // TODO ignore more variables

  // build useful data structure for converting betweein i,j,k indices and
  // linear indices used for the optimization vars
  ijk_to_idx.clear();
  idx_to_ijk.clear();
  int ind = 0;
  for (int k = 0; k < num_flows; ++k) { // flow
    for (int i = 0; i < agent_count; ++i) { // source node
      for (int j = 0; j < agent_count; ++j) { // destination node
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

    double psi_inv_eps = PsiInv(flows[k].qos);
    if (!divide_psi_inv_eps)
      psi_inv_eps = 1.0;
    if (debug)
      printf("psi_inv_eps = %f\n", psi_inv_eps);

    for (int i = 0; i < agent_count; ++i) {
      A_mats[idx].zeros(y_dim, y_dim);
      b_vecs[idx].zeros(y_dim);
      c_vecs[idx].zeros(y_dim);
      d_vecs[idx].zeros(1);

      // only source and network nodes have constraints
      if (idx_to_id.at(i) != flows[k].dest) {

        // components for \bar{b}_i^k and \tilde{b}_i^k
        for (int j = 0; j < agent_count; ++j) {

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

        // rate margin for the source node
        if (idx_to_id.at(i) == flows[k].src)
          d_vecs[idx](0) = -flows[k].rate / psi_inv_eps;

      }

      // slack var
      c_vecs[idx](y_dim-1) = -1.0 / psi_inv_eps;

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


bool NetworkPlanner::SOCPsrv(const point_vec& config,
                             std::vector<arma::mat>& alpha,
                             double& slack,
                             bool publish_qos,
                             bool debug)
{
  // call Robust Routing SOCP service

  socp::RobustRoutingSOCP srv;
  for (const arma::vec3& pt : team_config) {
    geometry_msgs::Point point;
    point.x = pt(0);
    point.y = pt(1);
    point.z = pt(2);
    srv.request.config.push_back(point);
  }

  srv.request.flows = flows;
  for (int i = 0; i < agent_count; ++i)
    srv.request.idx_to_id.push_back(idx_to_id.at(i));

  //std::cout << "request:" << std::endl << srv.request << std::endl;

  if (!socp_srv.call(srv)) {
    NP_WARN("socp service call failed");
    return false;
  }

  if (srv.response.status.compare("optimal") != 0) {
    NP_ERROR("socp service returned with status: %s", srv.response.status.c_str());
    return false;
  }

  if (srv.response.routes.size() != agent_count * agent_count * num_flows) {
    NP_ERROR("socp service solution inconsistant");
    return false;
  }

  //std::cout << "response:" << std::endl << srv.response << std::endl;

  // unpack result

  slack = srv.response.obj_fcn;

  alpha = std::vector<arma::mat>(num_flows);
  for (int k = 0; k < num_flows; ++k) {
    alpha[k] = arma::zeros<arma::mat>(agent_count, agent_count);
    for (int i = 0; i < agent_count; ++i) {
      for (int j = 0; j < agent_count; ++j) {
        int ind = k*agent_count*agent_count + j*agent_count + i;
        alpha[k](i,j) = srv.response.routes[ind];
      }
    }
  }

  // publish delivered qos

  if (publish_qos) {

    // get predicted rates
    arma::mat R_mean;
    arma::mat R_var;
    networkState(config, R_mean, R_var);

    // probability constraints without dividing by psi_inv_eps

    int num_constraints = agent_count * num_flows;
    int idx = 0;
    std::vector<arma::mat> A_mats(num_constraints);
    std::vector<arma::vec> b_vecs(num_constraints);
    std::vector<arma::vec> c_vecs(num_constraints);
    std::vector<arma::vec> d_vecs(num_constraints);
    probConstraints(R_mean, R_var, A_mats, b_vecs, c_vecs, d_vecs, idx, false, debug);

    // construct solution vector from routing matrices
    arma::vec y = arma::zeros<arma::vec>(y_dim);
    for (const auto& idx_ijk : idx_to_ijk) {
      int i,j,k;
      std::tie(i,j,k) = idx_ijk.second;
      y[idx_ijk.first] = alpha[k](i,j);
    }

    std_msgs::Float64MultiArray qos_msg; // qos for each node
    for (int k = 0; k < num_flows; ++k) {
      int src_idx = id_to_idx.at(flows[k].src) + k*agent_count;

      arma::mat margin = arma::trans(c_vecs[src_idx]) * y;
      double var = pow(arma::norm(A_mats[src_idx] * y), 2.0);

      qos_msg.data.push_back(flows[k].rate);
      qos_msg.data.push_back(margin(0,0));
      qos_msg.data.push_back(flows[k].qos);
      qos_msg.data.push_back(var);
    }
    if (qos_pub) qos_pub.publish(qos_msg);
  }

  return true;
}


// TODO don't reallicate fixed constraints
bool NetworkPlanner::SOCP(const point_vec& config,
                          std::vector<arma::mat>& alpha,
                          double& slack,
                          bool publish_qos,
                          bool debug)
{
  // get predicted rates

  arma::mat R_mean;
  arma::mat R_var;
  networkState(config, R_mean, R_var);

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
    agent_count * num_flows // probability margin constraints (for every i,k)
    + agent_count           // sum_jk alpha_ijk <= 1 (channel Tx availability, for every node)
    + agent_count           // sum_ik alpha_ijk <= 1 (channel Rx availability, for every node)
    + 2 * alpha_dim          // 0 <= alpha_ijk <= 1 (timeshare, for every opt var)
    + 1;                     // s >= 0 (slack variable)

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
  // ||()|| <= c^Ty + 1

  for (int i = 0; i < agent_count; ++i) {

    // Tx
    A_mats[idx].zeros(0,0);   // dummy
    b_vecs[idx].zeros(0);     // dummy
    c_vecs[idx].zeros(y_dim);
    d_vecs[idx].ones(1);

    // Rx
    A_mats[idx+1].zeros(0,0);                   // dummy
    b_vecs[idx+1].zeros(0);     // dummy
    c_vecs[idx+1].zeros(y_dim);
    d_vecs[idx+1].ones(1);

    for (int k = 0; k < num_flows; ++k) {
      for (int j = 0; j < agent_count; ++j) {

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
    // ||()|| <= alpha_ij_k + 0.0 (i.e. 0.0 <= alpha_ij_k)
    A_mats[idx].zeros(0,0);   // dummy
    b_vecs[idx].zeros(0);     // dummy
    c_vecs[idx].zeros(y_dim);
    c_vecs[idx](i) = 1.0;
    d_vecs[idx].zeros(1);
    ++idx;

    // ||()|| <= -alpha_ij_k + 1.0 (i.e. alpha_ij_k <= 1.0)
    A_mats[idx].zeros(0,0);   // dummy
    b_vecs[idx].zeros(0);     // dummy
    c_vecs[idx].zeros(y_dim);
    c_vecs[idx](i) = -1.0;
    d_vecs[idx].ones(1);
    ++idx;
  }

  // slack constraint ||()|| <= s + 2.0 (i.e.: s >= -2.0: slack is unconstrained)

  A_mats[idx].zeros(0,0);     // dummy
  b_vecs[idx].zeros(0);       // dummy
  c_vecs[idx].zeros(y_dim);
  c_vecs[idx](y_dim-1) = 1.0;
  d_vecs[idx].zeros(1);
  d_vecs[idx](0) = 2.0;

  NP_DEBUG("number of constraints %d", num_constraints);

  //
  // solve SOCP
  //

  // objective function (maximize slack)
  arma::vec f_obj = arma::zeros<arma::vec>(y_dim);
  f_obj(y_dim-1) = -1; // SOCP solver is a minimizer

  arma::vec y_col;
  std::vector<arma::vec> z_vecs; // dual??
  std::vector<double> w_vec;     // dual??
  double primal_ub = 1.0;
  double primal_lb = -1e-9;
  double rel_tol = 1e-4;
  int ret = SolveSOCP(f_obj, A_mats, b_vecs, c_vecs, d_vecs,
                      y_col, z_vecs, w_vec, primal_ub, primal_lb,
                      rel_tol, false);

  if (ret != 0) {
    NP_WARN("SolveSOCP failed with return value: %d", ret);
    return false;
  }

  slack = y_col(y_dim-1);

  alpha = std::vector<arma::mat>(num_flows);
  for (int k = 0; k < num_flows; ++k) {
    alpha[k] = arma::zeros<arma::mat>(agent_count, agent_count);
    for (int i = 0; i < agent_count; ++i) {
      for (int j = 0; j < agent_count; ++j) {
        auto it = ijk_to_idx.find(std::make_tuple(i,j,k));
        if (it != ijk_to_idx.end())
          alpha[k](i,j) = y_col(it->second);
      }
    }
  }

  // publish delivered qos
  if (publish_qos) {
    std_msgs::Float64MultiArray qos_msg; // qos for each node
    arma::vec y_tmp = y_col;
    y_tmp(y_dim-1) = 0.0;
    for (int k = 0; k < num_flows; ++k) {
      int src_idx = id_to_idx.at(flows[k].src) + k*agent_count;

      double psi_inv_eps = PsiInv(flows[k].qos);
      arma::mat margin = psi_inv_eps * arma::trans(c_vecs[src_idx]) * y_tmp;
      double var = pow(arma::norm(A_mats[src_idx]*y_tmp), 2.0);

      qos_msg.data.push_back(flows[k].rate);
      qos_msg.data.push_back(margin(0,0));
      qos_msg.data.push_back(flows[k].qos);
      qos_msg.data.push_back(var);
    }
    if (debug) {
      for (const double val : qos_msg.data)
        printf("%.3f ", val);
      printf("\n");
    }
    if (qos_pub) qos_pub.publish(qos_msg);
  }

  if (debug) {
    // check constraints
    printf("\n");
    for (int i = 0; i < agent_count * num_flows; ++i) {
      double lhs = arma::norm(A_mats[i] * y_col);
      arma::mat rhs = arma::trans(c_vecs[i]) * y_col + d_vecs[i];
      printf("constraint %d: %.2f <= %.2f\n", i+1, lhs, rhs(0,0));
    }

    // solution
    printf("\nslack = %f\n", slack);
    for (int k = 0; k < num_flows; ++k) {
      printf("\nalpha_ij_%d:\n",k+1);
      for (int i = 0; i < agent_count; ++i) {
        printf("   ");
        for (int j = 0; j < agent_count; ++j) {
          if (alpha[k](i,j) >= 1e-4)
            printf("%6.4f   ", alpha[k](i,j));
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
  R_mean = arma::zeros<arma::mat>(agent_count, agent_count);
  R_var = arma::zeros<arma::mat>(agent_count, agent_count);
  for (int i = 0; i < agent_count; ++i) {
    for (int j = 0; j < agent_count; ++j) {
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


double NetworkPlanner::computeSlack(const point_vec& config,
                                    const std::vector<arma::mat>& alpha,
                                    bool debug)
{
  // get predicted rates
  arma::mat R_mean;
  arma::mat R_var;
  networkState(config, R_mean, R_var);

  int num_constraints = agent_count * num_flows;

  // compute probability constraints without dividing by psi_inv_eps
  int idx = 0;
  std::vector<arma::mat> A_mats(num_constraints);
  std::vector<arma::vec> b_vecs(num_constraints);
  std::vector<arma::vec> c_vecs(num_constraints);
  std::vector<arma::vec> d_vecs(num_constraints);
  probConstraints(R_mean, R_var, A_mats, b_vecs, c_vecs, d_vecs, idx, false, debug);

  // construct solution vector from routing matrices
  arma::vec y = arma::zeros<arma::vec>(y_dim);
  for (const auto& idx_ijk : idx_to_ijk) {
    int i,j,k;
    std::tie(i,j,k) = idx_ijk.second;
    y[idx_ijk.first] = alpha[k](i,j);
  }
  if (debug) y.print("y:");

  // compute: _bik - mik - psi*sqrt(~bik)
  idx = 0;
  arma::vec s_col(num_constraints);
  for (int k = 0; k < num_flows; ++k) {

    double psi_inv_eps = PsiInv(flows[k].qos);

    for (int i = 0; i < agent_count; ++i) {
      double margin_avg = arma::as_scalar((c_vecs[idx].t()) * y + d_vecs[idx]);
      double margin_std = arma::norm(A_mats[idx] * y);
      s_col(idx) = margin_avg - psi_inv_eps*margin_std;
      ++idx;
    }
  }
  if (debug) s_col.print("s_col:");

  return s_col.min();
}


double NetworkPlanner::computeV(const point_vec& config,
                                const std::vector<arma::mat>& alpha,
                                bool debug)
{
  // get predicted rates
  arma::mat R_mean;
  arma::mat R_var;
  networkState(config, R_mean, R_var);

  int num_constraints = agent_count * num_flows;

  // compute probability constraints without dividing by psi_inv_eps
  int idx = 0;
  std::vector<arma::mat> A_mats(num_constraints);
  std::vector<arma::vec> b_vecs(num_constraints);
  std::vector<arma::vec> c_vecs(num_constraints);
  std::vector<arma::vec> d_vecs(num_constraints);
  probConstraints(R_mean, R_var, A_mats, b_vecs, c_vecs, d_vecs, idx, false, debug);

  // construct solution vector from routing matrices
  arma::vec y = arma::zeros<arma::vec>(y_dim);
  for (const auto& idx_ijk : idx_to_ijk) {
    int i,j,k;
    std::tie(i,j,k) = idx_ijk.second;
    y[idx_ijk.first] = alpha[k](i,j);
  }
  if (debug) y.print("y:");

  // compute v(alpha(x),x')
  idx = 0;
  arma::vec v_col(num_constraints);
  for (int k = 0; k < num_flows; ++k) {

    double psi_inv_eps = PsiInv(flows[k].qos);

    for (int i = 0; i < agent_count; ++i) {
      v_col(idx) = arma::as_scalar((c_vecs[idx].t()) * y + d_vecs[idx]);
      v_col(idx) /= arma::norm(A_mats[idx] * y);
      v_col(idx) -= psi_inv_eps;
      ++idx;
    }
  }
  if (debug) v_col.print("v_col:");

  return v_col.min();
}


// NOTE vertical altitude is unreliable in our GPS/IMU/Baro configuration so we
// only consider x,y when determining if agents are in proximity
bool NetworkPlanner::collisionFree(const point_vec& config)
{
  for (int i = 0; i < config.size(); ++i) {
    for (int j = i+1; j < config.size(); ++j) {
      arma::vec3 dp = config[i] - config[j];
      dp[2] = 0.0;
      if (arma::norm(dp) < collision_distance) {
        return false;
      }
    }
  }
  return true;
}


bool NetworkPlanner::obstacleFree(const point_vec& config)
{
  if (received_costmap) {
    for (const arma::vec3& pt : config) {
      grid_mapping::Point pt2d(pt(0), pt(1));
      if (!costmap.inBounds(pt2d))
        continue;
      else if (costmap.data[costmap.positionToIndex(pt2d)] > 10)
        return false;
    }
  }
  return true;
}


bool NetworkPlanner::updateNetworkConfig()
{
  static point_vec x_star = team_config;

  // ensure the following conditions are met before planning:
  // 1) odom messages have been received for each agent
  // 2) a task specification has been received
  if (!all(received_odom, true)) {
    NP_WARN("unable to plan: odometry not available for all agents");
    return false;
  }
  if (flows.empty()) {
    NP_WARN("unable to plan: no task specification set");
    return false;
  }

  //
  // find optimal routing variables
  //

  // solve SOCP for team_config
  double slack;
  std::vector<arma::mat> alpha;
  if (!SOCPsrv(team_config, alpha, slack, true, false)) {
    NP_ERROR("no valid solution to SOCP found for team_config");
    return false;
  }
  printf("   slack = %f\n", slack);
  std_msgs::Float64 slack_msg;
  slack_msg.data = slack;
  slack_pub.publish(slack_msg);

  // solve SOCP for x_star

  // update task agent positions in x_star
  for (int i = 0; i < task_count; ++i)
    x_star[task_idcs[i]] = team_config[task_idcs[i]];

  double slack_star;
  std::vector<arma::mat> alpha_star;
  if (!SOCPsrv(x_star, alpha_star, slack_star, false, false)) {
    NP_ERROR("no valid solution to SOCP found for x_star");
    return false;
  }
  printf("   slack_star = %f\n", slack_star);

  // x_star, slack_star should be the higher of the two
  if (slack > slack_star) {
    x_star = team_config;
    slack_star = slack;
    alpha_star = alpha;
    printf("   \033[0;33mslack (%.3f) > slack_star (%.3f) : resetting x_star to team_config\033[0m\n", slack, slack_star);
  }

  //
  // sample based local search
  //

  // compute set of perturbed configurations

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

  // ensure current optimal configuration is collision free; since vax_star is
  // the target to beat, setting it to 0.0 has the effect of ensuring one of the
  // sampled configurations gets chosen (i.e. x_star is guaranteed not to be
  // renewed as the target configuration)

  double vax_star = 0.0;
  if (collisionFree(x_star))
    vax_star = computeV(x_star, alpha_star, false);
  else
    printf("   \033[0;33mx_star discarded due to collision\033[0m\n");
  printf("   initial: vax_star = %.3f\n", vax_star);

  // search for local configurations with better node margins

  int new_max_count = 0, proximity_skip_count = 0, safety_skip_count = 0;
  std::vector<bool> sample_mask(sample_count, true); // for later visualization use
  for (int j = 0; j < sample_count; ++j) {

    // form candidate config, perturbing only network agents
    point_vec x_prime = team_config;
    for (int i = 0; i < comm_count; ++i) {
      x_prime[comm_idcs[i]](0) = samples(2*i,j);
      x_prime[comm_idcs[i]](1) = samples(2*i+1,j);
    }

    // check for collisions
    if (!collisionFree(x_prime)) {
      sample_mask[j] = false;
      ++proximity_skip_count;
      continue;
    }

    // check if point is in safe zone
    if (use_safety_bdry) {
      bool safety_violation = false;
      for (int i = 0; i < comm_count; ++i) {
        arma::vec3 dp_min = x_prime[comm_idcs[i]] - safety_p_min;
        arma::vec3 dp_max = safety_p_max - x_prime[comm_idcs[i]];
        if (dp_min(0) < 0 || dp_min(1) < 0 || dp_max(0) < 0 || dp_max(1) < 0)
          safety_violation = true;
      }
      if (safety_violation) {
        sample_mask[j] = false;
        ++safety_skip_count;
        continue;
      }
    }

    // check if x_prime is a better config
    double vax_prime = computeV(x_prime, alpha, false);
    if (vax_prime > vax_star) {
      vax_star = vax_prime;
      x_star = x_prime;
      ++new_max_count;
    }
  }
  printf("   %d samples skipped due to proximity, max updated %d times\n", proximity_skip_count, new_max_count);
  printf("   final: vax_star = %.3f\n", vax_star);

  if (new_max_count == 0)
    printf("   \033[0;33mat local optimum\033[0m\n");

  /*
  // optimal direction
  point_vec dist = team_config;
  for (int i = 0; i < dist.size(); ++i)
    dist[i] = x_star[i] - team_config[i];

  printf("      team config:                x_star:                     dist:\n");
  for (int i = 0; i < dist.size(); ++i) {
    printf("   %2d |%7.2f %7.2f %7.2f|   |%7.2f %7.2f %7.2f|   |%7.2f %7.2f %7.2f|\n",
           idx_to_id.at(i), team_config[i](0), team_config[i](1), team_config[i](2),
           x_star[i](0), x_star[i](1), x_star[i](2), dist[i](0), dist[i](1), dist[i](2));
  }
  */

  //
  // send waypoint updates
  //

  if (new_max_count > 0) {
    printf("   sending navigation goal\n");
    for (int i = 0; i < comm_count; ++i) {
      geometry_msgs::Pose goal;
      goal.orientation.w = 1.0;
      goal.position.x = x_star[comm_idcs[i]](0);
      goal.position.y = x_star[comm_idcs[i]](1);
      goal.position.z = desired_altitude;

      intel_aero_navigation::WaypointNavigationGoal goal_msg;
      goal_msg.waypoints.push_back(goal);
      goal_msg.end_action = intel_aero_navigation::WaypointNavigationGoal::HOVER;
      goal_msg.header.frame_id = world_frame;
      goal_msg.header.stamp = ros::Time::now();

      nav_clients[i]->sendGoal(goal_msg);
    }
  }

  // print out routing solution
  //printRoutingTable(alpha);

  //
  // send network update command
  //

  bool bi_directional = false;
  publishNetworkUpdate(alpha, bi_directional);

  //
  // visualizations
  //

  // visualize the sampled points

  visualization_msgs::MarkerArray plan_viz;
  visualization_msgs::Marker sample_points;
  sample_points.header.stamp = ros::Time(0);
  sample_points.header.frame_id = world_frame;
  sample_points.ns = "plan_viz";
  sample_points.id = 0;
  sample_points.type = visualization_msgs::Marker::POINTS;
  sample_points.action = visualization_msgs::Marker::ADD;
  sample_points.scale.x = 0.05; // meter
  sample_points.scale.y = 0.05; // meter
  sample_points.color.r = 1.0;
  sample_points.color.a = 0.5;
  sample_points.pose.orientation.w = 1.0;
  for (int j = 0; j < sample_count; ++j) {
    for (int i = 0; i < comm_count; ++ i) {
      if (sample_mask[i*comm_count+j]) { // true if the sample was not skipped due to collision
        geometry_msgs::Point sampled_point;
        sampled_point.x = samples(2*i,j);
        sampled_point.y = samples(2*i+1,j);
        sampled_point.z = team_config[comm_idcs[i]](2);
        sample_points.points.push_back(sampled_point);
      }
    }
  }
  plan_viz.markers.push_back(sample_points);

  // visualize velocity commands

  visualization_msgs::Marker goal_config;
  goal_config.header.stamp = ros::Time(0);
  goal_config.header.frame_id = world_frame;
  goal_config.ns = "plan_viz";
  goal_config.id = 1;
  goal_config.type = visualization_msgs::Marker::SPHERE_LIST;
  goal_config.action = visualization_msgs::Marker::ADD;
  goal_config.color.b = 1.0;
  goal_config.color.a = 1.0;
  goal_config.scale.x = 1.0;
  goal_config.scale.y = 1.0;
  goal_config.scale.z = 1.0;
  goal_config.pose.orientation.w = 1.0;
  for (int i = 0; i < comm_count; ++i) {
    geometry_msgs::Point goal;
    goal.x = x_star[comm_idcs[i]](0);
    goal.y = x_star[comm_idcs[i]](1);
    goal.z = x_star[comm_idcs[i]](2);
    goal_config.points.push_back(goal);
  }
  plan_viz.markers.push_back(goal_config);
  if (viz_pub) viz_pub.publish(plan_viz);

  return true;
}


void NetworkPlanner::runPlanningLoop()
{
  static int iter_count = 1;

  //
  // initialize system
  //

  NP_INFO("waiting for action servers to start");
  for (auto& client : nav_clients)
    client->waitForServer();
  NP_INFO("action servers started");

  NP_INFO("waiting for odom");
  if (!all(received_odom, true)) ros::Rate(1).sleep();
  NP_INFO("odom received");

  ros::Rate init_loop_rate(1);
  for (int i = 5; i > 0; --i) {
    NP_INFO("starting network planner in %d...", i);
    ros::spinOnce();
    init_loop_rate.sleep();
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
    goal_msg.header.frame_id = world_frame;
    goal_msg.header.stamp = ros::Time::now();

    nav_clients[i]->sendGoal(goal_msg);
  }

  NP_INFO("waiting for agents to complete takeoff");
  for (const auto& client : nav_clients)
    client->waitForResult();

  NP_INFO("system ready");

  //
  // main planning loop
  //

  ros::Time last_call = ros::Time::now();
  ros::Rate loop_rate(minimum_update_rate);
  while (ros::ok()) {
    printf("\n\niteration %d\n\n", iter_count);

    updateNetworkConfig();

    printf("   update_duration = %.3f\n", (ros::Time::now()-last_call).toSec());
    last_call = ros::Time::now();

    ++iter_count;
    loop_rate.sleep();
  }
}


bool NetworkPlanner::updateRouting()
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
  if (flows.empty()) {
    NP_WARN("unable to plan: no task specification set");
    return false;
  }

  //
  // find optimal routing variables
  //

  // solve SOCP for team_config

  double slack;
  std::vector<arma::mat> alpha;
  if (!SOCPsrv(team_config, alpha, slack, true, false)) {
    NP_ERROR("no valid solution to SOCP found for team_config");
    return false;
  }
  printf("slack = %f\n", slack);
  std_msgs::Float64 slack_msg;
  slack_msg.data = slack;
  slack_pub.publish(slack_msg);

  // print out routing solution
  //printRoutingTable(alpha);

  //
  // send network update command
  //

  bool bi_directional = false;
  publishNetworkUpdate(alpha, bi_directional);

  return true;
}


void NetworkPlanner::printRoutingTable(const std::vector<arma::mat>& alpha)
{
  for (int k = 0; k < num_flows; ++k) {
    printf("   flow %d routes:\n", k+1);
    printf("      |");
    for (int i = 0; i < agent_count; ++i) {
      if (flows[k].src == idx_to_id.at(i))
        printf("  (s)%2d", idx_to_id.at(i));
      else if (flows[k].dest == idx_to_id.at(i))
        printf("  (d)%2d", idx_to_id.at(i));
      else
        printf("   %4d", idx_to_id.at(i));
    }
    printf("\n");
    printf("   ----");
    for (int i = 0; i < agent_count; ++i)
      printf("-------");
    printf("\n");
    for (int i = 0; i < agent_count; ++i) {
      printf("   %2d |", idx_to_id.at(i));
      for (int j = 0; j < agent_count; ++j) {
        if (alpha[k](i,j) > 0.01)
          printf("   %4.2f", alpha[k](i,j));
        else
          printf("      -");
      }
      printf("\n");
    }
  }
}



void NetworkPlanner::publishNetworkUpdate(const std::vector<arma::mat>& alpha, bool bi_directional)
{
  // TODO 1) ensure sum of probabilities is 1.0
  // TODO 2) ensure the source is not a gateway
  routing_msgs::NetworkUpdate net_cmd;
  for (int k = 0; k < num_flows; ++k) {

    //
    // forward flow
    //

    routing_msgs::PRTableEntry rt_entry;
    rt_entry.src  = id_to_ip.at(flows[k].src);
    rt_entry.dest = id_to_ip.at(flows[k].dest);

    for (int i = 0; i < agent_count; ++i) {

      // probabilistic routing table entry for node i (all outgoing routes)
      rt_entry.node = idx_to_ip.at(i);

      // check for outgoing traffic to all other nodes
      rt_entry.gateways.clear();
      for (int j = 0; j < agent_count; ++j) {

        // no self transmissions
        if (i == j)
          continue;

        // only include routing table entry for significant usage
        if (alpha[k](i,j) > 0.01) { // ignore small routing vars
          routing_msgs::ProbGateway prob_gateway;
          prob_gateway.IP = idx_to_ip.at(j);
          prob_gateway.prob = alpha[k](i,j);
          rt_entry.gateways.push_back(prob_gateway);
        }
      }

      net_cmd.routes.push_back(rt_entry);
    }

    //
    // back flow
    //

    if (!bi_directional)
      continue;

    // opposite for bi-directional flow
    rt_entry.src  = id_to_ip.at(flows[k].dest);
    rt_entry.dest = id_to_ip.at(flows[k].src);

    // back flow
    for (int i = 0; i < agent_count; ++i) {

      // probabilistic routing table entry for node i (all outgoing routes)
      rt_entry.node = idx_to_ip.at(i);

      // check for outgoing traffic to all other nodes
      rt_entry.gateways.clear();
      for (int j = 0; j < agent_count; ++j) {

        // no self transmissions
        if (i == j)
          continue;

        // only include routing table entry for significant usage
        if (alpha[k](j,i) > 0.01) { // ignore small routing vars
          routing_msgs::ProbGateway prob_gateway;
          prob_gateway.IP = idx_to_ip.at(j);
          prob_gateway.prob = alpha[k](j,i);
          rt_entry.gateways.push_back(prob_gateway);
        }
      }

      net_cmd.routes.push_back(rt_entry);
    }
  }
  if (net_pub) net_pub.publish(net_cmd); // needed for testing w/o ROS
}


void NetworkPlanner::runRoutingLoop()
{
  static int iter_count = 1;

  // init system

  NP_INFO("waiting for odom");
  if (!all(received_odom, true)) ros::Rate(1).sleep();
  NP_INFO("odom received");

  ros::Rate init_loop_rate(1);
  for (int i = 5; i > 0; --i) {
    NP_INFO("starting network planner in %d...", i);
    ros::spinOnce();
    init_loop_rate.sleep();
  }

  // main loop

  ros::Time last_call = ros::Time::now();
  ros::Rate loop_rate(minimum_update_rate);
  while (ros::ok()) {
    printf("\n\niteration %d\n\n", iter_count);

    updateRouting();

    printf("update_duration = %.3f\n", (ros::Time::now()-last_call).toSec());
    last_call = ros::Time::now();

    ++iter_count;
    loop_rate.sleep();
  }
}


} // namespace network_planner
