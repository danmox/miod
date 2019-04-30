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
  channel_sim(true) // use map
{
  namespace stdph = std::placeholders;

  if (!nh.getParam("/task_agent_count", num_task_agents) ||
      !nh.getParam("/network_agent_count", num_network_agents))
    NP_FATAL("failed to fetch params from server");

  // initialize agent count dependent variables
  num_agents = num_task_agents + num_network_agents;
  team_config = point_vec(num_agents);
  received_odom = std::vector<bool>(num_agents, false);

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
}


void NetworkPlanner::odomCB(const nav_msgs::Odometry::ConstPtr& msg, int idx)
{
  geometry_msgs::Point pt = msg->pose.pose.position;
  team_config[idx] = octomap::point3d(pt.x, pt.y, pt.z);
  received_odom[idx] = true;
}


bool all(const std::vector<bool>& vec, bool val)
{
  return std::find(vec.begin(), vec.end(), !val) == vec.end();
}


bool NetworkPlanner::Plan()
{
  // ensure the following conditions are met before planning:
  // 1) odom messages have been received for each agent
  // 2) the channel_sim has received a map (i.e. is ready to predict)
  // 3) a task specification has been received
  if (!all(received_odom, true)) {
    NP_WARN("unable to plan: some agent positions not received yet");
    return false;
  }
  if (!channel_sim.mapSet()) {
    NP_WARN("unable to plan: channel simulator has not received a map yet");
    return false;
  }
  if (comm_reqs.empty()) {
    NP_WARN("unable to plan: no task specification set");
    return false;
  }

  // TODO construct possible configurations given current team state
  std::vector<point_vec> configs;
  configs.push_back(team_config);

  //
  // find next best network team configuration
  //

  for (auto& config : configs) {

    // get predicted rates

    arma::mat R_mean(num_agents, num_agents);
    arma::mat R_var(num_agents, num_agents);
    arma::mat dist(num_agents, num_agents, arma::fill::zeros);
    for (int i = 0; i < num_agents; ++i) {
      for (int j = 0; j < num_agents; ++j) {
        if (i == j) {
          dist(i,j) = R_mean(i,j) = R_var(i,j) = 0.0;
        } else {
          double xi = team_config[i].x(), yi = team_config[i].y();
          double xj = team_config[j].x(), yj = team_config[j].y();
          channel_sim.Predict(xi, yi, xj, yj, R_mean(i,j), R_var(i,j));
          dist(i,j) = sqrt((xj-xi)*(xj-xi) + (yj-yi)*(yj-yi));
        }
      }
    }

    //
    // SOCP constraints of the form ||Ay + b|| <= c^Ty + d
    //

    // form useful data structure for converting betweein i,j,k indices and
    // linear indices used for the optimization vars
    int num_flows = comm_reqs.size();
    std::map<std::tuple<int,int,int>, int> ijk_to_idx;
    int ind = 0;
    printf("\noptimization vars:\n  ");
    for (int i = 0; i < num_agents; ++i) {
      for (int j = 0; j < num_agents; ++j) {
        for (int k = 0; k < num_flows; ++k) {
          if (R_mean(i,j) > 0.0) {
            ijk_to_idx[std::make_tuple(i,j,k)] = ind++;
            printf("a_%d%d_%d(%d)   ", i+1, j+1, k+1, ind-1);
          } else {
            NP_DEBUG("ignoring alpha_%d%d_%d", i+1, j+1, k+1);
          }
        }
      }
    }
    printf("\n");
    int alpha_dim = ijk_to_idx.size(); // # optimization vars (excluding slack)
    int y_dim = alpha_dim + 1;         // # optimization vars (including slack)

    // total number of constraints for problem
    int num_constraints =
      num_agents * num_flows // probability QoS constraints (for every i,k)
      + num_agents           // sum_jk alpha_ijk <= 1 (channel availability, for every i)
      + 2 * alpha_dim          // 0 <= alpha_ijk <= 1 (timeshare, for every opt var)
      + 1;                     // s >= 0 (slack variable)

    std::vector<arma::mat> A_mats(num_constraints);
    std::vector<arma::vec> b_vecs(num_constraints);
    std::vector<arma::vec> c_vecs(num_constraints);
    std::vector<arma::vec> d_vecs(num_constraints);

    int idx = 0; // constraint index

    // probability constraints (second-order cones) ||Ay|| <= c^Ty + d

    for (int k = 0; k < num_flows; ++k) {
      double psi_inv_eps = PsiInv(comm_reqs[k].confidence);
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

        c_vecs[idx](y_dim-1) = -1.0 / psi_inv_eps;       // slack var

        if (comm_reqs[k].sources.count(i+1) > 0) // flow.sources is 1 indexed
          d_vecs[idx](0) = -comm_reqs[k].qos / psi_inv_eps; // rate margin

        ++idx;
      }
    }

    for (int i = 0; i < idx; ++i) {
      std::string name = std::string("A_mats[") + std::to_string(i) + std::string("]:");
      A_mats[i].raw_print(name.c_str());
      printf("\n");
    }

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

    arma::vec y_col; // optimization variables
    std::vector<arma::vec> z_vecs; //??
    std::vector<double> w_vec; //??
    std::vector<double> primal_ub(y_dim, 1.0);
    std::vector<double> primal_lb(y_dim, -1e-9);
    double tol = 1e-4;
    bool debug = false;

    /*
    if (debug) {
      for (int i = 0; i < num_constraints; ++i) {
        std::string index = std::string("[") + std::to_string(i) + std::string("]");
        std::string A_str = std::string("A_mats") + index + std::string(":");
        std::string b_str = std::string("b_mats") + index + std::string(":");
        std::string c_str = std::string("c_mats") + index + std::string(":");
        std::string d_str = std::string("d_mats") + index + std::string(":");
        printf("Constraint %d:\n", i+1);
        A_mats[i].raw_print(A_str.c_str());
        b_vecs[i].t().raw_print(b_str.c_str());
        c_vecs[i].t().raw_print(c_str.c_str());
        d_vecs[i].raw_print(d_str.c_str());
        printf("\n\n");
      }
    }
    */

    int ret = SolveSOCP(f_obj, A_mats, b_vecs, c_vecs, d_vecs,
                        y_col, z_vecs, w_vec, primal_ub, primal_lb,
                        tol, debug);

    if (ret != 0) {
      NP_WARN("SOCP failed, return value: %d", ret);
      continue;
    }

    // check constraints
    printf("\n");
    for (int i = 0; i < num_agents * num_flows; ++i) {
      double lhs = arma::norm(A_mats[i] * y_col, 2);
      arma::mat rhs = arma::trans(c_vecs[i]) * y_col + d_vecs[i];
      printf("constraint %d: %.2f <= %.2f\n", i+1, lhs, rhs(0,0));
    }

    // solution
    printf("\nslack = %f\n", y_col(y_dim-1));
    for (int k = 0; k < num_flows; ++k) {
      printf("\nalpha_ij_%d:\n",k+1);
      for (int i = 0; i < num_agents; ++i) {
        printf("   ");
        for (int j = 0; j < num_agents; ++j) {
          auto it = ijk_to_idx.find(std::make_tuple(i,j,k));
          if (it != ijk_to_idx.end() && y_col(it->second) >= 1e-4)
            printf("%6.4f   ", y_col(it->second));
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
    printf("\n");
    dist.print("dist:");
  }

  // choose configuration with highest slack variable
  return true;
}


} // namespace network_planner
