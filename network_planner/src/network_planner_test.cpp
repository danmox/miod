/*
  solve test SOCP problem
  expected results:
    alpha_ij_1
          0        0        0        0
          0        0   0.2110   0.3129
     1.0000        0        0        0
          0        0   0.5773        0
    slack = 0.023
 */
#include <network_planner/network_planner.h>


namespace network_planner {


class NPTest : public NetworkPlanner
{
  public:
    NPTest():
      NetworkPlanner()
    {
    }

    int runTest();
};

int NPTest::runTest()
{
  // set agent states
  team_config.clear();
  team_config.push_back(arma::vec3("0.0 0.0 0.0"));
  team_config.push_back(arma::vec3("30.0 0.0 0.0"));
  team_config.push_back(arma::vec3("10.0 2.0 0.0"));
  team_config.push_back(arma::vec3("20.0 -2.0 0.0"));

  total_agents = team_config.size();

  // initialize task spec
  network_planner::Flow flow;
  flow.srcs.insert(2);
  flow.dests.insert(1);
  flow.min_margin = 0.1;
  flow.confidence = 0.8;
  network_planner::CommReqs qos;
  qos.push_back(flow);
  setCommReqs(qos);

  // fetch network state
  arma::mat R_mean;
  arma::mat R_var;
  networkState(team_config, R_mean, R_var);

  // solve SOCP
  bool debug = true;
  double slack;
  std::vector<arma::mat> alpha_ij_k;
  if (!SOCP(R_mean, R_var, alpha_ij_k, slack, debug)) {
    ROS_ERROR("SOCP failed");
    return -1;
  }

  // show routes
  for (int k = 0; k < qos.size(); ++k) {
    printf("alpha_ij_%d\n", k+1);
    alpha_ij_k[k].elem(find(alpha_ij_k[k] < 0.01)).zeros();
    alpha_ij_k[k].print();
  }
  printf("slack = %.3f\n", slack);

  return 0;
}


} // namespace network_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "network_planner_test_node");

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  network_planner::NPTest npt;
  return npt.runTest();
}
