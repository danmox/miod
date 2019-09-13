#include <network_planner/network_planner.h>


namespace network_planner {


class NPTest : public NetworkPlanner
{
  public:
    NPTest():
      NetworkPlanner()
    {
    }

    int socpTest();
    int socpDebug();
    int networkConfigTest();
};


int NPTest::socpDebug()
{
  // set agent states
  team_config.clear();
  team_config.push_back(arma::vec3("0.0 0.0 0.05"));
  team_config.push_back(arma::vec3("15.0 0.0 0.05"));
  team_config.push_back(arma::vec3("5.0 5.0 1.81"));

  total_agents = team_config.size();

  // explicitly set channel model
  channel_sim = channel_simulator::ChannelSimulator(-53.0, 2.52, -70.0, 0.2, 6.0);

  // qos requirements
  network_planner::Flow flow1;
  flow1.srcs.insert(2);
  flow1.dests.insert(1);
  flow1.min_margin = 0.05;
  flow1.confidence = 0.7;
  network_planner::Flow flow2;
  flow2.srcs.insert(1);
  flow2.dests.insert(2);
  flow2.min_margin = 0.05;
  flow2.confidence = 0.7;
  network_planner::CommReqs qos;
  qos.push_back(flow1);
  qos.push_back(flow2);
  setCommReqs(qos);

  // solve SOCP
  bool debug = false;
  double slack;
  std::vector<arma::mat> alpha_ij_k;
  if (!SOCP(team_config, alpha_ij_k, slack, debug)) {
    ROS_ERROR("SOCP failed");
    return -1;
  }

  // show routes
  if (!debug) {
    for (int k = 0; k < qos.size(); ++k) {
      printf("alpha_ij_%d\n", k+1);
      alpha_ij_k[k].elem(find(alpha_ij_k[k] < 0.01)).zeros();
      alpha_ij_k[k].print();
    }
    printf("slack = %.3f\n", slack);
  }

  return 0;
}


int NPTest::socpTest()
{
  /*
    solve test SOCP problem

    settings:
    flow.srcs.insert(2);
    flow.dests.insert(1);
    flow.min_margin = 0.1;
    flow.confidence = 0.7;

    expected results:
    alpha_ij_1
      0        0        0        0
      0        0   0.2427   0.3339
      1.0000   0        0        0
      0        0   0.6093        0
    slack = 0.051
  */

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
  flow.confidence = 0.7;
  network_planner::CommReqs qos;
  qos.push_back(flow);
  setCommReqs(qos);

  // solve SOCP
  bool debug = true;
  double slack;
  std::vector<arma::mat> alpha_ij_k;
  if (!SOCP(team_config, alpha_ij_k, slack, debug)) {
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


int NPTest::networkConfigTest()
{
  // ensure requirements are met to run updateNetworkConfig()
  received_odom = std::vector<bool>(1, true);
  received_costmap = true;

  // set agent states
  team_config.clear();
  team_config.push_back(arma::vec3("0.0 0.0 0.05"));
  team_config.push_back(arma::vec3("15.0 0.0 0.05"));
  team_config.push_back(arma::vec3("5.0 5.0 1.81"));

  total_agents = team_config.size();

  // explicitly set channel model
  channel_sim = channel_simulator::ChannelSimulator(-53.0, 2.52, -70.0, 0.2, 6.0);

  // qos requirements
  network_planner::Flow flow1;
  flow1.srcs.insert(2);
  flow1.dests.insert(1);
  flow1.min_margin = 0.05;
  flow1.confidence = 0.7;
  network_planner::Flow flow2;
  flow2.srcs.insert(1);
  flow2.dests.insert(2);
  flow2.min_margin = 0.05;
  flow2.confidence = 0.7;
  network_planner::CommReqs qos;
  qos.push_back(flow1);
  qos.push_back(flow2);
  setCommReqs(qos);

  // set parameters
  total_agents = team_config.size();
  comm_count = 1;
  task_count = 2;
  comm_idcs.clear();
  for (int i = task_count; i < total_agents; ++i)
    comm_idcs.push_back(i);
  sample_var = 1.0;
  sample_count = 5000;

  updateNetworkConfig();
  updateNetworkConfig();

  return 0;
}


} // namespace network_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "network_planner_test_node");

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  if (argc > 1) {
    network_planner::NPTest npt;
    switch (atoi(argv[1])) {
      case 0:
        printf("running socpDebug()\n");
        return npt.socpDebug();
      case 1:
        printf("running socpTest()\n");
        return npt.socpTest();
      case 2:
        printf("running networkConfigTest()\n");
        return npt.networkConfigTest();
    }
  } else {
    printf("provide an argument\n");
  }

  return 0;
}
