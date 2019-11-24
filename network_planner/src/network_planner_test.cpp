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
    int socpSrvTest();
    int computeSlackTest();
};


int NPTest::socpDebug()
{
  double patrol_rad = 20.0;
  double comm_rad = 8.0;
  int task_agent_count = 3;
  int comm_agent_count = 6;

  // set agent states

  team_config.clear();

  for (int i = 0; i < task_agent_count; ++i) {
    arma::vec3 pt;
    pt(0) = patrol_rad*cos(2.0*M_PI/task_agent_count*i);
    pt(1) = patrol_rad*sin(2.0*M_PI/task_agent_count*i);
    pt(2) = 0.0;
    team_config.push_back(pt);
  }

  team_config.push_back(arma::zeros<arma::vec>(3));
  for (int i = 0; i < comm_agent_count-1; ++i) {
    arma::vec3 pt;
    pt(0) = comm_rad*cos(2.0*M_PI/(comm_agent_count-1)*i);
    pt(1) = comm_rad*sin(2.0*M_PI/(comm_agent_count-1)*i);
    pt(2) = 0.0;
    team_config.push_back(pt);
  }

  for (int i = 0; i < team_config.size(); ++i)
    printf("agent %2d: (%6.2f, %6.2f, %6.2f)\n", i+1, team_config[i](0), team_config[i](1), team_config[i](2));

  agent_count = team_config.size();

  // explicitly set channel model
  channel_sim = channel_simulator::ChannelSimulator(-48.0, 2.52, -70.0, 0.2, 6.0);

  // qos requirements
  double margin = 0.05;
  double confidence = 0.7;
  network_planner::Flow flow1;
  flow1.srcs.insert(1);
  flow1.dests.insert(2);
  flow1.dests.insert(3);
  flow1.min_margin = margin;
  flow1.confidence = confidence;
  network_planner::Flow flow2;
  flow2.srcs.insert(2);
  flow2.dests.insert(1);
  flow2.dests.insert(3);
  flow2.min_margin = margin;
  flow2.confidence = confidence;
  network_planner::Flow flow3;
  flow3.srcs.insert(3);
  flow3.dests.insert(1);
  flow3.dests.insert(2);
  flow3.min_margin = margin;
  flow3.confidence = confidence;
  network_planner::CommReqs qos;
  qos.push_back(flow1);
  qos.push_back(flow2);
  qos.push_back(flow3);
  setCommReqs(qos);

  // solve SOCP
  bool debug = false;
  double slack;
  std::vector<arma::mat> alpha_ij_k;
  for (int i = 0; i < 5; ++i) {
    ros::Time t0 = ros::Time::now();
    if (!SOCP(team_config, alpha_ij_k, slack, false, debug)) {
      ROS_ERROR("SOCP failed");
      return -1;
    }
    printf("elapsed time: %.3f", (ros::Time::now()-t0).toSec());
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

  agent_count = team_config.size();

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
  if (!SOCP(team_config, alpha_ij_k, slack, false, debug)) {
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

  agent_count = team_config.size();

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
  agent_count = team_config.size();
  comm_count = 1;
  task_count = 2;
  comm_idcs.clear();
  for (int i = 0; i < task_count; ++i)
    task_idcs.push_back(i);
  for (int i = task_count; i < agent_count; ++i)
    comm_idcs.push_back(i);
  sample_var = 1.0;
  sample_count = 5000;

  updateNetworkConfig();
  updateNetworkConfig();

  return 0;
}


int NPTest::socpSrvTest()
{
  int task_agent_count = 3;
  int comm_agent_count = 1;

  // set agent states

  team_config.clear();

  team_config.push_back(arma::vec3({ 20.0,  -0.00, 0.05}));
  team_config.push_back(arma::vec3({-10.0,  17.32, 0.05}));
  team_config.push_back(arma::vec3({-10.0, -17.32, 0.05}));
  team_config.push_back(arma::vec3({  5.0,   5.00, 1.83}));

  for (int i = 0; i < team_config.size(); ++i)
    printf("agent %2d: (%6.2f, %6.2f, %6.2f)\n", i+1, team_config[i](0), team_config[i](1), team_config[i](2));

  agent_count = team_config.size();

  // explicitly set channel model
  channel_sim = channel_simulator::ChannelSimulator(-48.0, 2.52, -70.0, 0.2, 6.0);

  // qos requirements
  double margin = 0.05;
  double confidence = 0.7;
  network_planner::Flow flow1;
  flow1.srcs.insert(1);
  flow1.dests.insert(2);
  flow1.dests.insert(3);
  flow1.min_margin = margin;
  flow1.confidence = confidence;
  network_planner::Flow flow2;
  flow2.srcs.insert(2);
  flow2.dests.insert(1);
  flow2.dests.insert(3);
  flow2.min_margin = margin;
  flow2.confidence = confidence;
  network_planner::Flow flow3;
  flow3.srcs.insert(3);
  flow3.dests.insert(1);
  flow3.dests.insert(2);
  flow3.min_margin = margin;
  flow3.confidence = confidence;
  network_planner::CommReqs qos;
  qos.push_back(flow1);
  qos.push_back(flow2);
  qos.push_back(flow3);
  setCommReqs(qos);

  // solve SOCP
  bool debug = false;
  double slack;
  std::vector<arma::mat> alpha_ij_k;
  if (!SOCP(team_config, alpha_ij_k, slack, false, debug)) {
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


int NPTest::computeSlackTest()
{
  // set agent states
  team_config.clear();
  team_config.push_back(arma::vec3("0.0 0.0 0.0"));
  team_config.push_back(arma::vec3("30.0 0.0 0.0"));
  team_config.push_back(arma::vec3("10.0 2.0 0.0"));
  team_config.push_back(arma::vec3("20.0 -2.0 0.0"));

  agent_count = team_config.size();

  for (int i = 0; i < agent_count; ++i) {
    idx_to_id[i] = i+1;
    id_to_idx[i+1] = i;
  }

  // initialize task spec
  network_planner::CommReqs qos;
  network_planner::Flow flow1;
  flow1.srcs.insert(2);
  flow1.dests.insert(1);
  flow1.min_margin = 0.1;
  flow1.confidence = 0.7;
  qos.push_back(flow1);
  network_planner::Flow flow2;
  flow2.srcs.insert(1);
  flow2.dests.insert(2);
  flow2.min_margin = 0.1;
  flow2.confidence = 0.7;
  qos.push_back(flow2);
  setCommReqs(qos);

  // solve SOCP
  bool debug = false;
  double slack;
  std::vector<arma::mat> alpha;
  if (!SOCP(team_config, alpha, slack, false, debug)) {
    ROS_ERROR("SOCP failed");
    return -1;
  }
  printRoutingTable(alpha);

  debug = false;
  double computed_slack = computeSlack(team_config, alpha, false);
  printf("slack: %f\n", slack);
  printf("computed: %f\n", computed_slack);

  return 0;
}


} // namespace network_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "network_planner_test_node");

  //if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  //  ros::console::notifyLoggerLevelsChanged();

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
      case 3:
        printf("running socpSrvTest()\n");
        return npt.socpSrvTest();
      case 4:
        printf("running computeSlackTest()");
        return npt.computeSlackTest();
    }
  } else {
    printf("provide an argument\n");
  }

  return 0;
}
