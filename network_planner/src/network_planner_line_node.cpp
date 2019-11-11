#include <network_planner/network_planner.h>


#define NPN_INFO(fmt, ...) ROS_INFO("[network_planner_line_node] " fmt, ##__VA_ARGS__)
#define NPN_FATAL(fmt, ...) ROS_FATAL("[network_planner_line_node] " fmt, ##__VA_ARGS__)


template<typename T>
void getParamStrict(const ros::NodeHandle& nh, std::string param_name, T& param)
{
  if (!nh.getParam(param_name, param)) {
    ROS_FATAL("[routing_visualization_node] failed to get ROS param \"%s\"", param_name.c_str());
    exit(EXIT_FAILURE);
  }
}


// TODO read mission form YAML file
int main(int argc, char** argv)
{
  ros::init(argc, argv, "network_planner_line_node");
  ros::NodeHandle nh, pnh("~");

  double margin, confidence;
  int source, dest;
  getParamStrict(pnh, "margin", margin);
  getParamStrict(pnh, "confidence", confidence);
  getParamStrict(pnh, "source", source);
  getParamStrict(pnh, "destination", dest);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  network_planner::NetworkPlanner np(nh, pnh);

  // Three flows with 2 destinations each
  network_planner::CommReqs comm_reqs;
  network_planner::Flow flow;
  flow.srcs.insert(source);
  flow.dests.insert(dest);
  flow.min_margin = margin;
  flow.confidence = confidence;
  comm_reqs.push_back(flow);

  np.setCommReqs(comm_reqs);

  np.initSystem();
  np.runPlanningLoop();

  return 0;
}
