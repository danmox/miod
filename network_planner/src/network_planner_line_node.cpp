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

  // bi-directional flow between src and destination
  network_planner::CommReqs comm_reqs;
  network_planner::Flow flow1;
  flow1.srcs.insert(source);
  flow1.dests.insert(dest);
  flow1.min_margin = margin;
  flow1.confidence = confidence;
  comm_reqs.push_back(flow1);
  network_planner::Flow flow2;
  flow2.srcs.insert(dest);
  flow2.dests.insert(source);
  flow2.min_margin = margin;
  flow2.confidence = confidence;
  comm_reqs.push_back(flow2);

  np.setCommReqs(comm_reqs);

  np.runPlanningLoop();

  return 0;
}
