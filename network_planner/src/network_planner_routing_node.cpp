#include <network_planner/network_planner.h>


template<typename T>
void getParamWarn(const ros::NodeHandle& nh, std::string name, T& param, T default_val)
{
  if (!nh.getParam(name, param)) {
    std::stringstream ss;
    ss << default_val;
    std::string default_val_str = ss.str();
    ROS_WARN("[network_planner_node] failed to get ROS param \"%s\"; using default value %s", name.c_str(), default_val_str.c_str());
  } else {
    std::stringstream ss;
    ss << param;
    std::string val_str = ss.str();
    ROS_INFO("[network_planner_node] using %s for %s", val_str.c_str(), name.c_str());
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "network_planner");
  ros::NodeHandle nh, pnh("~");

  double margin, confidence;
  int source, dest;
  getParamWarn(pnh, "margin", margin, 0.1);
  getParamWarn(pnh, "confidence", confidence, 0.8);
  getParamWarn(pnh, "source", source, 1);
  getParamWarn(pnh, "dest", dest, 2);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  network_planner::NetworkPlanner np(nh, pnh);

  // 1 flow with 1 destination
  network_planner::CommReqs comm_reqs;
  network_planner::Flow flow1;
  // TODO make these params
  flow1.srcs.insert(source);
  flow1.dests.insert(dest);
  flow1.min_margin = margin;
  flow1.confidence = confidence;
  comm_reqs.push_back(flow1);

  np.setCommReqs(comm_reqs);
  np.runRoutingLoop();

  return 0;
}
