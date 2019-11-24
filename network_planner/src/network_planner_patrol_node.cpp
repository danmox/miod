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
  getParamWarn(pnh, "margin", margin, 0.1);
  getParamWarn(pnh, "confidence", confidence, 0.8);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  network_planner::NetworkPlanner np(nh, pnh);

  // Three flows with 2 destinations each
  network_planner::CommReqs comm_reqs;
  network_planner::Flow flow1;
  flow1.srcs.insert(1);
  flow1.dests.insert(2);
  flow1.dests.insert(3);
  flow1.min_margin = margin;
  flow1.confidence = confidence;
  comm_reqs.push_back(flow1);

  network_planner::Flow flow2;
  flow2.srcs.insert(2);
  flow2.dests.insert(1);
  flow2.dests.insert(3);
  flow2.min_margin = margin;
  flow2.confidence = confidence;
  comm_reqs.push_back(flow2);

  network_planner::Flow flow3;
  flow3.srcs.insert(3);
  flow3.dests.insert(1);
  flow3.dests.insert(2);
  flow3.min_margin = margin;
  flow3.confidence = confidence;
  comm_reqs.push_back(flow3);

  np.setCommReqs(comm_reqs);

  np.runPlanningLoop();

  return 0;
}
