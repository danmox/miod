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

  double margin1, margin2, confidence1, confidence2;
  getParamWarn(pnh, "margin1", margin1, 0.1);
  getParamWarn(pnh, "margin2", margin2, 0.1);
  getParamWarn(pnh, "confidence1", confidence1, 0.8);
  getParamWarn(pnh, "confidence2", confidence2, 0.8);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  network_planner::NetworkPlanner np(nh, pnh);

  // initialize task spec
  network_planner::CommReqs comm_reqs;
  network_planner::Flow flow1;
  flow1.srcs.insert(2);
  flow1.dests.insert(1);
  flow1.min_margin = margin1;
  flow1.confidence = confidence1;
  comm_reqs.push_back(flow1);
  network_planner::Flow flow2;
  flow2.srcs.insert(1);
  flow2.dests.insert(2);
  flow2.min_margin = margin2;
  flow2.confidence = confidence2;
  comm_reqs.push_back(flow2);

  np.setCommReqs(comm_reqs);

  np.initSystem();
  np.runPlanningLoop();

  return 0;
}
