#include <network_planner/network_planner.h>


#define NPN_INFO(fmt, ...) ROS_INFO("[network_planner_net_patrol_node] " fmt, ##__VA_ARGS__)
#define NPN_ERROR(fmt, ...) ROS_ERROR("[network_planner_net_patrol_node] " fmt, ##__VA_ARGS__)
#define NPN_FATAL(fmt, ...) ROS_FATAL("[network_planner_net_patrol_node] " fmt, ##__VA_ARGS__)


template<typename T>
void getParamStrict(const ros::NodeHandle& nh, std::string param_name, T& param)
{
  if (!nh.getParam(param_name, param)) {
    NPN_FATAL("failed to get ROS param \"%s\"", param_name.c_str());
    exit(EXIT_FAILURE);
  }
}


// TODO read mission form YAML file
int main(int argc, char** argv)
{
  ros::init(argc, argv, "network_planner_line_node");
  ros::NodeHandle nh, pnh("~");

  std::string network_node_type;
  double margin, confidence;
  XmlRpc::XmlRpcValue source_nodes, destination_nodes;
  getParamStrict(pnh, "margin", margin);
  getParamStrict(pnh, "confidence", confidence);
  getParamStrict(pnh, "network_node_type", network_node_type);
  getParamStrict(nh, "/source_nodes", source_nodes);
  getParamStrict(nh, "/destination_nodes", destination_nodes);

  // source / destination nodes
  ROS_ASSERT(source_nodes.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(destination_nodes.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(source_nodes.size() == destination_nodes.size());
  network_planner::CommReqs comm_reqs;
  for (int i = 0; i < source_nodes.size(); ++i) {
    ROS_ASSERT(source_nodes[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(destination_nodes[i].getType() == XmlRpc::XmlRpcValue::TypeInt);

    // forward flow
    network_planner::Flow fwd_flow;
    fwd_flow.srcs.insert(static_cast<int>(source_nodes[i]));
    fwd_flow.dests.insert(static_cast<int>(destination_nodes[i]));
    fwd_flow.min_margin = margin;
    fwd_flow.confidence = confidence;
    comm_reqs.push_back(fwd_flow);

    // back flow
    network_planner::Flow bwd_flow;
    bwd_flow.srcs.insert(static_cast<int>(destination_nodes[i]));
    bwd_flow.dests.insert(static_cast<int>(source_nodes[i]));
    bwd_flow.min_margin = margin;
    bwd_flow.confidence = confidence;
    comm_reqs.push_back(bwd_flow);
  }

  ros::AsyncSpinner spinner(4);
  spinner.start();

  network_planner::NetworkPlanner np(nh, pnh);

  np.setCommReqs(comm_reqs);

  if (network_node_type.compare("routing") == 0) {
    NPN_INFO("running routing only node");
    np.runRoutingLoop();
  } else if (network_node_type.compare("planning") == 0) {
    NPN_INFO("running full planning node");
    np.runPlanningLoop();
  } else
    NPN_ERROR("invalid network_node_type %s", network_node_type.c_str());

  return 0;
}
