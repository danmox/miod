#include <network_planner/network_planner.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "network_planner");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  network_planner::NetworkPlanner np(nh, pnh);

  // initialize task spec
  network_planner::Flow flow;
  flow.sources.insert(1);
  flow.dests.insert(2);
  flow.qos = 0.2;
  flow.confidence = 0.8;
  network_planner::CommReqs comm_reqs;
  comm_reqs.push_back(flow);

  np.setCommReqs(comm_reqs);

  while (ros::ok() && !np.updateNetworkConfig()) {
    ros::Rate(2).sleep();
  }

  return 0;
}
