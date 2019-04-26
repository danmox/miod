#include <channel_simulator/channel_simulator.h>
#include <octomap_msgs/conversions.h>
#include <string.h>


int main(int argc, char** argv)
{
  std::string file_name(argv[1]);

  octomap::OcTree* tree = new octomap::OcTree(0.05);
  bool success = tree->readBinary(file_name);
  if (!success) {
    ROS_INFO("failed to load octomap from file %s", file_name.c_str());
    delete tree;
    return -1;
  }

  octomap_msgs::Octomap msg;
  if (!octomap_msgs::binaryMapToMsg(*tree, msg)) {
    ROS_INFO("failed to create ROS msg from octree");
    delete tree;
    return -1;
  }
  octomap_msgs::Octomap::ConstPtr msg_ptr(new octomap_msgs::Octomap(msg));

  channel_simulator::ChannelSimulator sim;
  sim.mapCB(msg_ptr);

  geometry_msgs::Pose pose1, pose2;
  pose1.position.x = 3.0;
  pose1.position.y = 5.0;
  pose1.position.z = 0.1;
  pose2.position.x = 3.0;
  pose2.position.y = -5.0;
  pose2.position.z = 0.9;

  sim.predict(pose1, pose2);

  delete tree;
  return 0;
}
