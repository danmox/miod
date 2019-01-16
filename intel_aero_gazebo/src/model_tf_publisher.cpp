#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>

int main(int argc, char **argv) {

	ros::init(argc, argv, "model_tf_publisher");
	ros::NodeHandle nh, pnh("~");

  std::string model_name, child_frame, parent_frame;
  if (!pnh.getParam("model_name", model_name) ||
      !pnh.getParam("child_frame", child_frame) ||
      !pnh.getParam("parent_frame", parent_frame)) {
    ROS_FATAL("[model_tf_publisher] failed to load parameters");
    exit(EXIT_FAILURE);
  }

	ros::ServiceClient sc;
  sc = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  gazebo_msgs::GetModelState model_state;
	model_state.request.model_name = model_name;

  tf2_ros::TransformBroadcaster br;

	ros::Rate loop_rate(30);
	while (ros::ok()) {

		if(sc.call(model_state)) {
      geometry_msgs::TransformStamped tfs;
      tfs.header.stamp = model_state.response.header.stamp;
      tfs.header.frame_id = parent_frame;
      tfs.child_frame_id = child_frame;
      tfs.transform.translation.x = model_state.response.pose.position.x;
      tfs.transform.translation.y = model_state.response.pose.position.y;
      tfs.transform.translation.z = model_state.response.pose.position.z;
      tfs.transform.rotation = model_state.response.pose.orientation;

      br.sendTransform(tfs);
    } else {
      std::string srv_name = sc.getService();
      ROS_DEBUG("[model_tf_publisher] service call to %s failed", srv_name.c_str());
    }

    ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
