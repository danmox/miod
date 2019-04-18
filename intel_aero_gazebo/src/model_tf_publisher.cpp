#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <nav_msgs/Odometry.h>

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
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        if(sc.call(model_state)) {
            //publish transform over tf
            geometry_msgs::TransformStamped tfs;
            tfs.header.stamp = model_state.response.header.stamp;
            tfs.header.frame_id = parent_frame;
            tfs.child_frame_id = child_frame;
            tfs.transform.translation.x = model_state.response.pose.position.x;
            tfs.transform.translation.y = model_state.response.pose.position.y;
            tfs.transform.translation.z = model_state.response.pose.position.z;
            tfs.transform.rotation = model_state.response.pose.orientation;

            bool valid = abs((tfs.transform.rotation.w * tfs.transform.rotation.w +
                              tfs.transform.rotation.x * tfs.transform.rotation.x +
                              tfs.transform.rotation.y * tfs.transform.rotation.y +
                              tfs.transform.rotation.z * tfs.transform.rotation.z) - 1.0f) < 10e-6;
            if (valid) {
              br.sendTransform(tfs);

              //publish odometry message over ROS
              nav_msgs::Odometry odom;
              odom.header.stamp = model_state.response.header.stamp;
              odom.header.frame_id = parent_frame;
              odom.child_frame_id = child_frame;
              odom.pose.pose.position.x = model_state.response.pose.position.x;
              odom.pose.pose.position.y = model_state.response.pose.position.y;
              odom.pose.pose.position.z = model_state.response.pose.position.z;
              odom.pose.pose.orientation = model_state.response.pose.orientation;
              odom.twist.twist = model_state.response.twist;

              odom_pub.publish(odom);
            } else {
              ROS_DEBUG("[model_tf_publisher] quaternion not normalized correctly");
            }
        } else {
            std::string srv_name = sc.getService();
            ROS_DEBUG("[model_tf_publisher] service call to %s failed", srv_name.c_str());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
