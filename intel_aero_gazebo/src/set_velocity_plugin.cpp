#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <geometry_msgs/Twist.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <string>
#include <thread>

namespace intel_aero_gazebo
{

class SetModelVelocity : public gazebo::ModelPlugin
{
  protected:
    std::unique_ptr<ros::NodeHandle> ros_nh;
    ros::Subscriber ros_sub;
    ros::CallbackQueue ros_queue;
    std::thread queue_thread;

    std::string link_name;
    gazebo::math::Vector3 linear_vel, angular_vel;

    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr updateConnection;

    void QueueThread();

  public:
    virtual void Load(gazebo::physics::ModelPtr _model,
                      sdf::ElementPtr _sdf);
    void Update(const gazebo::common::UpdateInfo &_info);
    void velCB(const geometry_msgs::Twist::ConstPtr&);
};

void SetModelVelocity::QueueThread()
{
  while (ros_nh->ok()) {
    ros_queue.callAvailable(ros::WallDuration(0.01));
  }
}

void SetModelVelocity::velCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  linear_vel.Set(msg->linear.x, msg->linear.y, msg->linear.z);
  angular_vel.Set(msg->angular.x, msg->angular.y, msg->angular.z);
}

void SetModelVelocity::Load(gazebo::physics::ModelPtr _model,
                            sdf::ElementPtr _sdf)
{
  model = _model;
  updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&SetModelVelocity::Update, this, std::placeholders::_1));

  // fetch link_name from plugin parameters
  if (_sdf->HasElement("link_name")) {
    link_name = _sdf->Get<std::string>("link_name");
    if (model->GetLink(link_name)) {
      ROS_INFO("[SetModelVelocity] setting velocity of link %s", link_name.c_str());
    } else {
      ROS_ERROR("[SetModelVelocity] %s does not name an element in the model", link_name.c_str());
    }
  } else {
    ROS_ERROR("[SetModelVelocity] SDF missing link_name parameter");
  }

  if (!ros::isInitialized()) {
    int ac = 0;
    char** av = NULL;
    ros::init(ac, av, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  ros_nh.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
      "/" + model->GetName() + "/vel_cmd",
      1,
      std::bind(&SetModelVelocity::velCB, this, std::placeholders::_1),
      ros::VoidPtr(), &ros_queue);
  ros_sub = ros_nh->subscribe(so);

  // Spin up the queue helper thread.
  queue_thread = std::thread(std::bind(&SetModelVelocity::QueueThread, this));
  ROS_INFO("[SetModelVelocity] plugin successfully loaded");
}

void SetModelVelocity::Update(const gazebo::common::UpdateInfo &_info)
{
  auto link = model->GetLink(link_name);
  if (link) {
    link->SetLinearVel(linear_vel);
    link->SetAngularVel(angular_vel);
  } else {
    ROS_DEBUG("[SetAngularVel] model %s not found", link_name.c_str());
  }
}

GZ_REGISTER_MODEL_PLUGIN(SetModelVelocity)
}
