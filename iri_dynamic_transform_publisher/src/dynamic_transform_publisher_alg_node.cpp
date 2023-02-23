#include "dynamic_transform_publisher_alg_node.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

DynamicTransformPublisherAlgNode::DynamicTransformPublisherAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<DynamicTransformPublisherAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2; //in [Hz]

  // [init publishers]

  // [init subscribers]

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

DynamicTransformPublisherAlgNode::~DynamicTransformPublisherAlgNode(void)
{
  // [free dynamic memory]
}

void DynamicTransformPublisherAlgNode::mainNodeThread(void)
{

  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]

  this->alg_.lock();
  tf2::Transform tf_head_optical, tf_link_optical, tf_head_link, tf_wrist_optical, tf_wrist_link;
  tf_head_optical.setOrigin(tf2::Vector3(-0.0124, -0.0823, 0.01692));
  // tf_head_optical.translation.x = -0.0124;
  // tf_head_optical.translation.y = -0.0823;
  // tf_head_optical.translation.z = 0.01692;
  tf2::Quaternion q;
  q.setRPY(0.0152272, 0.0283539, 0.0121376);
  tf_head_optical.setRotation(q);
  // tf_head_optical.rotation.setRPY(0.0152272, 0.0283539, 0.0121376);

  tf_link_optical.setOrigin(tf2::Vector3(-0.001, 0.015, 0.0));
  // tf_link_optical.translation.x= -0.001;
  // tf_link_optical.translation.y = 0.015;
  // tf_link_optical.translation.z = 0.0;
  q.setRPY(-1.575, 0.008, -1.564);
  // q.setRPY(-1.575, 0.008, -1.564);

  tf_link_optical.setRotation(q);
  // tf_link_optical.rotation.setRPY(-1.575, -0.008, -1.564);

  tf_wrist_optical.setOrigin(tf2::Vector3(0.008, -0.097, -0.098));
  // tf_wrist_optical.translation.x = 0.008;
  // tf_wrist_optical.translation.y = -0.097;
  // tf_wrist_optical.translation.z = -0.098;
  q.setRPY(3.124, -0.049, 3.138);
  tf_wrist_optical.setRotation(q);
  // tf_wrist_optical.rotation.setRPY(3.124, -0.049, 3.138);
  geometry_msgs::Transform p;

  tf_head_link = tf_head_optical*tf_link_optical.inverse();
  tf_wrist_link = tf_wrist_optical*tf_link_optical.inverse();

  // ROS_INFO_STREAM("TF HEAD CAMERA LINK");
  // p=tf2::toMsg(tf_head_link);
  // ROS_INFO_STREAM(p);
  // ROS_INFO_STREAM("TF WRIST CAMERA LINK");
  // p=tf2::toMsg(tf_wrist_link);
  // ROS_INFO_STREAM(p);



  if(this->config_.broadcast)
  {
    this->transform.header.stamp = ros::Time::now();
    this->tf_broadcaster.sendTransform(this->transform);
  }

  this->alg_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void DynamicTransformPublisherAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->transform.header.stamp    = ros::Time::now();
  this->transform.header.frame_id = config.parent_id;
  this->transform.child_frame_id  = config.frame_id;

  if(config.reset_translation)
  {
    config.reset_translation=false;
    config.x=0.0;
    config.y=0.0;
    config.z=0.0;
  }
  geometry_msgs::Transform t;
  t.translation.x = config.x;
  t.translation.y = config.y;
  t.translation.z = config.z;
  if(config.reset_rotation)
  {
    config.reset_rotation=false;
    config.roll =0.0;
    config.pitch=0.0;
    config.yaw  =0.0;
  }
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(config.roll,config.pitch,config.yaw);
  t.rotation = q;
  this->transform.transform  = t;

  this->setRate(1000.0/config.period);
  //this->loop_rate_ = 1000.0/config.period;

  if(config.show_rosrun)
  {
    config.show_rosrun=false;
    ROS_INFO("rosrun tf static_transform_publisher %f %f %f %f %f %f %s %s %f", config.x, config.y, config.z, config.yaw, config.pitch, config.roll, config.parent_id.c_str(), config.frame_id.c_str(), config.period);
  }
  this->config_=config;
  this->alg_.unlock();
}

void DynamicTransformPublisherAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<DynamicTransformPublisherAlgNode>(argc, argv, "dynamic_transform_publisher_alg_node");
}
