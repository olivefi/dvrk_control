#pragma once

#include <algorithm>
#include <Eigen/Core>
#include <ros/ros.h>


#include <any_node/any_node.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>

namespace dvrk_control {

class DVRKControl : public any_node::Node {
public:
  DVRKControl(any_node::Node::NodeHandlePtr nh);
  ~DVRKControl() override = default;

  bool init() override;
  void cleanup() override;
  bool update(const any_worker::WorkerEvent &event);

protected:
  // Methods
  geometry_msgs::WrenchStamped wrenchToDVRKFrame(const geometry_msgs::WrenchStamped &wrench, const geometry_msgs::TransformStamped &dvrkTrans);
  geometry_msgs::TransformStamped dvrkToNormal(const geometry_msgs::TransformStamped &dvrkTrans);

  // Coord frame transforms
  Eigen::Matrix3d dvrkCoordToNormalCoord_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d normalCoordToDvrkCoord_ = Eigen::Matrix3d::Identity();

  // Externally settable variables
  double publishRate_;
  double pGainPos_, pGainOri_;
  double dGainPos_, dGainOri_;

  // Internal variables
  geometry_msgs::TransformStamped dvrkPoseLeft_;
  geometry_msgs::TransformStamped dvrkPoseRight_;

  geometry_msgs::TransformStamped desPoseLeft_;
  geometry_msgs::TransformStamped desPoseRight_;

  // Callbacks
  void dvrkPoseLeftCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);
  void dvrkPoseRightCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);

  void controlPoseLeftCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);
  void controlPoseRightCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);


  // Publishers + Subscribers
  ros::Publisher leftGravcompPub_;
  ros::Publisher rightGravcompPub_;

  ros::Subscriber dvrkPoseLeftSub_;
  ros::Subscriber dvrkPoseRightSub_;

  ros::Subscriber desPoseLeftSub_;
  ros::Subscriber desPoseRightSub_;

  ros::Publisher currPoseLeftPub_;
  ros::Publisher currPoseRightPub_;

  ros::Publisher dvrkLeftWrenchPub_;
  ros::Publisher dvrkRightWrenchPub_;

  ros::Publisher leftErrorPub_;
  ros::Publisher rightErrorPub_;
};
} /* namespace dvrk_control */
