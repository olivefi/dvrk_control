#include "dvrk_control/DVRKControl.hpp"

namespace dvrk_control {

DVRKControl::DVRKControl(any_node::Node::NodeHandlePtr nh)
    : any_node::Node(nh) {}

bool DVRKControl::init() {
  // Initialize coord transform
  dvrkCoordToNormalCoord_ << 0, 1, 0, -1, 0, 0, 0, 0, 1;
  normalCoordToDvrkCoord_ << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  // Load parameters
  publishRate_ = param<double>("publish_rate", 200);
  pGainPos_ = param<double>("p_gain_pos", 1);
  pGainOri_ = param<double>("p_gain_ori", 1);
  dGainPos_ = param<double>("d_gain_pos", 1);
  dGainOri_ = param<double>("d_gain_ori", 1);
  // Initialize subscribers
  dvrkPoseLeftSub_ = nh_->subscribe(
      "/MTML/measured_cp", 1, &DVRKControl::dvrkPoseLeftCallback, this);
  dvrkPoseRightSub_ = nh_->subscribe(
      "/MTMR/measured_cp", 1, &DVRKControl::dvrkPoseRightCallback, this);
  desPoseLeftSub_ = nh_->subscribe(
      "left/pose_des", 1, &DVRKControl::controlPoseLeftCallback, this);
  desPoseRightSub_ = nh_->subscribe(
      "right/pose_des", 1, &DVRKControl::controlPoseRightCallback, this);

  // Initialize publishers
  dvrkLeftWrenchPub_ =
      nh_->advertise<geometry_msgs::WrenchStamped>("/MTML/body/servo_cf", 1);
  dvrkRightWrenchPub_ =
      nh_->advertise<geometry_msgs::WrenchStamped>("/MTMR/body/servo_cf", 1);
  leftErrorPub_ = nh_->advertise<geometry_msgs::TransformStamped>("error/left", 1);
  rightErrorPub_ = nh_->advertise<geometry_msgs::TransformStamped>("error/right", 1);
  leftGravcompPub_ = nh_->advertise<std_msgs::Bool>("/MTML/enable_gravity_compensation", 1);
  rightGravcompPub_ = nh_->advertise<std_msgs::Bool>("/MTMR/enable_gravity_compensation", 1);
  currPoseLeftPub_ = nh_->advertise<geometry_msgs::TransformStamped>("left/pose_curr", 1);
  currPoseRightPub_ = nh_->advertise<geometry_msgs::TransformStamped>("right/pose_curr", 1);
  
  // Make arms go slack
  std_msgs::Bool enableGravComp;
  enableGravComp.data = true;
  leftGravcompPub_.publish(enableGravComp);
  rightGravcompPub_.publish(enableGravComp);
  geometry_msgs::WrenchStamped zeroWrench;
  zeroWrench.header.stamp = ros::Time::now();
  zeroWrench.wrench.force.x = 0;
  zeroWrench.wrench.force.y = 0;
  zeroWrench.wrench.force.z = 0;
  zeroWrench.wrench.torque.x = 0;
  zeroWrench.wrench.torque.y = 0;
  zeroWrench.wrench.torque.z = 0;
  dvrkLeftWrenchPub_.publish(zeroWrench);
  dvrkRightWrenchPub_.publish(zeroWrench);
  // Start updating loop
  any_worker::WorkerOptions workerOptions;
  workerOptions.name_ = ros::this_node::getName() + std::string{"_broadcast"};
  workerOptions.callback_ = boost::bind(&DVRKControl::update, this, _1);
  workerOptions.timeStep_ = 1.0 / publishRate_;
  workerOptions.defaultPriority_ = 0;

  if (!addWorker(workerOptions)) {
    ROS_ERROR_STREAM("Could not add worker: " << workerOptions.name_);
    return false;
  }

  return true;
}

void DVRKControl::cleanup() {}

bool DVRKControl::update(const any_worker::WorkerEvent &event) {
  // Update curr
  currPoseLeftPub_.publish(dvrkToNormal(dvrkPoseLeft_));
  currPoseRightPub_.publish(dvrkToNormal(dvrkPoseRight_));
  return true;
}

geometry_msgs::WrenchStamped DVRKControl::wrenchToDVRKFrame(
    const geometry_msgs::WrenchStamped &wrench,
    const geometry_msgs::TransformStamped &dvrkTrans) {
  geometry_msgs::WrenchStamped dvrkWrench;
  dvrkWrench.header = wrench.header;
  Eigen::Vector3d force, torque;
  force << dvrkWrench.wrench.force.x, dvrkWrench.wrench.force.y,
      dvrkWrench.wrench.force.z;
  torque << dvrkWrench.wrench.torque.x, dvrkWrench.wrench.torque.y,
      dvrkWrench.wrench.torque.z;
  Eigen::Quaterniond q;
  q.x() = dvrkTrans.transform.rotation.x;
  q.y() = dvrkTrans.transform.rotation.y;
  q.z() = dvrkTrans.transform.rotation.z;
  q.w() = dvrkTrans.transform.rotation.w;

  Eigen::Matrix3d frameCorrection = q.inverse().toRotationMatrix();
  force = frameCorrection * normalCoordToDvrkCoord_ * force;
  torque = frameCorrection * normalCoordToDvrkCoord_ * torque;
  dvrkWrench.wrench.force.x = force(0);
  dvrkWrench.wrench.force.y = force(1);
  dvrkWrench.wrench.force.z = force(2);
  dvrkWrench.wrench.torque.x = torque(0);
  dvrkWrench.wrench.torque.y = torque(1);
  dvrkWrench.wrench.torque.z = torque(2);
  return dvrkWrench;
}

geometry_msgs::TransformStamped DVRKControl::dvrkToNormal(
    const geometry_msgs::TransformStamped &dvrkTrans) {
  geometry_msgs::TransformStamped normalTrans = dvrkTrans;
  normalTrans.header = dvrkTrans.header;
  Eigen::Vector3d pos;
  pos << dvrkTrans.transform.translation.x, dvrkTrans.transform.translation.y,
      dvrkTrans.transform.translation.z;
  pos = dvrkCoordToNormalCoord_ * pos;
  Eigen::Vector3d ori;
  ori << dvrkTrans.transform.rotation.x, dvrkTrans.transform.rotation.y,
      dvrkTrans.transform.rotation.z;
  ori = dvrkCoordToNormalCoord_ * ori;
  normalTrans.transform.translation.x = pos(0);
  normalTrans.transform.translation.y = pos(1);
  normalTrans.transform.translation.z = pos(2);
  normalTrans.transform.rotation.x = ori(0);
  normalTrans.transform.rotation.y = ori(1);
  normalTrans.transform.rotation.z = ori(2);

  return normalTrans;
}

void DVRKControl::dvrkPoseLeftCallback(
    const geometry_msgs::TransformStamped::ConstPtr &msg) {
  dvrkPoseLeft_ = *msg;
}

void DVRKControl::dvrkPoseRightCallback(
    const geometry_msgs::TransformStamped::ConstPtr &msg) {
  dvrkPoseRight_ = *msg;
}

void DVRKControl::controlPoseLeftCallback(
    const geometry_msgs::TransformStamped::ConstPtr &msg) {
  desPoseLeft_ = *msg;
}

void DVRKControl::controlPoseRightCallback(
    const geometry_msgs::TransformStamped::ConstPtr &msg) {
  desPoseRight_ = *msg;
}

} /* namespace dvrk_control */
