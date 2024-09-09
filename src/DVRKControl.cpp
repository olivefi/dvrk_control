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
  dvrkTwistLeftSub_ = nh_->subscribe(
      "/MTML/measured_cv", 1, &DVRKControl::dvrkTwistLeftCallback, this);
  dvrkTwistRightSub_ = nh_->subscribe(
      "/MTMR/measured_cv", 1, &DVRKControl::dvrkTwistRightCallback, this);
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

  geometry_msgs::TransformStamped zeroPose;
  zeroPose.header.stamp = ros::Time::now();
  zeroPose.transform.translation.x = -0.35;
  zeroPose.transform.translation.y = 0.0;
  zeroPose.transform.translation.z = -0.14;
  zeroPose.transform.rotation.x = 1.0;
  zeroPose.transform.rotation.y = 0.0;
  zeroPose.transform.rotation.z = 0.0;
  zeroPose.transform.rotation.w = 0.0;
  desPoseLeft_ = zeroPose;
  desPoseRight_ = zeroPose;

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
  currPoseLeftPub_.publish(dvrkPoseLeft_);
  currPoseRightPub_.publish(dvrkPoseRight_);

  geometry_msgs::WrenchStamped leftCmd = createImpdCmd(dvrkPoseLeft_, desPoseLeft_, dvrkTwistLeft_, geometry_msgs::TwistStamped());
  geometry_msgs::WrenchStamped rightCmd = createImpdCmd(dvrkPoseRight_, desPoseRight_, dvrkTwistRight_, geometry_msgs::TwistStamped());
  dvrkLeftWrenchPub_.publish(wrenchToDVRKFrame(leftCmd, dvrkPoseLeft_));
  // dvrkRightWrenchPub_.publish(wrenchToDVRKFrame(rightCmd, dvrkPoseRight_));

  return true;
}

geometry_msgs::WrenchStamped DVRKControl::createImpdCmd(const geometry_msgs::TransformStamped &currPose, const geometry_msgs::TransformStamped &desPose,
                                           const geometry_msgs::TwistStamped &currTwist, const geometry_msgs::TwistStamped &desTwist) {
  geometry_msgs::WrenchStamped cmd;
  Eigen::Vector3d posErr, velErr;
  posErr << desPose.transform.translation.x - currPose.transform.translation.x,
      desPose.transform.translation.y - currPose.transform.translation.y,
      desPose.transform.translation.z - currPose.transform.translation.z;
  velErr << desTwist.twist.linear.x - currTwist.twist.linear.x,
      desTwist.twist.linear.y - currTwist.twist.linear.y,
      desTwist.twist.linear.z - currTwist.twist.linear.z;
  Eigen::Vector3d posCmd  = pGainPos_ * posErr + dGainPos_ * velErr;
  cmd.header.stamp = ros::Time::now();
  Eigen::Quaterniond oriCurr, oriDes;
  oriCurr.x() = currPose.transform.rotation.x;
  oriCurr.y() = currPose.transform.rotation.y;
  oriCurr.z() = currPose.transform.rotation.z;
  oriCurr.w() = currPose.transform.rotation.w;
  oriDes.x() = desPose.transform.rotation.x;
  oriDes.y() = desPose.transform.rotation.y;
  oriDes.z() = desPose.transform.rotation.z;
  oriDes.w() = desPose.transform.rotation.w;
  Eigen::AngleAxisd oriErr(oriCurr.toRotationMatrix().transpose() * oriDes.toRotationMatrix());
  Eigen::Vector3d angvelErr;
  angvelErr << desTwist.twist.angular.x - currTwist.twist.angular.x,
      desTwist.twist.angular.y - currTwist.twist.angular.y,
      desTwist.twist.angular.z - currTwist.twist.angular.z;
  Eigen::Vector3d angvelCmd;
  angvelCmd = pGainOri_ * oriErr.angle() * oriCurr.toRotationMatrix() * oriErr.axis() + dGainOri_ * angvelErr;
  cmd.wrench.force.x = posCmd(0);
  cmd.wrench.force.y = posCmd(1);
  cmd.wrench.force.z = posCmd(2);
  // cmd.wrench.torque.x = angvelCmd(0);
  // cmd.wrench.torque.y = angvelCmd(1);
  // cmd.wrench.torque.z = angvelCmd(2);
  return cmd;
}

geometry_msgs::WrenchStamped DVRKControl::wrenchToDVRKFrame(
    const geometry_msgs::WrenchStamped &wrench,
    const geometry_msgs::TransformStamped &dvrkTrans) {
  geometry_msgs::WrenchStamped dvrkWrench = wrench;
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

geometry_msgs::TransformStamped DVRKControl::dvrkPoseToNormal(
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

geometry_msgs::TwistStamped DVRKControl::dvrkTwistToNormal(const geometry_msgs::TwistStamped& dvrkTwist){
  geometry_msgs::TwistStamped normalTwist = dvrkTwist;
  normalTwist.header = dvrkTwist.header;
  Eigen::Vector3d linVel;
  linVel << dvrkTwist.twist.linear.x, dvrkTwist.twist.linear.y, dvrkTwist.twist.linear.z;
  linVel = dvrkCoordToNormalCoord_ * linVel;
  Eigen::Vector3d angVel;
  angVel << dvrkTwist.twist.angular.x, dvrkTwist.twist.angular.y, dvrkTwist.twist.angular.z;
  angVel = dvrkCoordToNormalCoord_ * angVel;
  normalTwist.twist.linear.x = linVel(0);
  normalTwist.twist.linear.y = linVel(1);
  normalTwist.twist.linear.z = linVel(2);
  normalTwist.twist.angular.x = angVel(0);
  normalTwist.twist.angular.y = angVel(1);
  normalTwist.twist.angular.z = angVel(2);
  return normalTwist;
}

void DVRKControl::dvrkPoseLeftCallback(
    const geometry_msgs::TransformStamped::ConstPtr &msg) {
  geometry_msgs::TransformStamped dvrkPoseLeft = *msg;
  dvrkPoseLeft_ = dvrkPoseToNormal(dvrkPoseLeft);
}

void DVRKControl::dvrkPoseRightCallback(
    const geometry_msgs::TransformStamped::ConstPtr &msg) {
  geometry_msgs::TransformStamped dvrkPoseRight = *msg;
  dvrkPoseRight_ = dvrkPoseToNormal(dvrkPoseRight);
}

void DVRKControl::dvrkTwistLeftCallback(
    const geometry_msgs::TwistStamped::ConstPtr &msg) {
  geometry_msgs::TwistStamped dvrkTwistLeft = *msg;
  dvrkTwistLeft_ = dvrkTwistToNormal(dvrkTwistLeft);
}

void DVRKControl::dvrkTwistRightCallback(
    const geometry_msgs::TwistStamped::ConstPtr &msg) {
  geometry_msgs::TwistStamped dvrkTwistRight = *msg;
  dvrkTwistRight_ = dvrkTwistToNormal(dvrkTwistRight);
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
