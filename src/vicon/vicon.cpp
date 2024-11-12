// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

//
// Created by mun on 24. 1. 22.
//

#include "vicon/vicon.hpp"

namespace raisin
{
namespace plugin
{
Vicon::Vicon(
  raisim::World & world, raisim::RaisimServer & server,
  raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource)
: rclcpp::Node("RaisinViconPlugin"), Plugin(world, server, worldSim, serverSim, globalResource),
  param_(parameter::ParameterContainer::getRoot()["raisin_vicon_plugin"])
{
  pluginType_ = PluginType::CUSTOM;

  param_.loadFromPackageParameterFile("raisin_vicon_plugin");

  robotHub_ = reinterpret_cast<raisim::ArticulatedSystem *>(
    worldHub_.getObject("robot")
  ); /// robot

  robotVicon_ = reinterpret_cast<raisim::ArticulatedSystem *>(
    worldHub_.getObject("robot_vicon")
  ); /// robot vicon

}
bool Vicon::advance()
{
  rclcpp::spin_some(this->get_node_base_interface());

  Eigen::VectorXd gc;
  Eigen::VectorXd gv;
  raisim::Vec<4> quat;

  robotHub_->lockMutex();
  robotHub_->getState(gc, gv);
  robotHub_->unlockMutex();

  auto pose = poseBuffers_["robot"].getLastPose();

  raisim::rotMatToQuat(pose.orientation, quat);

  gc.head(3) = pose.position;
  gc.segment(3, 4) = quat.e();

  serverHub_.lockVisualizationServerMutex();
  robotVicon_->setState(gc, gv);
  serverHub_.unlockVisualizationServerMutex();

  if (use_object_) {
    pose = poseBuffers_["object"].getLastPose();
    raisim::rotMatToQuat(pose.orientation, quat);
    serverHub_.lockVisualizationServerMutex();
    objectVicon_->setPosition(pose.position);
    objectVicon_->setOrientation(pose.orientation_e);
    serverHub_.unlockVisualizationServerMutex();

    dataLogger_.append(
      logIdx_,
      poseBuffers_["robot"].getLastPose().position,
      poseBuffers_["robot"].getLastPose().orientation_e,
      poseBuffers_["object"].getLastPose().position,
      poseBuffers_["object"].getLastPose().orientation_e);
  } else {
    dataLogger_.append(
      logIdx_,
      poseBuffers_["robot"].getLastPose().position,
      poseBuffers_["robot"].getLastPose().orientation_e);
  }

  return true;
}

bool Vicon::init()
{
  use_object_ = bool(param_("use_object"));

  poseBuffers_.emplace("robot", PoseBuffer(100));

  /// Offset
  double robot_offset_z = double(param_["offset"]["robot"]("z"));
  Eigen::Vector3d robot_offset{0, 0, robot_offset_z};
  offsets_.emplace("robot", robot_offset);

  if (use_object_) {
    objectVicon_ = reinterpret_cast<raisim::SingleBodyObject *>(
      worldHub_.getObject("object")
    );
    poseBuffers_.emplace("object", PoseBuffer(100));

    double object_offset_z = -(reinterpret_cast<raisim::Box*>(objectVicon_)->getDim().e()(2) / 2);
    Eigen::Vector3d object_offset{0, 0, object_offset_z};
    offsets_.emplace("object", object_offset);

    logIdx_ = dataLogger_.initializeAnotherDataGroup(
      "Vicon",
      "RobotPosition", poseBuffers_["robot"].getLastPose().position,
      "RobotOrientation", poseBuffers_["robot"].getLastPose().orientation_e,
      "ObjectPosition", poseBuffers_["object"].getLastPose().position,
      "ObjectOrientation", poseBuffers_["object"].getLastPose().orientation_e);
  } else {
    logIdx_ = dataLogger_.initializeAnotherDataGroup(
      "Vicon",
      "RobotPosition", poseBuffers_["robot"].getLastPose().position,
      "RobotOrientation", poseBuffers_["robot"].getLastPose().orientation_e);
  }

  this->createSubscriber();

  return true;
}

bool Vicon::reset()
{
  is_initialized_.clear();

  return true;
}

void Vicon::createSubscriber()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  poseSubscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "vicon_pose", 10, std::bind(&Vicon::poseCallback, this, std::placeholders::_1));
}

void Vicon::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  Eigen::Vector3d position;
  raisim::Mat<3, 3> orientation;
  raisim::Vec<4> quat;

  std::string frame_id = msg->header.frame_id;

  position(0) = msg->pose.position.x * 1e-3;
  position(1) = msg->pose.position.y * 1e-3;
  position(2) = msg->pose.position.z * 1e-3;
  quat[0] = msg->pose.orientation.w;
  quat[1] = msg->pose.orientation.x;
  quat[2] = msg->pose.orientation.y;
  quat[3] = msg->pose.orientation.z;

  raisim::quatToRotMat(quat, orientation);

//  // Initialize position and orientation if not yet initialized for this frame_id
//  if (is_initialized_.find(frame_id) == is_initialized_.end() || !is_initialized_[frame_id]) {
//    initial_positions_[frame_id] = position;
//    initial_orientations_[frame_id] = orientation;
//    is_initialized_[frame_id] = true;
//  }
//
//  // Check if both "robot" and "object" are initialized, otherwise return
//  if (!(is_initialized_["robot"] && is_initialized_["object"])) {
//    return;
//  }
//
//  Eigen::Vector3d nominal_position;
//  // 목표 정렬 위치와 방향 정의
//  nominal_position << 0.0, 0.0, initial_positions_[frame_id](2);
//
//  // 현재 pose에서 초기 offset을 제거하고, 정렬 값에 맞게 변환
//  orientation.e() = initial_orientations_["robot"].e().transpose() * orientation.e();
//  position = initial_orientations_["robot"].e().transpose() * (position - initial_positions_["robot"]);

//
//  position = nominal_position + nominal_orientation.e() * (position - initial_positions_[frame_id]);
//  orientation.e() = nominal_orientation.e() * initial_orientations_[frame_id].e().transpose() * orientation.e();


  /// Offset
  position += orientation.e() * offsets_[frame_id];
  poseBuffers_[frame_id].addPose(0, position, orientation.e(), orientation);
}

extern "C" Plugin * create(
  raisim::World & world, raisim::RaisimServer & server,
  raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource)
{
  return new Vicon(world, server, worldSim, serverSim, globalResource);
}

extern "C" void destroy(Plugin * p)
{
  delete p;
}

}     // plugin
} // raisin
