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
: Plugin(world, server, worldSim, serverSim, globalResource),
  param_(parameter::ParameterContainer::getRoot()["raisin_vicon_plugin"]) 
{
  pluginType_ = PluginType::CUSTOM;

  param_.loadFromPackageParameterFile("raisin_vicon_plugin");

  host_address_ = std::string(param_("host_address"));
  buffer_size_ = param_("buffer_size");
  use_object_ = param_("use_object");

  robotHub_ = reinterpret_cast<raisim::ArticulatedSystem *>(
    worldHub_.getObject("robot")
  ); /// robot

  robotVicon_ = reinterpret_cast<raisim::ArticulatedSystem *>(
    worldHub_.getObject("robot_vicon")
  ); /// robot vicon

  poseBuffers_.emplace("robot", PoseBuffer(100));

  if(use_object_) {
    objectVicon_ = reinterpret_cast<raisim::SingleBodyObject *>(
      worldHub_.getObject("object_vicon")
    );
    poseBuffers_.emplace("object", PoseBuffer(100));

    logIdx_ = dataLogger_.initializeAnotherDataGroup(
    "Vicon",
    "RobotPosition", poseBuffers_["robot"].getLastPose().position,
    "RobotOrientation", poseBuffers_["robot"].getLastPose().orientation,
    "ObjectPosition", poseBuffers_["object"].getLastPose().position,
    "ObjectOrientation", poseBuffers_["roboobjectt"].getLastPose().orientation);
  }

  else {
    logIdx_ = dataLogger_.initializeAnotherDataGroup(
    "Vicon",
    "RobotPosition", poseBuffers_["robot"].getLastPose().position,
    "RobotOrientation", poseBuffers_["robot"].getLastPose().orientation);
  }

  
  
}
bool Vicon::advance()
{
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

  if(use_object_) {
    pose = poseBuffers_["object"].getLastPose();
    raisim::rotMatToQuat(pose.orientation, quat);
    serverHub_.lockVisualizationServerMutex();
    objectVicon_->setPosition(pose.position);
    objectVicon_->setOrientation(pose.orientation);
    serverHub_.unlockVisualizationServerMutex();

    dataLogger_.append(
    logIdx_,
    poseBuffers_["robot"].getLastPose().position,
    poseBuffers_["robot"].getLastPose().orientation,
    poseBuffers_["object"].getLastPose().position,
    poseBuffers_["object"].getLastPose().orientation);
  }

  else {
    dataLogger_.append(
    logIdx_,
    poseBuffers_["robot"].getLastPose().position,
    poseBuffers_["robot"].getLastPose().orientation);
  }

  return true;
}

bool Vicon::init()
{
  initVicon();

  initData();

  return true;
}

bool Vicon::initVicon()
{
  while (!client_.IsConnected().Connected) {
    RSINFO("Connecting")
    client_.Connect(host_address_);
  }

  client_.EnableSegmentData();
  client_.EnableMarkerData();
  RSFATAL_IF(client_.EnableMarkerData().Result != Result::Success, "Enable Marker Data Failed")
  RSFATAL_IF(client_.EnableSegmentData().Result != Result::Success, "Enable Segment Data Failed")
  client_.SetStreamMode(StreamMode::ServerPush);
  client_.SetBufferSize(buffer_size_);

  return true;
}

bool Vicon::initData()
{
  return true;
}

void Vicon::viconUpdate()
{
  auto Output = client_.GetFrame();
  if(Output.Result == Result::Success)
  {
    Output_GetFrameNumber frame_number = client_.GetFrameNumber();
    unsigned int subject_num = client_.GetSubjectCount().SubjectCount;
    for (unsigned int subject_idx = 0; subject_idx < subject_num; subject_idx++) {
      std::string subject_name = client_.GetSubjectName(subject_idx).SubjectName;
      unsigned int segment_num = client_.GetSegmentCount(subject_name).SegmentCount;
      std::string segment_name = client_.GetSegmentName(subject_name, 0).SegmentName;
      Output_GetTimecode outputTimeCode = client_.GetTimecode();
      Output_GetFrameRate outputFrameRate = client_.GetFrameRate();
      Output_GetSegmentGlobalTranslation trans =
          client_.GetSegmentGlobalTranslation(subject_name, segment_name);
      Output_GetSegmentGlobalRotationMatrix rot =
          client_.GetSegmentGlobalRotationMatrix(subject_name, segment_name);
      Output_GetSegmentGlobalRotationQuaternion quat =
          client_.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);

      double timeStamp = outputTimeCode.Hours * 1e6 + outputTimeCode.Minutes * 1e3 + outputTimeCode.Seconds;
      Eigen::Vector3d pos_e;
      Eigen::Matrix3d rot_e;
      
      for (size_t i = 0; i < 3; i++) {
        pos_e(i) = trans.Translation[i] * 1e-3;
      }

      for (size_t i = 0; i < 9; i++) {
        rot_e(i) = rot.Rotation[i];
      }
      poseBuffers_[subject_name].addPose(timeStamp, pos_e, rot_e);
    }
  }
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
