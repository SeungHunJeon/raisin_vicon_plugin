// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

//
// Created by mun on 24. 1. 22.
//

#ifndef RAISIN_PLUGIN_VICON_HPP
#define RAISIN_PLUGIN_VICON_HPP

#include "raisin_plugin/plugin.hpp"
#include "raisin_data_logger/raisin_data_logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace raisin
{
namespace plugin
{

struct Pose
{
  double timeStamp;
  Eigen::Vector3d position;
  Eigen::Matrix3d orientation_e;
  raisim::Mat<3,3> orientation;
};

class PoseBuffer
{
public:
  PoseBuffer(size_t maxSize = 100)
  : maxSize_(maxSize)
  {
    // Fill the buffer with default poses
    for (size_t i = 0; i < maxSize_; ++i) {
      buffer.push_back(getDefaultPose());
    }
  }

  // Move constructor
  PoseBuffer(PoseBuffer && other) noexcept
  : maxSize_(other.maxSize_),
    buffer(std::move(other.buffer)) {}

  // Move assignment operator
  PoseBuffer & operator=(PoseBuffer && other) noexcept
  {
    if (this != &other) {
      std::lock_guard<std::mutex> guard(mtx_);
      maxSize_ = other.maxSize_;
      buffer = std::move(other.buffer);
    }
    return *this;
  }

  void addPose(double timeStamp, Eigen::Vector3d pos, Eigen::Matrix3d ori_e, raisim::Mat<3,3> ori)
  {

    std::lock_guard<std::mutex> guard(mtx_);
    if (buffer.size() >= maxSize_) {
      buffer.pop_front();        // Remove the oldest pose if buffer is full
    }
    buffer.push_back({timeStamp, pos, ori_e, ori});
  }

  Pose getClosestPose(double timeStamp)
  {
    Pose closestPose;
    double minDiff = std::numeric_limits<double>::max();

    std::lock_guard<std::mutex> guard(mtx_);
    for (auto rit = buffer.rbegin(); rit != buffer.rend(); ++rit) {
      double diff = std::abs(rit->timeStamp - timeStamp);
      if (diff < minDiff) {
        minDiff = diff;
        closestPose = *rit;
      } else {
        break;
      }
    }
    return closestPose;
  }

  bool empty()
  {
    std::lock_guard<std::mutex> guard(mtx_);
    return buffer.empty();
  }

  Pose getLastPose()
  {
    std::lock_guard<std::mutex> guard(mtx_);
    if (buffer.empty()) {
      throw std::runtime_error("PoseBuffer is empty");
    }
    return buffer.back();
  }

private:
  size_t maxSize_;
  std::deque<Pose> buffer;
  std::mutex mtx_;

  Pose getDefaultPose() const
  {
    Pose defaultPose;
    raisim::Mat<3,3> rot{1, 0, 0, 0, 1, 0, 0, 0, 1};
    defaultPose.timeStamp = 0.0;
    defaultPose.position = Eigen::Vector3d(0.0, 0.0, 0.0);
    defaultPose.orientation_e = Eigen::Matrix3d::Identity();
    defaultPose.orientation = rot;
    return defaultPose;
  }
};

class Vicon : public rclcpp::Node, public Plugin
{
public:
  Vicon(
    raisim::World & world, raisim::RaisimServer & server,
    raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource);
  bool advance() final;

  void createSubscriber();

  bool init() final;

  bool reset() final;

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /// Initialization for initial pose
  // 초기 pose 데이터 정렬 - 첫 번째 데이터로 offset 계산
  std::map<std::string, Eigen::Vector3d> initial_positions_;
  std::map<std::string, raisim::Mat<3, 3>> initial_orientations_;
  std::map<std::string, bool> is_initialized_;

  /// Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSubscription_;

  /// Articulated
  raisim::ArticulatedSystem * robotHub_;
  raisim::ArticulatedSystem * robotVicon_;
  parameter::ParameterContainer & param_;

  /// Vicon Data
  std::map<std::string, PoseBuffer> poseBuffers_;

  /// Vicon Offset
  std::map<std::string, Eigen::Vector3d> offsets_;

  /// param
  bool use_object_;
  raisim::SingleBodyObject * objectVicon_;
};
}     // plugin
} // raisin

#endif //RAISIN_PLUGIN_VICON_HPP
