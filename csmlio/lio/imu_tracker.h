/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CSMLIO_MAPPING_IMU_TRACKER_H_
#define CSMLIO_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "csmlio/common/time.h"

namespace csmlio {
namespace mapping {

// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
/**
 * @brief 使用IMU的角速度和线加速度来保持跟踪姿态。
 * 假设缓慢运动下，线加速度是重力的直接观测，并且roll/pitch不会漂移
 *
 * TODO: 当前这个ImuTracker比较简单，可以优化 [epsilon.john]
 * 优点：进行了隔离，这个ImuTracker得到的姿态只与IMU数据有关，确保了连续性
 */
class ImuTracker {
 public:
  /**
   * @brief 构造函数，初始化成员变量
   * @param imu_gravity_time_constant 重力常数
   * @param time 第一帧imu时间
   */
  ImuTracker(double imu_gravity_time_constant, common::Time time);

  // Advances to the given 'time' and updates the orientation to reflect this.
  void Advance(common::Time time);

  // Updates from an IMU reading (in the IMU frame).
  void AddImuLinearAccelerationObservation(
      const Eigen::Vector3d& imu_linear_acceleration);
  void AddImuAngularVelocityObservation(
      const Eigen::Vector3d& imu_angular_velocity);

  // Query the current time.
  common::Time time() const { return time_; }

  // Query the current orientation estimate.
  Eigen::Quaterniond orientation() const { return orientation_; }

 private:
  const double imu_gravity_time_constant_;
  common::Time time_;
  common::Time last_linear_acceleration_time_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d gravity_vector_;      ///< 重力向量在载体坐标系下的表示
  Eigen::Vector3d imu_angular_velocity_;
};

}  // namespace mapping
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_IMU_TRACKER_H_
