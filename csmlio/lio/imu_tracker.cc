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

#include "csmlio/lio/imu_tracker.h"

#include <cmath>
#include <limits>

#include "csmlio/common/math.h"
#include "csmlio/lio/internal/eigen_quaterniond_from_two_vectors.h"
#include "csmlio/transform/transform.h"
#include "glog/logging.h"

namespace csmlio {
namespace mapping {

ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(Eigen::Vector3d::UnitZ()),
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

/**
 * @brief 利用imu角速度信息更新姿态估计，同时更新重力向量在载体坐标系下的表示
 * @param time
 */
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  // 计算delta_t，需要确保time_ < time
  const double delta_t = common::ToSeconds(time - time_);
  // 姿态更新
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  orientation_ = (orientation_ * rotation).normalized();
  // 重力向量在载体坐标系下的表示
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  time_ = time;
}

/**
 * @brief 1. 使用线加速度来表示重力向量在载体坐标系下的表示 2. 反求出当前载体的姿态
 * TODO: 可以换
 * @param imu_linear_acceleration
 */
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  // 构造ImuTracker时，last_linear_acceleration_time_的值=common::Time::min()
  // 第一次调用时，delta_t 会被赋值为std::numeric_limits<double>::infinity()
  // 后续调用，delta_t = time_ - last_linear_acceleration_time_
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.

  // wgh 以下，兼容Google的bag包，以及个人bag包。
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  if (gravity_vector_.z() > 0) {
    // wgh Google原始实现。
    rotation = FromTwoVectors(
        gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  }
  else {
    // wgh 添加负号，适配IMU的z轴方向。
    rotation = FromTwoVectors(
        -gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ()); 
  }
  orientation_ = (orientation_ * rotation).normalized();
  // CHECK_GT((orientation_ * gravity_vector_).z(), 0.); // wgh 注释掉。
  // CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

/**
 * @brief 将角速度信息保存到imu_angular_velocity_
 * @param imu_angular_velocity
 */
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace csmlio
