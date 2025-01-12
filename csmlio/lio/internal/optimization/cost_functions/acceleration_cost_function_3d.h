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

#ifndef CSMLIO_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ACCELERATION_COST_FUNCTION_3D_H_
#define CSMLIO_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ACCELERATION_COST_FUNCTION_3D_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"

namespace csmlio {
namespace mapping {

// Penalizes differences between IMU data and optimized accelerations.
class AccelerationCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor,
      const Eigen::Vector3d& delta_velocity_imu_frame,
      const double first_delta_time_seconds,
      const double second_delta_time_seconds) {
    return new ceres::AutoDiffCostFunction<
        AccelerationCostFunction3D, 3 /* residuals */,
        4 /* rotation variables */, 3 /* position variables */,
        3 /* position variables */, 3 /* position variables */,
        1 /* gravity variables */, 4 /* rotation variables */>(
        new AccelerationCostFunction3D(scaling_factor, delta_velocity_imu_frame,
                                       first_delta_time_seconds,
                                       second_delta_time_seconds));
  }

  template <typename T>
  bool operator()(const T* const middle_rotation, const T* const start_position,
                  const T* const middle_position, const T* const end_position,
                  const T* const gravity_constant,
                  const T* const imu_calibration, T* residual) const {
    const Eigen::Quaternion<T> eigen_imu_calibration(
        imu_calibration[0], imu_calibration[1], imu_calibration[2],
        imu_calibration[3]);
    const Eigen::Matrix<T, 3, 1> imu_delta_velocity =
        ToEigen(middle_rotation) * eigen_imu_calibration *
            delta_velocity_imu_frame_.cast<T>() -
        *gravity_constant *
            (0.5 * (first_delta_time_seconds_ + second_delta_time_seconds_) *
             Eigen::Vector3d::UnitZ())
                .cast<T>();
    const Eigen::Matrix<T, 3, 1> start_velocity =
        (Eigen::Map<const Eigen::Matrix<T, 3, 1>>(middle_position) -
         Eigen::Map<const Eigen::Matrix<T, 3, 1>>(start_position)) /
        T(first_delta_time_seconds_);
    const Eigen::Matrix<T, 3, 1> end_velocity =
        (Eigen::Map<const Eigen::Matrix<T, 3, 1>>(end_position) -
         Eigen::Map<const Eigen::Matrix<T, 3, 1>>(middle_position)) /
        T(second_delta_time_seconds_);
    const Eigen::Matrix<T, 3, 1> delta_velocity = end_velocity - start_velocity;

    (Eigen::Map<Eigen::Matrix<T, 3, 1>>(residual) =
         T(scaling_factor_) * (imu_delta_velocity - delta_velocity));
    return true;
  }

 private:
  AccelerationCostFunction3D(const double scaling_factor,
                             const Eigen::Vector3d& delta_velocity_imu_frame,
                             const double first_delta_time_seconds,
                             const double second_delta_time_seconds)
      : scaling_factor_(scaling_factor),
        delta_velocity_imu_frame_(delta_velocity_imu_frame),
        first_delta_time_seconds_(first_delta_time_seconds),
        second_delta_time_seconds_(second_delta_time_seconds) {}

  AccelerationCostFunction3D(const AccelerationCostFunction3D&) = delete;
  AccelerationCostFunction3D& operator=(const AccelerationCostFunction3D&) =
      delete;

  template <typename T>
  static Eigen::Quaternion<T> ToEigen(const T* const quaternion) {
    return Eigen::Quaternion<T>(quaternion[0], quaternion[1], quaternion[2],
                                quaternion[3]);
  }

  const double scaling_factor_;
  const Eigen::Vector3d delta_velocity_imu_frame_;
  const double first_delta_time_seconds_;
  const double second_delta_time_seconds_;
};

}  // namespace mapping
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ACCELERATION_COST_FUNCTION_3D_H_
