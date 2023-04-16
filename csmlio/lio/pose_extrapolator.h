/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CSMLIO_MAPPING_POSE_EXTRAPOLATOR_H_
#define CSMLIO_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "csmlio/common/time.h"
#include "csmlio/lio/utility.h"
#include "csmlio/lio/imu_tracker.h"
#include "csmlio/lio/pose_extrapolator_interface.h"
#include "csmlio/sensor/imu_data.h"
#include "csmlio/sensor/odometry_data.h"
#include "csmlio/transform/rigid_transform.h"

namespace csmlio {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
/**
 * @brief 维护一定时间的poses来估计线速度和角速度，并且用这些速度信息来外推运动估计
 * 如果有IMU和Odom信息，则可以用来提升估计精度
 */
class PoseExtrapolator : public PoseExtrapolatorInterface {
  public:
    explicit PoseExtrapolator(common::Duration pose_queue_duration,
                                double imu_gravity_time_constant);

    PoseExtrapolator(const PoseExtrapolator&) = delete;
    PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

    static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
        common::Duration pose_queue_duration, double imu_gravity_time_constant,
        const sensor::ImuData& imu_data);

    // Returns the time of the last added pose or Time::min() if no pose was added
    // yet.
    common::Time GetLastPoseTime() const override;
    common::Time GetLastExtrapolatedTime() const override;

    void AddPose(common::Time time, const transform::Rigid3d& pose) override;
    void AddImuData(const sensor::ImuData& imu_data) override;
    void AddOdometryData(const sensor::OdometryData& odometry_data) override;
    transform::Rigid3d ExtrapolatePose(common::Time time) override;

    ExtrapolationResult ExtrapolatePosesWithGravity(
        const std::vector<common::Time>& times) override;

    // Returns the current gravity alignment estimate as a rotation from
    // the tracking frame into a gravity aligned frame.
    Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

  private:
    void UpdateVelocitiesFromPoses();
    void TrimImuData();
    void TrimOdometryData();
    void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
    Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                            ImuTracker* imu_tracker) const;
    Eigen::Vector3d ExtrapolateTranslation(common::Time time);

    const common::Duration pose_queue_duration_;
    // struct TimedPose {
    //     common::Time time;
    //     transform::Rigid3d pose;
    // };
    std::deque<TimedPose> timed_pose_queue_;
    Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

    const double gravity_time_constant_;
    std::deque<sensor::ImuData> imu_data_;
    std::unique_ptr<ImuTracker> imu_tracker_;
    // 每次PoseExtrapolator::AddPose完之后，odometry_imu_tracker_, extrapolation_imu_tracker_是一个新的ImuTracker
    std::unique_ptr<ImuTracker> odometry_imu_tracker_;      ///< 在PoseExtrapolator::AddOdometryData用到
    std::unique_ptr<ImuTracker> extrapolation_imu_tracker_; ///
    TimedPose cached_extrapolated_pose_;

    std::deque<sensor::OdometryData> odometry_data_;
    Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_POSE_EXTRAPOLATOR_H_
