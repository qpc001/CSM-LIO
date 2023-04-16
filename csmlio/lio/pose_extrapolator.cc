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

#include "csmlio/lio/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
#include "csmlio/transform/transform.h"
#include "glog/logging.h"

namespace csmlio {
namespace mapping {

PoseExtrapolator::PoseExtrapolator(
    const common::Duration pose_queue_duration,
    double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration)
    , gravity_time_constant_(imu_gravity_time_constant)
    , cached_extrapolated_pose_{common::Time::min(), 
        transform::Rigid3d::Identity()} 
{
    LOG(INFO) << " ** Pose extrapolator constructed with: ";
    LOG(INFO) << " ** pose_queue_duration = " << common::ToSeconds(pose_queue_duration);
    LOG(INFO) << " ** gravity_time_constant = " << imu_gravity_time_constant;
}

/**
 * @brief 静态方法，
 * @param pose_queue_duration
 * @param imu_gravity_time_constant
 * @param imu_data
 * @return
 */
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, 
    const sensor::ImuData& imu_data) 
{
    // 构造PoseExtrapolator
    auto extrapolator = absl::make_unique<PoseExtrapolator>(
        pose_queue_duration, imu_gravity_time_constant);
    // 添加最新imu数据
    extrapolator->AddImuData(imu_data);
    // 创建PoseExtrapolator内部成员变量imu_tracker_
    extrapolator->imu_tracker_ =
        absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
    // 向extrapolator->imu_tracker_添加线加速度
    extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
        imu_data.linear_acceleration);
    // 向extrapolator->imu_tracker_添加角速度
    extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
        imu_data.angular_velocity);
    // 调用extrapolator->imu_tracker_进行姿态更新
    extrapolator->imu_tracker_->Advance(imu_data.time);
    // 取extrapolator->imu_tracker_更新后的姿态，添加pose到
    extrapolator->AddPose(
        imu_data.time,
        transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
    return extrapolator;
}

common::Time PoseExtrapolator::GetLastPoseTime() const 
{
    if (timed_pose_queue_.empty()) {
        return common::Time::min();
    }
    return timed_pose_queue_.back().time;
}

common::Time PoseExtrapolator::GetLastExtrapolatedTime() const 
{
    if (!extrapolation_imu_tracker_) {
        return common::Time::min();
    }
    return extrapolation_imu_tracker_->time();
}

/**
 * @brief 主要有两个调用方式：
 * 1. PoseExtrapolator::InitializeWithImu内部调用此函数
 * 2. CSMLidarInertialOdometry::AddAccumulatedRangeData激光匹配后调用此函数
 * @param time
 * @param pose
 */
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) 
{
    if (imu_tracker_ == nullptr) {          // 这个情况，可能是先进行了激光匹配，imu还没有
        common::Time tracker_start = time;
        if (!imu_data_.empty()) {
        tracker_start = std::min(tracker_start, imu_data_.front().time);
        }
        imu_tracker_ =
            absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
    }
    // 保存pose到timed_pose_queue_
    timed_pose_queue_.push_back(TimedPose{time, pose});
    // 如果timed_pose_queue_队列有3个或以上的pose，并且队列中最早的数据已经超过了时间限制，则弹出
    while (timed_pose_queue_.size() > 2 &&
            timed_pose_queue_[1].time <= time - pose_queue_duration_) {
        timed_pose_queue_.pop_front();
    }
    // 利用timed_pose_queue_队列中最旧的数据和最新的数据来更新线速度和角速度
    UpdateVelocitiesFromPoses();
    // 利用imu_data_队列中的imu数据，将imu_tracker向前推算到指定时间time
    // 也就是说，imu_tracker_的推算是最慢的，当有新的激光匹配结果时，才对imu_tracker_进行推算，推算到新的激光匹配结果对应的时间戳
    AdvanceImuTracker(time, imu_tracker_.get());
    // 弹出已经没有用的imu/odom数据
    TrimImuData();
    TrimOdometryData();
    // 重新构造odometry_imu_tracker_ 和 extrapolation_imu_tracker_
    //每次PoseExtrapolator::AddPose完之后，odometry_imu_tracker_/extrapolation_imu_tracker_是一个新的ImuTracker
    odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);   // 在AddOdometryData用到，作用是基于imu_tracker_再向前推算到最新odom时刻的姿态
    extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);  //
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) 
{
    // 确保新加进来的imu时间戳大于队列最后一个pose的时间戳
    CHECK(timed_pose_queue_.empty() ||
            imu_data.time >= timed_pose_queue_.back().time);
    // 储存imu数据
    imu_data_.push_back(imu_data);
    // 清理已经使用过的imu数据
    TrimImuData();
}

/**
 * @brief 添加odom数据：
 * 1. 储存数据到odometry_data_
 * 2. 取odometry_data_队列最旧和最新的两个odom数据，计算角速度/线速度
 *    （其中，计算线速度时，需要用到 odometry_imu_tracker_和timed_pose_queue_）
 * @param odometry_data
 */
void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) 
{
    // 确保新加进来的odom时间戳大于队列最后一个pose的时间戳
    CHECK(timed_pose_queue_.empty() ||
            odometry_data.time >= timed_pose_queue_.back().time);
    // 储存odom数据
    odometry_data_.push_back(odometry_data);
    // 清理已经使用过的odom数据
    TrimOdometryData();
    if (odometry_data_.size() < 2) {
        return;
    }
    // TODO(whess): Improve by using more than just the last two odometry poses.
    // Compute extrapolation in the tracking frame.
    // 取odometry_data_队列最旧和最新的两个odom数据
    const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
    const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
    const double odometry_time_delta =
        common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
    // 计算相对位姿变换（旧位姿到新位姿的变换）
    const transform::Rigid3d odometry_pose_delta =
        odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
    // 计算角速度，保存到angular_velocity_from_odometry_
    angular_velocity_from_odometry_ =
        transform::RotationQuaternionToAngleAxisVector(
            odometry_pose_delta.rotation()) /
        odometry_time_delta;
    // 如果timed_pose_queue_队列为空，则无法计算线速度
    if (timed_pose_queue_.empty()) {
        return;
    }

    // 两个odom数据/时间计算出来的只是在odom坐标系的线速度，因此需要进行坐标系转换
    const Eigen::Vector3d
        linear_velocity_in_tracking_frame_at_newest_odometry_time =
            odometry_pose_delta.translation() / odometry_time_delta;
    // ExtrapolateRotation()：
    // 1. 利用imu_data_队列中的imu数据，将odometry_imu_tracker_向前推算到最新odom时刻
    // 2. 计算得到两个ImuTracker的旋转变换: 新的odometry_imu_tracker_ 到 旧imu_tracker_的姿态变换（可以说是到timed_pose_queue_.back().pose的姿态变换）
    // 注意：timed_pose_queue_.back().pose 与 旧imu_tracker_的姿态 挂钩
    const Eigen::Quaterniond orientation_at_newest_odometry_time =
        timed_pose_queue_.back().pose.rotation() *
        ExtrapolateRotation(odometry_data_newest.time,
                            odometry_imu_tracker_.get());
    // orientation_at_newest_odometry_time: 最新odom时刻下，pose到地图坐标系的旋转
    // linear_velocity_in_tracking_frame_at_newest_odometry_time: 旧位姿到新位姿(最新odom)的变换 (旧位姿在新位姿坐标系下的表示)
    // 所以，linear_velocity_from_odometry_: 在地图坐标系下的速度表示
    linear_velocity_from_odometry_ =
        orientation_at_newest_odometry_time *
        linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) 
{
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    CHECK_GE(time, newest_timed_pose.time);
    if (cached_extrapolated_pose_.time != time) {
        const Eigen::Vector3d translation =
            ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
        const Eigen::Quaterniond rotation =
            newest_timed_pose.pose.rotation() *
            ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
        cached_extrapolated_pose_ =
            TimedPose{time, transform::Rigid3d{translation, rotation}};
    }
    return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) 
{
    ImuTracker imu_tracker = *imu_tracker_;
    AdvanceImuTracker(time, &imu_tracker);
    return imu_tracker.orientation();
}

/**
 * @brief 利用timed_pose_queue_队列中最旧的数据和最新的数据来更新线速度和角速度
 */
void PoseExtrapolator::UpdateVelocitiesFromPoses() 
{
    if (timed_pose_queue_.size() < 2) {
        // We need two poses to estimate velocities.
        return;
    }
    CHECK(!timed_pose_queue_.empty());
    // 确保timed_pose_queue_队列中有2帧数据
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    const auto newest_time = newest_timed_pose.time;
    const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
    const auto oldest_time = oldest_timed_pose.time;
    const double queue_delta = common::ToSeconds(newest_time - oldest_time);
    if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
        LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                    << queue_delta << " s";
        return;
    }
    // 更新线速度和角速度
    const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
    const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
    linear_velocity_from_poses_ =
        (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
    angular_velocity_from_poses_ =
        transform::RotationQuaternionToAngleAxisVector(
            oldest_pose.rotation().inverse() * newest_pose.rotation()) /
        queue_delta;
}

void PoseExtrapolator::TrimImuData() 
{
    // 弹出没有用的imu数据
    // 须满足以下条件：
    // 1. imu_data_有2个或以上imu数据 (即确保pop之后，至少还有1个imu数据在队列中)
    // 2. timed_pose_queue_位姿队列非空
    // 3. imu_data_[1].time <= timed_pose_queue_.back().time
    while (imu_data_.size() > 1 
            && !timed_pose_queue_.empty() 
            && imu_data_[1].time <= timed_pose_queue_.back().time) 
    { imu_data_.pop_front(); }
}

void PoseExtrapolator::TrimOdometryData() 
{
    // (即确保pop之后，至少还有2个odom数据在队列中)
    while (odometry_data_.size() > 2 
            && !timed_pose_queue_.empty() 
            && odometry_data_[1].time <= timed_pose_queue_.back().time) 
    { odometry_data_.pop_front(); }
}

/**
 * @brief 利用imu_data_队列中的imu数据，将imu_tracker向前推算到指定时间time
 * @param time 指定期望时间
 * @param imu_tracker
 */
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const 
{
    // 确保time >= imu_tracker->time()， 不然后续没法调用imu_tracker->Advance(time);
    CHECK_GE(time, imu_tracker->time());
    // 如果imu_data_数据队列为空，或者要推算的时间time比imu_data_队列最旧的数据时间戳还早，则无法推算
    if (imu_data_.empty() || time < imu_data_.front().time) {
        // There is no IMU data until 'time', so we advance the ImuTracker and use
        // the angular velocities from poses and fake gravity to help 2D stability.
        imu_tracker->Advance(time);
        imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
        imu_tracker->AddImuAngularVelocityObservation(
            odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                    : angular_velocity_from_odometry_);
        return;
    }
    // 如果imu_tracker->time() 时间戳 < imu_data_队列最旧的数据时间戳
    if (imu_tracker->time() < imu_data_.front().time) {
        // Advance to the beginning of 'imu_data_'.
        // 需要先将imu_tracker向前推算到imu_data_队列最旧的数据时间戳
        imu_tracker->Advance(imu_data_.front().time);
    }
    // 从imu_data_队列中，使用二分法查找时间上大于imu_tracker->time()时间戳的数据，找到下界it
    auto it = std::lower_bound(
        imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
        [](const sensor::ImuData& imu_data, const common::Time& time) {
            return imu_data.time < time;
        });
    // 利用时间戳满足[imu_tracker->time(), time)的IMU数据，imu_tracker进行推算
    while (it != imu_data_.end() && it->time < time) {
        imu_tracker->Advance(it->time);
        imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
        imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
        ++it;
    }
    // imu_tracker推算到指定时间点time
    imu_tracker->Advance(time);
}

/**
 * @brief 计算得到两个ImuTracker的旋转变换: 新的imu_tracker 到 旧imu_tracker_的姿态变换
 * @param time
 * @param imu_tracker
 * @return
 */
Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const 
{
    CHECK_GE(time, imu_tracker->time());
    // 将另一个ImuTracker推算到指定时间time
    AdvanceImuTracker(time, imu_tracker);
    // 取imu_tracker_的姿态
    const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
    // 计算得到两个ImuTracker的旋转变换: 新的imu_tracker 到 旧imu_tracker_的姿态变换
    return last_orientation.inverse() * imu_tracker->orientation();
}

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) 
{
    const TimedPose& newest_timed_pose = timed_pose_queue_.back();
    const double extrapolation_delta =
        common::ToSeconds(time - newest_timed_pose.time);
    if (odometry_data_.size() < 2) {
        return extrapolation_delta * linear_velocity_from_poses_;
    }
    return extrapolation_delta * linear_velocity_from_odometry_;
}

PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) 
{
    std::vector<transform::Rigid3f> poses;
    for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
        poses.push_back(ExtrapolatePose(*it).cast<float>());
    }

    const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
                                                ? linear_velocity_from_poses_
                                                : linear_velocity_from_odometry_;
    return ExtrapolationResult{poses, ExtrapolatePose(times.back()),
                                current_velocity,
                                EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace csmlio
