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

#include "csmlio/io/outlier_removing_points_processor.h"

#include "absl/memory/memory.h"
#include "csmlio/common/lua_parameter_dictionary.h"
#include "glog/logging.h"

namespace csmlio {
namespace io {

std::unique_ptr<OutlierRemovingPointsProcessor>
OutlierRemovingPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  const double miss_per_hit_limit = [&]() {
    if (!dictionary->HasKey("miss_per_hit_limit")) {
      LOG(INFO) << "Using default value of 3 for miss_per_hit_limit.";
      return 3.;
    } else {
      return dictionary->GetDouble("miss_per_hit_limit");
    }
  }();
  return absl::make_unique<OutlierRemovingPointsProcessor>(
      dictionary->GetDouble("voxel_size"), miss_per_hit_limit, next);
}

OutlierRemovingPointsProcessor::OutlierRemovingPointsProcessor(
    const double voxel_size, const double miss_per_hit_limit,
    PointsProcessor* next)
    : voxel_size_(voxel_size),
      miss_per_hit_limit_(miss_per_hit_limit),
      next_(next),
      state_(State::kPhase1),
      voxels_(voxel_size_) {
  LOG(INFO) << "Marking hits...";
}

void OutlierRemovingPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  switch (state_) {
    case State::kPhase1:
      ProcessInPhaseOne(*batch);
      break;

    case State::kPhase2:
      ProcessInPhaseTwo(*batch);
      break;

    case State::kPhase3:
      ProcessInPhaseThree(std::move(batch));
      break;
  }
}

PointsProcessor::FlushResult OutlierRemovingPointsProcessor::Flush() {
  switch (state_) {
    case State::kPhase1:
      LOG(INFO) << "Counting rays...";
      state_ = State::kPhase2;
      return FlushResult::kRestartStream;

    case State::kPhase2:
      LOG(INFO) << "Filtering outliers...";
      state_ = State::kPhase3;
      return FlushResult::kRestartStream;

    case State::kPhase3:
      CHECK(next_->Flush() == FlushResult::kFinished)
          << "Voxel filtering and outlier removal must be configured to occur "
             "after any stages that require multiple passes.";
      return FlushResult::kFinished;
  }
  LOG(FATAL);
}

void OutlierRemovingPointsProcessor::ProcessInPhaseOne(
    const PointsBatch& batch) {
  for (size_t i = 0; i < batch.points.size(); ++i) {
    ++voxels_.mutable_value(voxels_.GetCellIndex(batch.points[i].position))
          ->hits;
  }
}

void OutlierRemovingPointsProcessor::ProcessInPhaseTwo(
    const PointsBatch& batch) {
  // TODO(whess): This samples every 'voxel_size' distance and could be improved
  // by better ray casting, and also by marking the hits of the current range
  // data to be excluded.
  for (size_t i = 0; i < batch.points.size(); ++i) {
    const Eigen::Vector3f delta = batch.points[i].position - batch.origin;
    const float length = delta.norm();
    for (float x = 0; x < length; x += voxel_size_) {
      const Eigen::Array3i index =
          voxels_.GetCellIndex(batch.origin + (x / length) * delta);
      if (voxels_.value(index).hits > 0) {
        ++voxels_.mutable_value(index)->rays;
      }
    }
  }
}

void OutlierRemovingPointsProcessor::ProcessInPhaseThree(
    std::unique_ptr<PointsBatch> batch) {
  absl::flat_hash_set<int> to_remove; // wgh 保存所有动态点的索引。
  for (size_t i = 0; i < batch->points.size(); ++i) {
    const VoxelData voxel =
        voxels_.value(voxels_.GetCellIndex(batch->points[i].position));
    if (!(voxel.rays < miss_per_hit_limit_ * voxel.hits)) {
      to_remove.insert(i);
    }
  }
  RemovePoints(to_remove, batch.get()); // wgh 一次性干掉所有的动态点。
  next_->Process(std::move(batch));     // wgh 动态过滤后的点云，给到后续processor去处理。
}

}  // namespace io
}  // namespace csmlio
