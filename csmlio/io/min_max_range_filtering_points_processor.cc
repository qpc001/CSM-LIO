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

#include "csmlio/io/min_max_range_filtering_points_processor.h"

#include "absl/memory/memory.h"
#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/io/points_batch.h"

namespace csmlio {
namespace io {

std::unique_ptr<MinMaxRangeFilteringPointsProcessor>
MinMaxRangeFilteringPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return absl::make_unique<MinMaxRangeFilteringPointsProcessor>(
      dictionary->GetDouble("min_range"), dictionary->GetDouble("max_range"),
      next);
}

MinMaxRangeFilteringPointsProcessor::MinMaxRangeFilteringPointsProcessor(
    const double min_range, const double max_range, PointsProcessor* next)
    : min_range_squared_(min_range * min_range),
      max_range_squared_(max_range * max_range),
      next_(next) {}

void MinMaxRangeFilteringPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  absl::flat_hash_set<int> to_remove;
  for (size_t i = 0; i < batch->points.size(); ++i) {
    const float range_squared =
        (batch->points[i].position - batch->origin).squaredNorm();
    if (!(min_range_squared_ <= range_squared &&
          range_squared <= max_range_squared_)) {
      to_remove.insert(i);
    }
  }
  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult MinMaxRangeFilteringPointsProcessor::Flush() {
  return next_->Flush();
}

}  // namespace io
}  // namespace csmlio
