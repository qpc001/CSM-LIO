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

#ifndef CSMLIO_IO_FRAME_ID_FILTERING_POINTS_PROCESSOR_H_
#define CSMLIO_IO_FRAME_ID_FILTERING_POINTS_PROCESSOR_H_

#include "absl/container/flat_hash_set.h"
#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/io/points_processor.h"

namespace csmlio {
namespace io {

// Filters all points with blacklisted frame id or a non-whitelisted frame id.
// Note that you can either specify the whitelist or the blacklist, but not both
// at the same time.
class FrameIdFilteringPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "frame_id_filter";
  FrameIdFilteringPointsProcessor(
      const absl::flat_hash_set<std::string>& keep_frame_ids,
      const absl::flat_hash_set<std::string>& drop_frame_ids,
      PointsProcessor* next);
  static std::unique_ptr<FrameIdFilteringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);
  ~FrameIdFilteringPointsProcessor() override {}

  FrameIdFilteringPointsProcessor(const FrameIdFilteringPointsProcessor&) =
      delete;
  FrameIdFilteringPointsProcessor& operator=(
      const FrameIdFilteringPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const absl::flat_hash_set<std::string> keep_frame_ids_;
  const absl::flat_hash_set<std::string> drop_frame_ids_;
  PointsProcessor* const next_;
};

}  // namespace io
}  // namespace csmlio

#endif  // CSMLIO_IO_FRAME_ID_FILTERING_POINTS_PROCESSOR_H_
