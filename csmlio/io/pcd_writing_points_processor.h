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

#include <fstream>

#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/io/file_writer.h"
#include "csmlio/io/points_processor.h"

namespace csmlio {
namespace io {

// Streams a PCD file to disk. The header is written in 'Flush'.
class PcdWritingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "write_pcd";
  PcdWritingPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                            PointsProcessor* next);

  static std::unique_ptr<PcdWritingPointsProcessor> FromDictionary(
      FileWriterFactory file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~PcdWritingPointsProcessor() override {}

  PcdWritingPointsProcessor(const PcdWritingPointsProcessor&) = delete;
  PcdWritingPointsProcessor& operator=(const PcdWritingPointsProcessor&) =
      delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  PointsProcessor* const next_;

  int64 num_points_;
  bool has_colors_;
  std::unique_ptr<FileWriter> file_writer_;
};

}  // namespace io
}  // namespace csmlio
