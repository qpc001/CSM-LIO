#include "csmlio/io/hybrid_grid_points_processor.h"

#include <memory>
#include <string>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "csmlio/io/file_writer.h"
#include "csmlio/io/points_batch.h"
#include "csmlio/io/points_processor.h"
#include "csmlio/lio/3d/hybrid_grid.h"
#include "csmlio/lio/3d/range_data_inserter_3d.h"
#include "csmlio/sensor/range_data.h"
#include "glog/logging.h"

namespace csmlio {
namespace io {

HybridGridPointsProcessor::HybridGridPointsProcessor(
    const double voxel_size,
    const mapping::proto::RangeDataInserterOptions3D&
        range_data_inserter_options,
    std::unique_ptr<FileWriter> file_writer, PointsProcessor* const next)
    : next_(next),
      range_data_inserter_(range_data_inserter_options),
      hybrid_grid_(voxel_size),
      file_writer_(std::move(file_writer)) {}

std::unique_ptr<HybridGridPointsProcessor>
HybridGridPointsProcessor::FromDictionary(
    const FileWriterFactory& file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return absl::make_unique<HybridGridPointsProcessor>(
      dictionary->GetDouble("voxel_size"),
      mapping::CreateRangeDataInserterOptions3D(
          dictionary->GetDictionary("range_data_inserter").get()),
      file_writer_factory(dictionary->GetString("filename")), next);
}

void HybridGridPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  range_data_inserter_.Insert(
      {batch->origin, sensor::PointCloud(batch->points), {}}, &hybrid_grid_,
      /*intensity_hybrid_grid=*/nullptr);
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult HybridGridPointsProcessor::Flush() {
  const mapping::proto::HybridGrid hybrid_grid_proto = hybrid_grid_.ToProto();
  std::string serialized;
  hybrid_grid_proto.SerializeToString(&serialized);
  file_writer_->Write(serialized.data(), serialized.size());
  CHECK(file_writer_->Close());

  switch (next_->Flush()) {
    case FlushResult::kRestartStream:
      LOG(FATAL) << "Hybrid grid generation must be configured to occur after "
                    "any stages that require multiple passes.";

    case FlushResult::kFinished:
      return FlushResult::kFinished;
  }
  LOG(FATAL) << "Failed to receive FlushResult::kFinished";
  // The following unreachable return statement is needed to avoid a GCC bug
  // described at https://gcc.gnu.org/bugzilla/show_bug.cgi?id=81508
  return FlushResult::kFinished;
}

}  // namespace io
}  // namespace csmlio
