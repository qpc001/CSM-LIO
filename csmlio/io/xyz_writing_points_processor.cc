#include "csmlio/io/xyz_writing_points_processor.h"

#include <iomanip>

#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace csmlio {
namespace io {

namespace {

void WriteXyzPoint(const Eigen::Vector3f& point,
                   FileWriter* const file_writer) {
  std::ostringstream stream;
  stream << std::setprecision(6);
  stream << point.x() << " " << point.y() << " " << point.z() << "\n";
  const std::string out = stream.str();
  CHECK(file_writer->Write(out.data(), out.size()));
}

}  // namespace

XyzWriterPointsProcessor::XyzWriterPointsProcessor(
    std::unique_ptr<FileWriter> file_writer, PointsProcessor* const next)
    : next_(next), file_writer_(std::move(file_writer)) {}

std::unique_ptr<XyzWriterPointsProcessor>
XyzWriterPointsProcessor::FromDictionary(
    const FileWriterFactory& file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return absl::make_unique<XyzWriterPointsProcessor>(
      file_writer_factory(dictionary->GetString("filename")), next);
}

PointsProcessor::FlushResult XyzWriterPointsProcessor::Flush() {
  CHECK(file_writer_->Close()) << "Closing XYZ file failed.";
  switch (next_->Flush()) {
    case FlushResult::kFinished:
      return FlushResult::kFinished;

    case FlushResult::kRestartStream:
      LOG(FATAL) << "XYZ generation must be configured to occur after any "
                    "stages that require multiple passes.";
  }
  LOG(FATAL);
}

void XyzWriterPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  for (const sensor::RangefinderPoint& point : batch->points) {
    WriteXyzPoint(point.position, file_writer_.get());
  }
  next_->Process(std::move(batch));
}

}  // namespace io
}  // namespace csmlio
