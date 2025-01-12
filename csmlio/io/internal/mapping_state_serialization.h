/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CSMLIO_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_
#define CSMLIO_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_

#include "csmlio/io/proto_stream_interface.h"
#include "csmlio/lio/pose_graph.h"
#include "csmlio/lio/proto/trajectory_builder_options.pb.h"

namespace csmlio {
namespace io {

// The current serialization format version.
static constexpr int kMappingStateSerializationFormatVersion = 2;
static constexpr int kFormatVersionWithoutSubmapHistograms = 1;

// Serialize mapping state to a pbstream.
void WritePbStream(
    const mapping::PoseGraph& pose_graph,
    const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        builder_options,
    ProtoStreamWriterInterface* const writer, bool include_unfinished_submaps);

}  // namespace io
}  // namespace csmlio

#endif  // CSMLIO_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_
