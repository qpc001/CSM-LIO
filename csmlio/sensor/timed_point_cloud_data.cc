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

#include "csmlio/sensor/timed_point_cloud_data.h"

#include "csmlio/transform/proto/transform.pb.h"
#include "csmlio/transform/transform.h"

namespace csmlio {
namespace sensor {

proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data) {
  proto::TimedPointCloudData proto;
  proto.set_timestamp(common::ToUniversal(timed_point_cloud_data.time));
  *proto.mutable_origin() = transform::ToProto(timed_point_cloud_data.origin);
  proto.mutable_point_data()->Reserve(timed_point_cloud_data.ranges.size());
  for (const TimedRangefinderPoint& range : timed_point_cloud_data.ranges) {
    *proto.add_point_data() = ToProto(range);
  }
  for (const float intensity : timed_point_cloud_data.intensities) {
    proto.add_intensities(intensity);
  }
  return proto;
}

TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto) {
  CHECK(proto.intensities().size() == 0 ||
        proto.intensities().size() == proto.point_data().size());
  TimedPointCloud timed_point_cloud;
  if (proto.point_data().size() > 0) {
    timed_point_cloud.reserve(proto.point_data().size());
    for (const auto& timed_point_proto : proto.point_data()) {
      timed_point_cloud.push_back(FromProto(timed_point_proto));
    }
  } else {
    timed_point_cloud.reserve(proto.point_data_legacy().size());
    for (const auto& timed_point_proto : proto.point_data_legacy()) {
      const Eigen::Vector4f timed_point = transform::ToEigen(timed_point_proto);
      timed_point_cloud.push_back({timed_point.head<3>(), timed_point[3]});
    }
  }
  return TimedPointCloudData{common::FromUniversal(proto.timestamp()),
                             transform::ToEigen(proto.origin()),
                             timed_point_cloud,
                             std::vector<float>(proto.intensities().begin(),
                                                proto.intensities().end())};
}

}  // namespace sensor
}  // namespace csmlio
