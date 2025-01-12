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

#ifndef CSMLIO_MAPPING_INTERNAL_NORMAL_ESTIMATION_2D_H_
#define CSMLIO_MAPPING_INTERNAL_NORMAL_ESTIMATION_2D_H_

#include <vector>

#include "csmlio/lio/proto/normal_estimation_options_2d.pb.h"
#include "csmlio/sensor/point_cloud.h"
#include "csmlio/sensor/range_data.h"
#include "csmlio/transform/transform.h"

namespace csmlio {
namespace mapping {

proto::NormalEstimationOptions2D CreateNormalEstimationOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

// Estimates the normal for each 'return' in 'range_data'.
// Assumes the angles in the range data returns are sorted with respect to
// the orientation of the vector from 'origin' to 'return'.
std::vector<float> EstimateNormals(
    const sensor::RangeData& range_data,
    const proto::NormalEstimationOptions2D& normal_estimation_options);

}  // namespace mapping
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_INTERNAL_NORMAL_ESTIMATION_2D_H_
