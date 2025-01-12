
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

#ifndef CSMLIO_MAPPING_INTERNAL_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_
#define CSMLIO_MAPPING_INTERNAL_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_

#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/lio/proto/scan_matching/real_time_correlative_scan_matcher_options.pb.h"

namespace csmlio {
namespace mapping {
namespace scan_matching {

proto::RealTimeCorrelativeScanMatcherOptions
CreateRealTimeCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace scan_matching
}  // namespace mapping
}  // namespace csmlio

#endif  // CSMLIO_MAPPING_INTERNAL_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_
