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

#ifndef CSMLIO_COMMON_CERES_SOLVER_OPTIONS_H_
#define CSMLIO_COMMON_CERES_SOLVER_OPTIONS_H_

#include "csmlio/common/lua_parameter_dictionary.h"
#include "csmlio/common/proto/ceres_solver_options.pb.h"
#include "ceres/ceres.h"

namespace csmlio {
namespace common {

proto::CeresSolverOptions CreateCeresSolverOptionsProto(
    common::LuaParameterDictionary* parameter_dictionary);

ceres::Solver::Options CreateCeresSolverOptions(
    const proto::CeresSolverOptions& proto);

}  // namespace common
}  // namespace csmlio

#endif  // CSMLIO_COMMON_CERES_SOLVER_OPTIONS_H_
