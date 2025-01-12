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

#ifndef ROS_APP_ROS_MAP_H
#define ROS_APP_ROS_MAP_H

#include <string>

#include "Eigen/Core"
#include "csmlio/io/file_writer.h"
#include "csmlio/io/image.h"
#include "csmlio/lio/2d/map_limits.h"

namespace ros_app {

// Write 'image' as a pgm into 'file_writer'. The resolution is used in the
// comment only'
void WritePgm(const ::csmlio::io::Image& image, const double resolution,
              ::csmlio::io::FileWriter* file_writer);

// Write the corresponding yaml into 'file_writer'.
void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& pgm_filename,
               ::csmlio::io::FileWriter* file_writer);

}  // namespace ros_app

#endif  // ROS_APP_ROS_MAP_H
