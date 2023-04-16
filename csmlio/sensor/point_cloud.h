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

#ifndef CSMLIO_SENSOR_POINT_CLOUD_H_
#define CSMLIO_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "csmlio/sensor/proto/sensor.pb.h"
#include "csmlio/sensor/rangefinder_point.h"
#include "csmlio/transform/rigid_transform.h"
#include "glog/logging.h"

namespace csmlio {
namespace sensor {

// Stores 3D positions of points together with some additional data, e.g.
// intensities.
/**
 * @brief 主要使用的点云数据结构,但是没有时间属性
 */
class PointCloud {
 public:
  using PointType = RangefinderPoint;

  PointCloud();
  explicit PointCloud(std::vector<PointType> points);
  PointCloud(std::vector<PointType> points, std::vector<float> intensities);

  // Returns the number of points in the point cloud.
  size_t size() const;
  // Checks whether there are any points in the point cloud.
  bool empty() const;

  const std::vector<PointType>& points() const;
  const std::vector<float>& intensities() const;
  const PointType& operator[](const size_t index) const;

  // Iterator over the points in the point cloud.
  using ConstIterator = std::vector<PointType>::const_iterator;
  ConstIterator begin() const;
  ConstIterator end() const;

  void push_back(PointType value);

  // Creates a PointCloud consisting of all the points for which `predicate`
  // returns true, together with the corresponding intensities.
  template <class UnaryPredicate>
  PointCloud copy_if(UnaryPredicate predicate) const {
    std::vector<PointType> points;
    std::vector<float> intensities;

    // Note: benchmarks show that it is better to have this conditional outside
    // the loop.
    if (intensities_.empty()) {
      for (size_t index = 0; index < size(); ++index) {
        const PointType& point = points_[index];
        if (predicate(point)) {
          points.push_back(point);
        }
      }
    } else {
      for (size_t index = 0; index < size(); ++index) {
        const PointType& point = points_[index];
        if (predicate(point)) {
          points.push_back(point);
          intensities.push_back(intensities_[index]);
        }
      }
    }

    return PointCloud(points, intensities);
  }

 private:
  // For 2D points, the third entry is 0.f.
  // 储存3D点的位置，对于2D，则第3维=0
  std::vector<PointType> points_;
  // Intensities are optional. If non-empty, they must have the same size as
  // points.
  std::vector<float> intensities_;
};

// Stores 3D positions of points with their relative measurement time in the
// fourth entry. Time is in seconds, increasing and relative to the moment when
// the last point was acquired. So, the fourth entry for the last point is 0.f.
// If timing is not available, all fourth entries are 0.f. For 2D points, the
// third entry is 0.f (and the fourth entry is time).
// 储存3D点位置以及它们的相对测量时间，储存在第4维
using TimedPointCloud = std::vector<TimedRangefinderPoint>;

// TODO(wohe): Retained for cartographer_ros. To be removed once it is no
// longer used there.
// 旧版的点云数据结构，主要是使用TimedPointCloud，多了个时间属性，可以用来去畸变
struct PointCloudWithIntensities {
  TimedPointCloud points;
  std::vector<float> intensities;
};

// Transforms 'point_cloud' according to 'transform'.
// 根据3D变换，对点云进行刚体变换
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

// Transforms 'point_cloud' according to 'transform'.
// Transforms 'point_cloud' according to 'transform'.
// 根据3D变换，对点云进行刚体变换
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
// 通过设置z轴的范围，去除范围以外的点，返回区域内的点云
PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
                          float max_z);

}  // namespace sensor
}  // namespace csmlio

#endif  // CSMLIO_SENSOR_POINT_CLOUD_H_
