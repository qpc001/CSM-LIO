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

#include "csmlio/sensor/internal/collator.h"
#include "glog/logging.h"

namespace csmlio {
namespace sensor {

/**
 * @brief 主要作用: 为指定的某个传感器的消息队列设置回调函数
 * @param trajectory_id  轨迹ID
 * @param expected_sensor_ids 传感器id (字符串)
 * @param callback 回调函数，这里进来的是 sensor_collator_->AddTrajectory 时的lambda函数
 */
void Collator::AddTrajectory(
    const int trajectory_id,
    const absl::flat_hash_set<std::string>& expected_sensor_ids,
    const Callback& callback) 
{
    // 也就是说，对于同一个轨迹id的所有回调，都是这里的callback
    for (const auto& sensor_id : expected_sensor_ids) {
        const auto queue_key = QueueKey{trajectory_id, sensor_id};
        queue_.AddQueue(queue_key,
                        [callback, sensor_id](std::unique_ptr<Data> data) {
                        callback(sensor_id, std::move(data));
                        });
        queue_keys_[trajectory_id].push_back(queue_key);
        LOG(INFO) << "Collator registered sensor_id " << sensor_id << " .";
    }
}

void Collator::FinishTrajectory(const int trajectory_id) 
{
    for (const auto& queue_key : queue_keys_[trajectory_id]) {
        queue_.MarkQueueAsFinished(queue_key);
    }
}

/**
 * @brief 主要的操作,添加传感器数据,数据形式是:key+data
 * @param trajectory_id
 * @param data
 */
void Collator::AddSensorData(const int trajectory_id,
                             std::unique_ptr<Data> data) 
{
    QueueKey queue_key{trajectory_id, data->GetSensorId()};
    queue_.Add(std::move(queue_key), std::move(data));
}

void Collator::Flush() { queue_.Flush(); }

absl::optional<int> Collator::GetBlockingTrajectoryId() const 
{
    return absl::optional<int>(queue_.GetBlocker().trajectory_id);
}

}  // namespace sensor
}  // namespace csmlio
