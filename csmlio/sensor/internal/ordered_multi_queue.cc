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

#include "csmlio/sensor/internal/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>
#include <iomanip>

#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace csmlio {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace

inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

OrderedMultiQueue::OrderedMultiQueue() {}

OrderedMultiQueue::~OrderedMultiQueue() {
  for (auto& entry : queues_) {
    CHECK(entry.second.finished);
  }
}

/**
 * @brief 添加传感器数据队列，并且绑定对应的回调函数
 * @param queue_key
 * @param callback
 */
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  CHECK_EQ(queues_.count(queue_key), 0);
  queues_[queue_key].callback = std::move(callback);
}

/**
 * @brief 将指定queue_key的队列标记为结束，一旦最后一个数据被分发完成，队列将被删除(Dispatch()函数中)
 * @param queue_key
 */
void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";
  auto& queue = it->second;
  CHECK(!queue.finished);
  queue.finished = true;
  Dispatch();
}

void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  // 检查queues_容器中是否有对应的QueueKey
  auto it = queues_.find(queue_key);
  // 没有，则警告，且返回
  if (it == queues_.end()) {
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }
  // 向对应的common::BlockingQueue<std::unique_ptr<Data>>阻塞队列中push数据
  it->second.queue.Push(std::move(data));
  // 执行回调
  Dispatch();
}

void OrderedMultiQueue::Flush() {
  std::vector<QueueKey> unfinished_queues;
  for (auto& entry : queues_) {
    if (!entry.second.finished) {
      unfinished_queues.push_back(entry.first);
    }
  }
  for (auto& unfinished_queue : unfinished_queues) {
    MarkQueueAsFinished(unfinished_queue);
  }
}

QueueKey OrderedMultiQueue::GetBlocker() const {
  CHECK(!queues_.empty());
  return blocker_;
}

/**
 * @brief 数据传递的核心函数，每调用一次，会执行以下步骤：
 * 1. 遍历所有队列，取时间上最小的数据（即最早的数据），保存到next_data、next_queue、next_queue_key
 * 1.1 如果遍历到的队列没有数据
 * 1.1.1 如果队列并且被标记为finished => 从queues_容器中删除队列，然后检查下一个队列
 * 1.1.2 如果没有被标记为finished，直接返回
 * 1.2 取时间上最小的数据（即最早的数据），保存到next_data、next_queue、next_queue_key
 *
 * 2. 使用next_data来执行回调，弹出数据，并记录last_dispatched_time_
 */
void OrderedMultiQueue::Dispatch() {
    // 此处一直循环，直到所有队列都没有新的数据要分发
    while (true) {
        const Data* next_data = nullptr;
        Queue* next_queue = nullptr;
        QueueKey next_queue_key;
        //遍历所有队列，取时间上最小的数据（即最早的数据），保存到next_data、next_queue、next_queue_key
        for (auto it = queues_.begin(); it != queues_.end();) {
            const auto* data = it->second.queue.Peek<Data>();
            // 如果队列没有数据
            if (data == nullptr) {
                // 并且被标记为finished
                if (it->second.finished) {
                    // 从queues_容器中删除队列
                    queues_.erase(it++);
                    continue;
                }
                // 1. 将输入的queue_key保存到blocker_ 2. 遍历所有队列，查看哪些队列长度超过缓存长度kMaxQueueSize，输出日志
                CannotMakeProgress(it->first);
                // TODO: 这里为啥不用continue?  [epsilon.john]
                return;
            }
            // 到这里，表明队列是有数据的
            // 如果next_data指针为空，或者当前数据时间戳小于next_data时间戳
            if (next_data == nullptr || data->GetTime() < next_data->GetTime()) {
                // 取时间上最早的数据，作为next_data
                next_data = data;
                next_queue = &it->second;
                next_queue_key = it->first;
            }
            CHECK_LE(last_dispatched_time_, next_data->GetTime())
                << "Non-sorted data added to queue: '" << it->first << "'";
            ++it;
        }
        // 如果遍历完所有队列，next_data仍然为空，则需要检查队列容器queues_是不是空的（即没有队列的情况）
        if (next_data == nullptr) {
            CHECK(queues_.empty());
            // 如果next_data为空，则直接返回
            return;
        }

        /// 到这里，表示已经准备好需要分发的数据了（next_data不为空）

        // If we haven't dispatched any data for this trajectory yet, fast forward
        // all queues of this trajectory until a common start time has been reached.
        // 输入指定轨迹id，获取同一轨迹id下所有队列数据最新的时间戳
        const common::Time common_start_time =
            GetCommonStartTime(next_queue_key.trajectory_id);

        if (next_data->GetTime() >= common_start_time) {                    // 如果next_data时间 >= common_start_time
            // Happy case, we are beyond the 'common_start_time' already.
            // 最好的情况，准备分发的数据的时间在'common_start_time'之后
            last_dispatched_time_ = next_data->GetTime();
            // 执行回调并弹出数据
            next_queue->callback(next_queue->queue.Pop());
        } else if (next_queue->queue.Size() < 2) {                          // 如果next_data时间 < common_start_time 且 next_queue->queue.Size() < 2
            // 检查队列是否标记为finished
            if (!next_queue->finished) {
                // We cannot decide whether to drop or dispatch this yet.
                // 轨迹还没结束，此时无法确定应该丢弃还是调用回调，这里直接报警告，然后返回
                CannotMakeProgress(next_queue_key);
                return;
            }
            // 如果队列被标记为finished，则分发最后一个数据
            last_dispatched_time_ = next_data->GetTime();
            next_queue->callback(next_queue->queue.Pop());
        } else {                                                            // 如果next_data时间 < common_start_time 且 next_queue->queue.Size() >= 2
            // We take a peek at the time after next data. If it also is not beyond
            // 'common_start_time' we drop 'next_data', otherwise we just found the
            // first packet to dispatch from this queue.
            // 这种情况下，我们再看一眼next_data的下一个数据，
            // 检查他的时间戳，如果下一个数据的时间戳正常，那就不管他，使用next_data执行一次回调
            // 如果下一个数据的时间戳也不正常，则丢弃next_data，啥也不干
            std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
            if (next_queue->queue.Peek<Data>()->GetTime() > common_start_time) {
                last_dispatched_time_ = next_data->GetTime();
                next_queue->callback(std::move(next_data_owner));
            }
        }
    }

}

/**
 * @brief 1. 将输入的queue_key保存到blocker_ 2. 遍历所有队列，查看哪些队列长度超过缓存长度kMaxQueueSize，输出日志
 * @param queue_key
 */
void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) 
{
    blocker_ = queue_key;
    for (auto& entry : queues_) {
        if (entry.second.queue.Size() > kMaxQueueSize) {
        LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;
        return;
        }
    }
}

/**
 * @brief 输入指定轨迹id，获取同一轨迹id下所有队列数据最新的时间戳
 * @param trajectory_id
 * @return
 */
common::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) 
{
    // std::chrono::time_point::min  <- 返回一个最小的时间点 （用来初始化,功效等同与-1）
    // 这里使用了std::map::emplace，只有在第一次插入时，会返回true的结果
    // emplace_result：返回一个由bool组成的对，表示是否发生了插入，并将迭代器返回给新插入的元素
    auto emplace_result = common_start_time_per_trajectory_.emplace(
        trajectory_id, common::Time::min());
    common::Time& common_start_time = emplace_result.first->second;
    // 检查是否第一次插入，否则，直接返回第一次插入时的common_start_time
    if (emplace_result.second) {
        // 遍历queues_中的所有队列
        for (auto& entry : queues_) {
            // 找到对应轨迹id的队列
            if (entry.first.trajectory_id == trajectory_id) {
                // 找到最大的时间戳
                common_start_time = std::max(
                    common_start_time, entry.second.queue.Peek<Data>()->GetTime());
            }
        }
        LOG(INFO) << "All sensor data for trajectory " << trajectory_id
                << " is available starting at '" << common_start_time << "'.";
        LOG(INFO) << "All sensor data for trajectory " << trajectory_id
                << " is available starting at '" 
                << std::fixed << std::setprecision(6) 
                << common::ToSeconds(common_start_time) << "'.";
    }
    // 这里返回的是同一轨迹id下所有队列数据最新的时间
    return common_start_time;
}

}  // namespace sensor
}  // namespace csmlio
