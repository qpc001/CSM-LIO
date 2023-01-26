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

#ifndef CSMLIO_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
#define CSMLIO_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "csmlio/common/internal/blocking_queue.h"
#include "csmlio/common/port.h"
#include "csmlio/common/time.h"
#include "csmlio/sensor/data.h" // #include "csmlio/sensor/internal/dispatchable.h"

namespace csmlio {
namespace sensor {

struct QueueKey {
    int trajectory_id;
    std::string sensor_id;

    bool operator<(const QueueKey& other) const {
        return std::forward_as_tuple(trajectory_id, sensor_id) <
            std::forward_as_tuple(other.trajectory_id, other.sensor_id);
    }
};

// Maintains multiple queues of sorted sensor data and dispatches it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
//
// This class is thread-compatible.
/**
 * @brief 这个类维护了多个储存传感器数据的队列，并且根据储存的顺序进行分发(Dispatch)
 * 对于没有被调用FinishTrajectory ... MarkQueueAsFinished的队列，本类将等待至少一个数据来进行分发
 *
 * 这个类是线程兼容的
 * 线程兼容（Thread-compatible）：此类提供对const方法的安全并发访问，但需要对非const（或混合const /非const访问）进行同步。
 * 这样，唯一的用例就是对象（通常是全局对象），这些对象以线程安全的方式（在启动的单线程阶段或通过线程安全的静态局部局部化范式惰性地初始化）
 */
class OrderedMultiQueue {
  public:
    using Callback = std::function<void(std::unique_ptr<Data>)>;

    OrderedMultiQueue();
    OrderedMultiQueue(OrderedMultiQueue&& queue) = default;

    ~OrderedMultiQueue();

    // Adds a new queue with key 'queue_key' which must not already exist.
    // 'callback' will be called whenever data from this queue can be dispatched.
    void AddQueue(const QueueKey& queue_key, Callback callback);

    // Marks a queue as finished, i.e. no further data can be added. The queue
    // will be removed once the last piece of data from it has been dispatched.
    void MarkQueueAsFinished(const QueueKey& queue_key);

    // Adds 'data' to a queue with the given 'queue_key'. Data must be added
    // sorted per queue.
    void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);

    // Dispatches all remaining values in sorted order and removes the underlying
    // queues.
    void Flush();

    // Must only be called if at least one unfinished queue exists. Returns the
    // key of a queue that needs more data before the OrderedMultiQueue can
    // dispatch data.
    QueueKey GetBlocker() const;

  private:
    struct Queue {
        common::BlockingQueue<std::unique_ptr<Data>> queue;
        Callback callback;
        bool finished = false;
    };

    void Dispatch();
    void CannotMakeProgress(const QueueKey& queue_key);
    common::Time GetCommonStartTime(int trajectory_id);

    // Used to verify that values are dispatched in sorted order.
    common::Time last_dispatched_time_ = common::Time::min();

    std::map<int, common::Time> common_start_time_per_trajectory_;
    std::map<QueueKey, Queue> queues_;
    QueueKey blocker_;
};

}  // namespace sensor
}  // namespace csmlio

#endif  // CSMLIO_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
