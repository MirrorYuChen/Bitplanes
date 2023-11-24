/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/24 15:47
 * @Description: Bounded Buffer
 * @FilePath: Practice/test/BoundedBuffer.h
 */
#pragma once

#include "API.h"
#include "circular_buffer.h"
#include <mutex>
#include <condition_variable>

NAMESPACE_BEGIN
template<class T>
class BoundedBuffer {
public:
  typedef cb::circular_buffer<T> Container_t;
  typedef typename Container_t::size_type size_type;
  typedef typename Container_t::value_type value_type;
  typedef value_type &&param_type;

public:
  /**
   * Initializers a buffers with at most 'capacity' elements
   */
  explicit BoundedBuffer(size_type capacity) :
    unread_(0), container_(capacity) {}
  BoundedBuffer(const BoundedBuffer &) = delete;
  BoundedBuffer &operator=(const BoundedBuffer &) = delete;
  ~BoundedBuffer() = default;

  /**
   * pushes an element into the buffer
   * If the buffer is full, the function waits until a slot is available
   */
  void push(param_type item) {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_not_full_.wait(lock, [=] { return unread_ < container_.capacity(); });
    container_.push_front(std::move(item));
    ++unread_;
    lock.unlock();
    cond_not_empty_.notify_one();
  }

  /**
   * set 'item' to the first element that was pushed into the buffer.
   *
   * The function waits for the specified milliseconds for data if the buffer
   * was empty
   *
   * \return true if we popped something, false otherwise (timer has gone off)
   */
  bool pop(value_type *item, int wait_time_ms = 1) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (cond_not_empty_.wait_for(lock, std::chrono::milliseconds(wait_time_ms),
                                 [=] { return unread_ > 0; })) {
      (*item).swap(container_[--unread_]);
      lock.unlock();
      cond_not_full_.notify_one();
      return true;
    } else {
      lock.unlock();
      return false;
    }
  }

  /**
   * \return true if the buffer is full (if possible)
   */
  bool full() {
    if (mutex_.try_lock()) {
      bool ret = container_.full();
      mutex_.unlock();
      return ret;
    }
    return false;
  }

  /**
   * \return the size of the buffer (number of elements) if we are able to get a
   * lock. If not, we return -1
   */
  int size() {
    int ret = -1;
    if (mutex_.try_lock()) {
      ret = static_cast<int>(container_.size());
      mutex_.unlock();
    }
    return ret;
  }

private:
  size_type unread_;
  Container_t container_;
  std::mutex mutex_;
  std::condition_variable cond_not_empty_;
  std::condition_variable cond_not_full_;
};

NAMESPACE_END
