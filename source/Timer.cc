/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 17:26
 * @Description: Timer
 * @FilePath: Bitplanes/source/Timer.cc
 */
#include "Timer.h"

NAMESPACE_BEGIN
void Timer::start() {
  start_time_ = std::chrono::high_resolution_clock::now();
}

auto Timer::stop() -> Milliseconds {
  auto t_now = std::chrono::high_resolution_clock::now();
  auto ret = std::chrono::duration_cast<Milliseconds>(t_now - start_time_);
  start_time_ = t_now;
  return ret;
}

auto Timer::elapsed() -> Milliseconds {
  auto t_now = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<Milliseconds>(t_now - start_time_);
}
NAMESPACE_END
