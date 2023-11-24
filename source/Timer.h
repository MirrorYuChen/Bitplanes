/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 17:25
 * @Description: Timer
 * @FilePath: Bitplanes/source/Timer.h
 */
#pragma once

#include "API.h"
#include <chrono>

NAMESPACE_BEGIN
/**
 * Simple timer. The timer is enabled if BITPLANES_WITH_TIMING is enabled when
 * compiling code.
 */
class Timer {
  typedef std::chrono::milliseconds Milliseconds;

public:
  /**
   * By default the constructor start timing.
   */
  inline Timer() { start(); }

  /**
   * Calling start again will reset the time
   */
  void start();

  /**
   * Stops the timer. The start time point is reset
   */
  Milliseconds stop();

  /**
   * Keeps the timer running and reports elapsed milliseconds since the last
   * call to start() (or stop())
   */
  Milliseconds elapsed();

protected:
  std::chrono::high_resolution_clock::time_point start_time_;
};


/**
 * Times a piece of code by running N_rep and returns the average run time in
 * milliseconds
 *
 * Example
 *
 * MyClass my_class;
 * auto t = TimeCode(100, [=]() { my_class.runOperation(); });
 * std::cout << "Time: " << t << " ms\n";
 */
template<class Func, class ...Args>
static inline
double TimeCode(int N_rep, Func &&f, Args... args) {
  Timer timer;
  for (int i = 0; i < N_rep; ++i)
    f(args...);
  auto t = timer.stop();
  return static_cast<double>(t.count()) / (double) N_rep;
}
NAMESPACE_END
