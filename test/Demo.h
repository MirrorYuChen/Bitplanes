/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/24 15:18
 * @Description: Demo
 * @FilePath: Practice/test/Demo.h
 */
#pragma once

#include <string>
#include <atomic>

class DemoLiveCapture {
public:
  /**
  * path to configuration file
  */
  explicit DemoLiveCapture();

  ~DemoLiveCapture();

  void stop();
  bool isRunning() const;

private:
  struct Impl;
  Impl* impl_;
};
