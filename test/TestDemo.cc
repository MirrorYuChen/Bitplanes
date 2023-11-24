/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/24 15:50
 * @Description: TestDemo
 * @FilePath: Practice/test/TestDemo.cc
 */

#include "Demo.h"
#include <thread>
#include <chrono>

void Sleep(int32_t ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
  DemoLiveCapture demo;
  while (demo.isRunning()) {
    Sleep(100);
  }
}
