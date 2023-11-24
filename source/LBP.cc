/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 17:14
 * @Description: Census
 * @FilePath: Bitplanes/source/LBP.cc
 */
#include "LBP.h"
#include <cassert>

NAMESPACE_BEGIN
namespace simd {
void LBP(const cv::Mat &src, const cv::Rect &roi, cv::Mat &dst) {
  assert(src.type() == CV_8UC1);
  assert(src.isContinuous());
  assert(roi.x >= 1 || roi.x <= src.cols - 2 ||
         roi.y >= 1 || roi.y <= src.rows - 2);

  // 1.创建ROI对应区域
  dst.create(roi.size(), src.type());
  assert(!dst.empty());

  int src_stride = src.cols;
  for (int y = 0; y < roi.height; ++y) {
    const auto *s_row = src.ptr<const uint8_t>(y + roi.y);
    auto *d_row = dst.ptr<uint8_t>(y);

    /*
     *  P00 P01 P02
     *  P10  P  P12
     *  P20 P21 P22
     */
    // 2.ROI每个像素对应LBP编码
    for (int x = 0; x < roi.width; ++x) {
      const uint8_t *p = s_row + x + roi.x;
      d_row[x] =
        ((*(p - src_stride - 1) >= *p) << 0) |
        ((*(p - src_stride) >= *p) << 1) |
        ((*(p - src_stride + 1) >= *p) << 2) |
        ((*(p - 1) >= *p) << 3) |
        ((*(p + 1) >= *p) << 4) |
        ((*(p + src_stride - 1) >= *p) << 5) |
        ((*(p + src_stride) >= *p) << 6) |
        ((*(p + src_stride + 1) >= *p) << 7);
    }
  }
}

} // namespace simd
NAMESPACE_END
