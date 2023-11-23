/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 17:33
 * @Description: Tracker Pyramid
 * @FilePath: Bitplanes/source/BitplanesTrackerPyramid.cc
 */
#include "PyramidTracker.h"
#include "MotionModel.h"

#include <opencv2/opencv.hpp>

NAMESPACE_BEGIN
static inline
Parameters ReduceAlgorithmParameters(Parameters p) {
  p.max_iterations = 25;
  p.parameter_tolerance *= 10;
  p.function_tolerance *= 10;
  p.sigma = 0.8;
  return p;
}

static inline
std::vector<Parameters>
MakeAlgorithmParametersPyramid(Parameters p) {
  assert(p.num_levels >= 1);
  std::vector<Parameters> ret(p.num_levels);
  ret[0] = p;

  for (size_t i = 1; i < ret.size(); ++i)
    ret[i] = ReduceAlgorithmParameters(ret[0]);

  return ret;
}


template<class M>
void PyramidTracker<M>::setTemplate(const cv::Mat &I, const cv::Rect &bbox) {
  // 1.创建金字塔参数
  auto alg_params = MakeAlgorithmParametersPyramid(alg_params_);

  // 2.拷贝传入图像
  cv::Mat I0;
  I.copyTo(I0);

  // 3.创建跟踪器金字塔
  pyramid_.clear();
  for (size_t i = 0; i < alg_params.size(); ++i) {
    pyramid_.push_back(Tracker(alg_params[i]));
  }

  // 4.为金字塔每一层设置模板
  pyramid_[0].setTemplate(I0, bbox);
  cv::Rect bbox_copy(bbox);
  for (size_t i = 1; i < pyramid_.size(); ++i) {
    cv::pyrDown(I0, I0);
    bbox_copy.x /= 2;
    bbox_copy.y /= 2;
    bbox_copy.width /= 2;
    bbox_copy.height /= 2;
    pyramid_[i].setTemplate(I0, bbox_copy);
  }

  // 5.将初始位姿设置位单位矩阵
  T_init_.setIdentity();
}

template<class M>
Result PyramidTracker<M>::Track(const cv::Mat &I, const Transform &T_init) {
  float s = 1.0f / (1 << (pyramid_.size() - 1));
  Result ret(MotionModelType::Scale(T_init, s));

  std::vector<cv::Mat> I_pyr(pyramid_.size());
  I.copyTo(I_pyr[0]);
  for (size_t i = 1; i < I_pyr.size(); ++i)
    cv::pyrDown(I_pyr[i - 1], I_pyr[i]);

  for (int i = (int) pyramid_.size() - 1; i >= 0; --i) {
    ret = pyramid_[i].Track(I_pyr[i], ret.T);
    if (i != 0) ret.T = MotionModelType::Scale(ret.T, 2.0);
  }

  T_init_ = ret.T;
  return ret;
}

template
class PyramidTracker<Homography>;


NAMESPACE_END
