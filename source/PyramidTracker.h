/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 17:33
 * @Description: Tracker Pyramid
 * @FilePath: Bitplanes/source/BitplanesTrackerPyramid.h
 */
#pragma once

#include "API.h"
#include "Types.h"
#include "Parameters.h"
#include "Tracker.h"
#include <vector>
#include <iostream>

NAMESPACE_BEGIN
template<class M>
class PyramidTracker {
  typedef Tracker<M> Tracker;

public:
  typedef typename Tracker::Transform Transform;
  typedef typename Tracker::MotionModelType MotionModelType;

public:
  explicit PyramidTracker(const Parameters &p = Parameters())
    : alg_params_(p) {
    if (alg_params_.verbose)
      std::cout << "AlgorithmParameters:\n" << alg_params_ << std::endl;
  }

  ~PyramidTracker() = default;

  /**
   * sets the template
   *
   * \param I reference image
   * \param bbox template location
   */
  void setTemplate(const cv::Mat &, const cv::Rect &bbox);

  /**
   * Tracks the template
   *
   * \param I input image
   * \param T pose to use for initialization
   */
  Result Track(const cv::Mat &, const Transform &);

  /**
   * Tracks the template
   *
   * \param I input image
   *
   * Uses the previously estimated pose for initialization
   */
  Result Track(const cv::Mat &I) {
    return Track(I, T_init_);
  }

private:
  Parameters alg_params_;
  std::vector<Tracker> pyramid_;
  Transform T_init_ = Transform::Identity();
};

NAMESPACE_END
