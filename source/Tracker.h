/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 16:17
 * @Description: Tracker
 * @FilePath: Bitplanes/source/Tracker.h
 */
#pragma once

#include "API.h"
#include "Types.h"
#include "Parameters.h"
#include "MotionModel.h"
#include "ChannelDataSampler.h"

#include <opencv2/opencv.hpp>
#include <limits>
#include <fstream>
#include <array>

#include <Eigen/Cholesky>

NAMESPACE_BEGIN
template<class M>
class Tracker {
public:
  typedef Matrix33f Transform;
  typedef MotionModel<M> MotionModelType;

  typedef typename MotionModelType::Hessian Hessian;
  typedef typename MotionModelType::Gradient Gradient;
  typedef typename MotionModelType::ParameterVector ParameterVector;

  typedef typename Eigen::LDLT<Hessian> Solver;
  typedef ChannelDataSampler<M> ChannelDataType;

public:
  explicit Tracker(Parameters p = Parameters());
  ~Tracker() = default;

  /**
   * Sets the template
   *
   * \param image the template image (I_0)
   * \param bbox  location of the template in image
   */
  void setTemplate(const cv::Mat &image, const cv::Rect &bbox);

  /**
   * Tracks the template that was set during the call setTemplate
   *
   * \param image the input image (I_1)
   * \param T_init initialization of the transform
   */
  Result Track(const cv::Mat &image, const Transform &T_init = Transform::Identity());

protected:
  /**
   * Performs the linearization step, which is:
   *  - warp the image
   *  - re-compute the multi-channel descriptors
   *  - compute the cost function gradient (J^T * error)
   */
  float Linearize(const cv::Mat &, const Transform &T_init);

  /**
   * applies smoothing to the image at the specified ROI
   */
  void SmoothImage(cv::Mat &I, const cv::Rect &roi);

protected:
  Parameters params_;              //< AlgorithmParameters
  ChannelDataType cdata_;          //< holds the multi-channel data
  cv::Rect bbox_;                  //< the template's bounding box
  cv::Mat I_, Iw_;                 //< buffers for input image and warped image
  Matrix33f T_, T_inv_;            //< normalization matrices
  Gradient gradient_;              //< gradient of the cost function
  Vector_<float> residuals_;       //< vector of residuals
  Solver solver_;                  //< the linear solver
  int interp_;                     //< interpolation, e.g. cv::INTER_LINEAR

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}; // Tracker

NAMESPACE_END

