/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 16:27
 * @Description: channel data sampled
 * @FilePath: Bitplanes/source/BitplanesChannelDataSampled.h
 */
#pragma once

#include "API.h"
#include "Types.h"
#include <opencv2/opencv.hpp>

NAMESPACE_BEGIN
template<class>
struct ChannelDataTraits;

template<class DerivedType>
class ChannelData {
public:
  typedef DerivedType Derived;
  typedef typename ChannelDataTraits<Derived>::MotionModelType MotionModelType;
  typedef typename ChannelDataTraits<Derived>::Pixels Pixels;
  typedef typename ChannelDataTraits<Derived>::Residuals Residuals;
  typedef typename MotionModelType::WarpJacobian WarpJacobian;
  typedef typename MotionModelType::JacobianMatrix JacobianMatrix;
  typedef typename MotionModelType::Transform Transform;
  typedef typename MotionModelType::Hessian Hessian;
  typedef typename MotionModelType::Gradient Gradient;

public:
  /**
   * sets the template data
   *
   * \param image the input image
   * \param roi   region of interest specifying the template location
   * \param args  normalization to be applied when computing jacobians
   */
  template<class ... Args>
  inline
  void set(const cv::Mat &image, const cv::Rect &roi, Args &...args) {
    return derived()->set(image, roi, args...);
  }

  /**
   * computes the residuals
   *
   * \param warped_image the warped image
   * \param residuals  output residuals
   */
  inline void ComputeResiduals(const cv::Mat &warped_image, Residuals &residuals) const {
    return derived()->ComputeResiduals(warped_image, residuals);
  }

  template<class ... Args>
  inline
  void warpImage(const cv::Mat &src, const Transform &T, const cv::Rect &bbox,
                 cv::Mat &dst, Args &...args) const {
    return derived()->WarpImage(src, T, bbox, dst, args...);
  }

  inline const Pixels &pixels() const { return derived()->pixels(); }

  inline const Hessian &hessian() const { return derived()->hessian(); }

  inline const JacobianMatrix &jacobian() const { return derived()->jacobian(); }

  inline void getNormedCoordinate(const cv::Rect &roi,
                                  Transform &T, Transform &T_inv) const {
    return derived()->getNormedCoordinate(roi, T, T_inv);
  }

private:
  inline const Derived *derived() const { return static_cast<const Derived *>(this); }

  inline Derived *derived() { return static_cast<Derived *>(this); }
};

template<class>
class ChannelDataSampler;

template<class M>
struct ChannelDataTraits<ChannelDataSampler<M> > {
  typedef M MotionModelType;
  typedef Vector_<uint8_t> Pixels;
  typedef Vector_<float> Residuals;
};

template<class M>
class ChannelDataSampler :
  public ChannelData<ChannelDataSampler<M>> {
public:
  typedef ChannelDataSampler<M> Self;
  typedef ChannelData<Self> Base;
  typedef typename Base::MotionModelType MotionModelType;
  typedef typename Base::Pixels Pixels;
  typedef typename Base::Residuals Residuals;
  typedef typename Base::WarpJacobian WarpJacobian;
  typedef typename Base::JacobianMatrix JacobianMatrix;
  typedef typename Base::Hessian Hessian;
  typedef typename Base::Transform Transform;
  typedef typename Base::Gradient Gradient;

public:
  /**
   * \param s subsampling/decimation factor. A value of 1 means no decimation, a
   * value of 2 means decimate by half, and so on
   */
  explicit inline ChannelDataSampler(size_t s = 1)
    : Base(), sub_sampling_(s) {}

  void set(const cv::Mat &, const cv::Rect &roi, float s = 1,
           float c1 = 0, float c2 = 0);

  void ComputeResiduals(const cv::Mat &Iw, Residuals &residuals) const;

  float DoLinearize(const cv::Mat &Iw, Gradient &) const;

  void WarpImage(const cv::Mat &src, const Transform &T, const cv::Rect &roi,
                 cv::Mat &dst, int interp = cv::INTER_LINEAR, float border = 0.0f);

  inline const Pixels &pixels() const { return pixels_; }

  inline const Hessian &hessian() const { return hessian_; }

  inline const JacobianMatrix &jacobian() const { return jacobian_; }

  void getNormedCoordinate(const cv::Rect &, Transform &, Transform &) const;

protected:
  JacobianMatrix jacobian_;
  Pixels pixels_;
  Hessian hessian_;
  int sub_sampling_;
  int roi_stride_;
};

bool TestConverged(float dp_norm, float p_norm, float x_tol, float g_norm,
                   float tol_opt, float rel_factor, float new_f, float old_f,
                   float f_tol, float sqrt_eps, int it, int max_iters, bool verbose,
                   OptimizerStatus &status);

NAMESPACE_END

