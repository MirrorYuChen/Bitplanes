/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 16:36
 * @Description: Bitplanes Channel Data Sampled
 * @FilePath: Bitplanes/source/BitplanesChannelDataSampled.cc
 */
#include "ChannelDataSampler.h"
#include "MotionModel.h"
#include "LBP.h"
#include <cassert>
#include <cmath>
#include <stdio.h>

NAMESPACE_BEGIN
static inline int getNumValid(const cv::Rect &roi, int s) {
  int ret = 0;
  for (int y = 1; y < roi.height - 1; y += s) {
    for (int x = 1; x < roi.width - 1; x += s) {
      ++ret;
    }
  }
  return ret;
}

template<class M>
void ChannelDataSampler<M>::set(const cv::Mat &src, const cv::Rect &roi, float s, float c1, float c2) {
  assert(roi.x >= 1 || roi.x <= src.cols - 1 || roi.y >= 1 || roi.y <= src.rows - 1);
  assert(s > 0);

  // 1.计算采样后，有效像素点数目
  auto n_valid = getNumValid(roi, sub_sampling_);

  // 2.像素点数组和对应雅可比矩阵数组
  pixels_.resize(n_valid);
  jacobian_.resize(8 * n_valid, M::DOF);

  // 3.计算ROI对应每个像素的LBP特征
  cv::Mat lbp;
  simd::LBP(src, roi, lbp);
  int stride = lbp.cols;

  /**
   * compute the channel x- and y-gradient at col:=x for bit:=b
   */
  // 4.计算每个像素在X，Y方向梯度：按位取出
  auto G = [=](const uint8_t *p, int x, int b) {
    auto ix1 = static_cast<float>((p[x + 1] & (1 << b)) >> b), ix2 = static_cast<float>((p[x - 1] & (1 << b)) >> b),
      iy1 = static_cast<float>((p[x + stride] & (1 << b)) >> b), iy2 = static_cast<float>((p[x - stride] & (1 << b)) >> b);
    return Eigen::Matrix<float, 1, 2>(0.5f * (ix1 - ix2), 0.5f * (iy1 - iy2));
  };

  // 5.计算雅可比矩阵
  typename M::WarpJacobian Jw;
  auto *pixels_ptr = pixels_.data();
  for (int y = 1, j = 0, i = 0; y < lbp.rows - 1; y += sub_sampling_) {
    const auto *s_row = lbp.ptr<const uint8_t>(y);
    for (int x = 1; x < lbp.cols - 1; x += sub_sampling_, i += 8, ++j) {
      Jw = M::ComputeWarpJacobian(x + roi.x, y + roi.y, s, c1, c2);
      pixels_ptr[j] = s_row[x];
      jacobian_.row(i + 0) = G(s_row, x, 0) * Jw;
      jacobian_.row(i + 1) = G(s_row, x, 1) * Jw;
      jacobian_.row(i + 2) = G(s_row, x, 2) * Jw;
      jacobian_.row(i + 3) = G(s_row, x, 3) * Jw;
      jacobian_.row(i + 4) = G(s_row, x, 4) * Jw;
      jacobian_.row(i + 5) = G(s_row, x, 5) * Jw;
      jacobian_.row(i + 6) = G(s_row, x, 6) * Jw;
      jacobian_.row(i + 7) = G(s_row, x, 7) * Jw;
    }
  }

  // 6.计算海塞矩阵
  hessian_ = jacobian_.transpose() * jacobian_;
  roi_stride_ = roi.width;
}

template<class M>
void ChannelDataSampler<M>::ComputeResiduals(const cv::Mat &Iw, Residuals &residuals) const {
  typedef int8_t CType;
  cv::AutoBuffer<CType> buf(8 * pixels_.size());
  CType *r_ptr = buf;

  // 1.计算LBP描述子之间残差
  const uint8_t *c0_ptr = pixels_.data();
  const int src_stride = Iw.cols;
  for (int y = 1; y < Iw.rows - 1; y += sub_sampling_) {
    const auto *s_row = Iw.ptr<const uint8_t>(y);

#pragma omp simd
    for (int x = 1; x < Iw.cols - 1; x += sub_sampling_) {
      const uint8_t *p = s_row + x;
      const uint8_t c = *c0_ptr++;
      *r_ptr++ = (*(p - src_stride - 1) >= *p) - ((c & (1 << 0)) >> 0);
      *r_ptr++ = (*(p - src_stride) >= *p) - ((c & (1 << 1)) >> 1);
      *r_ptr++ = (*(p - src_stride + 1) >= *p) - ((c & (1 << 2)) >> 2);
      *r_ptr++ = (*(p - 1) >= *p) - ((c & (1 << 3)) >> 3);
      *r_ptr++ = (*(p + 1) >= *p) - ((c & (1 << 4)) >> 4);
      *r_ptr++ = (*(p + src_stride - 1) >= *p) - ((c & (1 << 5)) >> 5);
      *r_ptr++ = (*(p + src_stride) >= *p) - ((c & (1 << 6)) >> 6);
      *r_ptr++ = (*(p + src_stride + 1) >= *p) - ((c & (1 << 7)) >> 7);
    }
  }

  // 2.将残差返回
  using namespace Eigen;
  residuals = Map<Vector_<CType>, Aligned>(buf, pixels_.size() * 8, 1).template cast<float>();
}

template<class M>
float ChannelDataSampler<M>::DoLinearize(const cv::Mat &Iw, Gradient &g) const {
  g.setZero();
  float ret = 0.0f;

  const uint8_t *c0_ptr = pixels_.data();
  const int src_stride = Iw.cols;

  for (int y = 1, i = 0; y < Iw.rows - 1; y += sub_sampling_) {
    const auto srow = Iw.ptr<const uint8_t>(y);
    for (int x = 1; x < Iw.cols - 1; x += sub_sampling_, i += 8) {
      const auto *p = srow + x;
      const auto *p0 = p - src_stride;
      const auto *p1 = p + src_stride;

      const auto c = *c0_ptr++;

      Eigen::Matrix<float, 8, 1> err;
      err[0] = (p0[-1] > *p) - ((c & (1 << 0)) >> 0);
      err[1] = (p0[0] > *p) - ((c & (1 << 1)) >> 1);
      err[2] = (p0[1] > *p) - ((c & (1 << 2)) >> 2);
      err[3] = (p[-1] > *p) - ((c & (1 << 3)) >> 3);
      err[4] = (p[0] > *p) - ((c & (1 << 4)) >> 4);
      err[5] = (p1[-1] > *p) - ((c & (1 << 5)) >> 5);
      err[6] = (p1[0] > *p) - ((c & (1 << 6)) >> 6);
      err[7] = (p1[1] > *p) - ((c & (1 << 7)) >> 7);

      g.noalias() += jacobian_.template block<8, 8>(i, 0) * err;
      ret += err.squaredNorm();
    }
  }

  return ret;
}

template<class Derived>
static inline
Eigen::Matrix<typename Derived::PlainObject::Scalar,
  Derived::PlainObject::RowsAtCompileTime, Derived::PlainObject::ColsAtCompileTime>
NormalizeHomography(const Eigen::MatrixBase<Derived> &x) {
  static_assert(Derived::PlainObject::RowsAtCompileTime != Eigen::Dynamic &&
                Derived::PlainObject::ColsAtCompileTime == 1,
                "normHomog: input must be a vector of known dimension");

  return x * (1.0f / x[Derived::PlainObject::RowsAtCompileTime - 1]);
}


template<class M>
void ChannelDataSampler<M>::WarpImage(const cv::Mat &src, const Transform &T, const cv::Rect &roi,
          cv::Mat &dst, int interp, float border) {
  // 1.X和Y轴方向映射
  cv::Mat x_map(roi.size(), CV_32FC1);
  cv::Mat y_map(roi.size(), CV_32FC1);

  assert(!x_map.empty() && !y_map.empty());

  using namespace Eigen;

  // 2.获取单应矩阵作用后的ROI区域对应像素坐标
  const int x_s = roi.x, y_s = roi.y;
  for (int y = 0; y < roi.height; ++y) {
    auto *xm_ptr = x_map.ptr<float>(y);
    auto *ym_ptr = y_map.ptr<float>(y);

    int yy = y + y_s;

    for (int x = 0; x < roi.width; ++x) {
      // 将T作用在原始坐标上，并归一化得到当前帧中坐标
      const Vector3f pw = NormalizeHomography(T * Vector3f(static_cast<float>(x + x_s), static_cast<float>(yy), 1.0f));
      xm_ptr[x] = pw[0];
      ym_ptr[x] = pw[1];
    }
  }

  // 3.获取单应矩阵作用后的模板区域图像
  cv::remap(src, dst, x_map, y_map, interp, cv::BORDER_CONSTANT, cv::Scalar(border));
}

template<class M>
void ChannelDataSampler<M>::getNormedCoordinate(const cv::Rect & /*roi*/, Transform &T, Transform &T_inv) const {
  T.setIdentity();
  T_inv.setIdentity();
}

template<>
void ChannelDataSampler<Homography>::getNormedCoordinate(const cv::Rect &roi, Transform &T, Transform &T_inv) const {
  Vector2f c(0, 0);

  int n_valid = 0;
  for (int y = 1; y < roi.height - 1; y += sub_sampling_)
    for (int x = 1; x < roi.width - 1; x += sub_sampling_, ++n_valid)
      c += Vector2f(x + roi.x, y + roi.y);
  c /= n_valid;

  float m = 0.0f;
  for (int y = 1; y < roi.height - 1; y += sub_sampling_)
    for (int x = 1; x < roi.width - 1; x += sub_sampling_)
      m += (Vector2f(x + roi.x, y + roi.y) - c).norm();
  m /= n_valid;

  float s = sqrt(2.0f) / std::max(m, 1e-6f);

  T << s, 0, -s * c[0],
    0, s, -s * c[1],
    0, 0, 1;

  T_inv << 1.0f / s, 0, c[0],
    0, 1.0f / s, c[1],
    0, 0, 1;
}

template
class ChannelDataSampler<Homography>;

bool TestConverged(float dp_norm, float p_norm, float x_tol, float g_norm,
                   float tol_opt, float rel_factor, float new_f, float old_f,
                   float f_tol, float sqrt_eps, int it, int max_iters, bool verbose,
                   OptimizerStatus &status) {
  if (it > max_iters) {
    if (verbose) {
      printf("MaxIterations reached\n");
    }
    return true;
  }

  if (g_norm < tol_opt * rel_factor) {
    if (verbose) {
      printf("First order optimality reached [%g < %g]\n", g_norm, tol_opt * rel_factor);
    }

    status = OptimizerStatus::FirstOrderOptimality;
    return true;
  }

  if (dp_norm < x_tol) {
    if (verbose) {
      printf("Small abs step [%g < %g]\n", dp_norm, x_tol);
    }
    status = OptimizerStatus::SmallAbsParameters;
    return true;
  }

  if (dp_norm < x_tol * (sqrt_eps * p_norm)) {
    if (verbose) {
      printf("Small change in parameters [%g < %g]\n", dp_norm, x_tol * (sqrt_eps * p_norm));
    }
    status = OptimizerStatus::SmallParameterUpdate;
    return true;
  }

  if (true || dp_norm < 5e-5) {
    // do not converge based on function value if parameters are not small enough
    if (std::fabs(old_f - new_f) < f_tol * old_f) {
      if (verbose) {
        printf("Small relative reduction in error [%g < %g]\n", std::fabs(old_f - new_f), f_tol * old_f);
      }
      status = OptimizerStatus::SmallRelativeReduction;
      return true;
    }
  }

  return false;
}
NAMESPACE_END
