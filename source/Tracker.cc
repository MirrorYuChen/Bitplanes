/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 17:21
 * @Description: Tracker
 * @FilePath: Bitplanes/source/Tracker.cc
 */
#include "Tracker.h"
#include "MotionModel.h"
#include "Timer.h"

#include <Eigen/LU>

NAMESPACE_BEGIN
template<class M>
Tracker<M>::Tracker(Parameters p)
  : params_(p), cdata_(p.subsampling), T_(Matrix33f::Identity()), T_inv_(Matrix33f::Identity()),
    interp_(cv::INTER_LINEAR) {}

template<class M>
void Tracker<M>::setTemplate(const cv::Mat &image, const cv::Rect &bbox) {
  // 1.设置模板图像，并进行高斯模糊
  image.copyTo(I_);
  SmoothImage(I_, bbox);

  // 2.将矩阵设置为单位阵
  cdata_.getNormedCoordinate(bbox, T_, T_inv_);

  // 3.保存ROI位置
  bbox_ = bbox;

  // 4.设置采样数据：ROI对应LBP特征的梯度对应海塞矩阵
  cdata_.set(I_, bbox, T_(0, 0), T_inv_(0, 2), T_inv_(1, 2));

  // 5.对海塞矩阵进行LDLT分解
  solver_.compute(-cdata_.hessian());
}

template<class M>
Result Tracker<M>::Track(const cv::Mat &image, const Transform &T_init) {
  // 1.保存图像，并进行高斯平滑
  image.copyTo(I_);
  SmoothImage(I_, bbox_);

  // 2.将返回结果设置位初始化位姿矩阵
  Result ret(T_init);
  Timer timer;

  // 3.获取梯度最大值
  auto g_norm = this->Linearize(I_, ret.T);
  const auto p_tol = this->params_.parameter_tolerance,
    f_tol = this->params_.function_tolerance,
    sqrt_eps = std::sqrt(std::numeric_limits<float>::epsilon()),
    tol_opt = 1e-4f * f_tol, rel_factor = std::max(sqrt_eps, g_norm);

  const auto max_iterations = this->params_.max_iterations;
  const auto verbose = this->params_.verbose;

  if (verbose) {
    printf("\n                                        First-Order         Norm of \n"
           " Iteration  Func-count    Residual       optimality            step\n");
    printf(" %5d       %5d   %13.6g    %12.3g\n", 0, 1, residuals_.squaredNorm(), g_norm);
  }

  // 4.若初始位姿态满足误差要求
  if (g_norm < tol_opt * rel_factor) {
    if (verbose) {
      printf("initial value is optimal %g < %g\n", g_norm, tol_opt * rel_factor);
    }

    ret.final_ssd_error = residuals_.squaredNorm();
    ret.first_order_optimality = g_norm;
    ret.time_ms = static_cast<float>(timer.stop().count());
    ret.num_iterations = 1;
    ret.status = OptimizerStatus::FirstOrderOptimality;
    return ret;
  }

  // 5.循环迭代估算位姿
  float old_sum_sq = std::numeric_limits<float>::max();
  bool has_converged = false;
  int it = 1;
  while (!has_converged && it++ < max_iterations) {
    // 5.1 解算位姿
    const ParameterVector dp = solver_.solve(gradient_);
    // 5.2 计算残差
    const auto sum_sq = residuals_.squaredNorm();
    {
      const auto dp_norm = dp.norm();
      const auto p_norm = MotionModelType::MatrixToParams(ret.T).norm();

      if (verbose) {
        printf(" %5d       %5d   %13.6g    %12.3g    %12.6g\n", it, 1 + it, sum_sq, g_norm, dp_norm);
      }

      has_converged = TestConverged(dp_norm, p_norm, p_tol,
                                    g_norm, tol_opt, rel_factor,
                                    sum_sq, old_sum_sq, f_tol,
                                    sqrt_eps, it, max_iterations, verbose,
                                    ret.status);
      old_sum_sq = sum_sq;
    }

    // 5.3 迭代计算
    const Transform Td = T_inv_ * MotionModelType::ParamsToMatrix(dp) * T_;
    ret.T = Td * ret.T;

    if (!has_converged) {
      g_norm = this->Linearize(I_, ret.T);
    }
  }

  // 6.获取解算结果
  ret.time_ms = static_cast<float>(timer.stop().count());
  ret.num_iterations = it;
  ret.final_ssd_error = old_sum_sq;
  ret.first_order_optimality = g_norm;
  if (ret.status == OptimizerStatus::NotStarted) {
    ret.status = OptimizerStatus::MaxIterations;
    if (verbose) {
      std::cout << "Max iterations reached\n";
    }
  }

  if (verbose) {
    printf("\n\n");
  }
  return ret;
}

template<class M>
inline
float Tracker<M>::Linearize(const cv::Mat &I, const Transform &T) {
  // 1.获取T作用于bbox_后，对应ROI区域，并将结果保存到Iw_中
  cdata_.WarpImage(I, T, bbox_, Iw_, interp_, 0.0f);

  // 2.计算当前图像中对应位置LBP描述子残差
  cdata_.ComputeResiduals(Iw_, residuals_);

  // 3.计算梯度：雅可比矩阵乘以残差
  gradient_ = cdata_.jacobian().transpose() * residuals_;

  // 4.使用lpNorm<p>()方法，当模板参数p取特殊值Infinity时，得所有元素最大绝对值
  return gradient_.template lpNorm<Eigen::Infinity>();
}

template<class M>
inline
void Tracker<M>::SmoothImage(cv::Mat &I, const cv::Rect & /*roi*/) {
  // 对输入图像进行高斯平滑
  if (params_.sigma > 0) {
    cv::GaussianBlur(I, I, cv::Size(), params_.sigma);
  }
}

template
class Tracker<Homography>;

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
