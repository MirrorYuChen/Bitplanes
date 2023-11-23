/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 15:23
 * @Description: Motion Model
 * @FilePath: Marker//MotionModel.cc
 */
#include "MotionModel.h"

#include <unsupported/Eigen/MatrixFunctions> // for exp and log
#include <Eigen/Cholesky>

NAMESPACE_BEGIN
auto Homography::Scale(const Transform &T, float scale) -> Transform {
  Transform S(Transform::Identity()), S_i(Transform::Identity());
  S(0, 0) = scale;
  S_i(0, 0) = 1.0f / scale;
  S(1, 1) = scale;
  S_i(1, 1) = 1.0f / scale;
  return S * T * S_i;
}

auto Homography::MatrixToParams(const Transform &H) -> ParameterVector {
  Transform L = H.log();
  ParameterVector p;
  p[0] = L(0, 2);
  p[1] = L(1, 2);
  p[2] = -L(1, 0);
  p[3] = -3 / 2.0 * L(2, 2);
  p[4] = L(0, 0) + 1 / 2.0 * L(2, 2);
  p[5] = L(1, 0) + L(0, 1);
  p[6] = L(2, 0);
  p[7] = L(2, 1);

  return p;
}

auto Homography::ParamsToMatrix(const ParameterVector &p) -> Transform {
  Transform H;
  H <<
    p[3] / 3 + p[4], p[2] + p[5], p[0],
    -p[2], p[3] / 3 - p[4], p[1],
    p[6], p[7], -2 * p[3] / 3;

  return H.exp();
}

auto Homography::Solve(const Hessian &A, const Gradient &b) -> ParameterVector {
  return -A.ldlt().solve(b);
}

auto Homography::ComputeJacobian(float x, float y, float Ix, float Iy,
                                 float s, float c1, float c2) -> Jacobian {
  Jacobian J;
  J <<
    Ix / s,
    Iy / s,
    Iy * (c1 - x) - Ix * (c2 - y),
    -Ix * (c1 - x) - Iy * (c2 - y),
    Iy * (c2 - y) - Ix * (c1 - x),
    -Ix * (c2 - y),
    -Ix * s * sq(c1 - x) - Iy * s * (c1 - x) * (c2 - y),
    -Iy * s * sq(c2 - y) - Ix * s * (c1 - x) * (c2 - y);

  return J;
}

auto Homography::ComputeWarpJacobian(float x, float y, float s, float c1, float c2)
-> WarpJacobian {
  WarpJacobian Jw;
  Jw <<
     1 / s, 0, y - c2, x - c1, x - c1, y - c2, -s * sq(c1 - x), -s * (c1 - x) * (c2 - y),
    0, 1 / s, c1 - x, y - c2, c2 - y, 0, -s * (c1 - x) * (c2 - y), -s * sq(c2 - y);

  return Jw;
}

NAMESPACE_END
