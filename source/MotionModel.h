/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 15:15
 * @Description: Motion Model
 * @FilePath: Marker//MotionModel.h
 */
#pragma once

#include "API.h"
#include "Types.h"

NAMESPACE_BEGIN
class Homography;

template<class>
struct MotionModelTraits;

template<>
struct MotionModelTraits<Homography> {
  static constexpr int DOF = 8;
  typedef Eigen::Matrix<float, 3, 3> Transform;
  typedef Eigen::Matrix<float, DOF, DOF> Hessian;
  typedef Eigen::Matrix<float, DOF, 1> ParameterVector;
  typedef Eigen::Matrix<float, 1, DOF> Jacobian;
  typedef ParameterVector Gradient;
  typedef Eigen::Matrix<float, Dynamic, DOF> JacobianMatrix;
  typedef Eigen::Matrix<float, 2, DOF> WarpJacobian;
};

template<class Derived>
class MotionModel {
public:
  typedef MotionModelTraits<Derived> traits;
  typedef typename traits::Transform Transform;
  typedef typename traits::Hessian Hessian;
  typedef typename traits::Gradient Gradient;
  typedef typename traits::Jacobian Jacobian;
  typedef typename traits::JacobianMatrix JacobianMatrix;
  typedef typename traits::ParameterVector ParameterVector;
  typedef typename traits::WarpJacobian WarpJacobian;
  static constexpr int DOF = traits::DOF;

public:
  static inline Transform Scale(const Transform &T, float s) {
    return Derived::Scale(T, s);
  }

  static inline Transform ParamsToMatrix(const ParameterVector &p) {
    return Derived::ParamsToMatrix(p);
  }

  static inline ParameterVector MatrixToParams(const Transform &p) {
    return Derived::MatrixToParams(p);
  }

  static inline ParameterVector Solve(const Hessian &H, const Gradient &g) {
    return Derived::Solve(H, g);
  }

  template<class ... Args>
  static inline
  Jacobian ComputeJacobian(float x, float y, float Ix, float Iy, Args &... args) {
    return Derived::ComputeJacobian(x, y, Ix, Iy, args...);
  }

  template<class ... Args>
  static inline
  void ComputeJacobian(Eigen::Ref<Jacobian> J, float x, float y, float Ix, float Iy,
                       Args &... args) {
    Derived::ComputeJacobian(J, x, y, Ix, Iy, args...);
  }

  template<class ... Args>
  static inline
  WarpJacobian ComputeWarpJacobian(float x, float y, Args &... args) {
    return Derived::ComputeWarpJacobian(x, y, args...);
  }

  template<class ... Args>
  static inline
  void ComputeWarpJacobian(Eigen::Ref<WarpJacobian> Jw, float x, float y,
                           Args &... args) {
    Derived::ComputeWarpJacobian(Jw, x, y, args...);
  }

protected:
  inline const Derived *derived() const { return static_cast<const Derived *>(this); }

  inline Derived *derived() { return static_cast<Derived *>(this); }
};

class Homography : public MotionModel<Homography> {
public:
  typedef MotionModel<Homography> Base;
  typedef typename Base::Transform Transform;
  typedef typename Base::Hessian Hessian;
  typedef typename Base::Gradient Gradient;
  typedef typename Base::Jacobian Jacobian;
  typedef typename Base::JacobianMatrix JacobianMatrix;
  typedef typename Base::ParameterVector ParameterVector;
  typedef typename Base::WarpJacobian WarpJacobian;

public:
  /**
   * scale the transform with the given value
   */
  static Transform Scale(const Transform &, float);

  /**
   * convert the parameter vector to a matrix
   */
  static Transform ParamsToMatrix(const ParameterVector &);

  /**
   * convert the matrix to a parameter vector
   */
  static ParameterVector MatrixToParams(const Transform &);

  /**
   * solve the linear system
   */
  static ParameterVector Solve(const Hessian &, const Gradient &);

  static Jacobian
  ComputeJacobian(float x, float y, float Ix, float Iy,
                  float s = 1.0f, float c1 = 0.0f, float c2 = 0.0f);

  static inline void
  ComputeJacobian(Eigen::Ref<Jacobian> J, float x, float y,
                  float Ix, float Iy, float s = 1.0f,
                  float c1 = 0.0f, float c2 = 0.0f) {
    J = Homography::ComputeJacobian(x, y, Ix, Iy, s, c1, c2);
  }

  static WarpJacobian
  ComputeWarpJacobian(float x, float y, float s = 1.0,
                      float c1 = 0.0, float c2 = 0.0);

  static inline void
  ComputeWarpJacobian(Eigen::Ref<WarpJacobian> Jw, float x, float y,
                      float s = 1.0f, float c1 = 0.0f, float c2 = 0.0f) {
    Jw = Homography::ComputeWarpJacobian(x, y, s, c1, c2);
  }
};

NAMESPACE_END
