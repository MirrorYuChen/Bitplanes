/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 15:09
 * @Description: Types
 * @FilePath: Marker//Types.cc
 */
#include "Types.h"
#include <iostream>

NAMESPACE_BEGIN
std::string ToString(OptimizerStatus status) {
  std::string s;
  switch (status) {
    case OptimizerStatus::NotStarted:
      s = "NotStarted";
      break;
    case OptimizerStatus::MaxIterations:
      s = "MaxIterations";
      break;
    case OptimizerStatus::FirstOrderOptimality:
      s = "FirstOrderOptimality";
      break;
    case OptimizerStatus::SmallRelativeReduction:
      s = "SmallRelativeReduction";
      break;
    case OptimizerStatus::SmallAbsError:
      s = "SmallAbsError";
      break;
    case OptimizerStatus::SmallParameterUpdate:
      s = "SmallParameterUpdate";
      break;
    case OptimizerStatus::SmallAbsParameters:
      s = "SmallAbsParameters";
      break;
  }

  return s;
}

std::ostream &operator<<(std::ostream &os, const Result &r) {
  os << "OptimizerStatus: " << ToString(r.status) << "\n";
  os << "NumIterations: " << r.num_iterations << "\n";
  os << "FinalSsdError: " << r.final_ssd_error << "\n";
  os << "FirstOrderOptimality: " << r.first_order_optimality << "\n";
  os << "TimeMilliSeconds: " << r.time_ms << "\n";
  os << "T:\n" << r.T;
  return os;
}

std::string ToString(MotionType m) {
  std::string ret;
  switch (m) {
    case MotionType::Translation:
      ret = "Translation";
      break;
    case MotionType::Affine:
      ret = "Affine";
      break;
    case MotionType::Homography:
      ret = "Homography";
      break;
  }
  return ret;
}

NAMESPACE_END
