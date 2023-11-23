/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 14:48
 * @Description: Parameters
 * @FilePath: Marker//Parameters.cpp
 */
#include "Parameters.h"
#include <iostream>

NAMESPACE_BEGIN
std::string ToString(Parameters::MultiChannelExtractorType m) {
  std::string ret;
  switch (m) {
    case Parameters::MultiChannelExtractorType::IntensityGrayChannel:
      ret = "IntensityGrayChannel";
      break;
    case Parameters::MultiChannelExtractorType::GradientAbsMag:
      ret = "GradientAbsMag";
      break;
    case Parameters::MultiChannelExtractorType::IntensityAndGradient:
      ret = "IntensityAndGradient";
      break;
    case Parameters::MultiChannelExtractorType::CensusChannel:
      ret = "CensusChannel";
      break;
    case Parameters::MultiChannelExtractorType::DescriptorFields1:
      ret = "DescriptorFields1";
      break;
    case Parameters::MultiChannelExtractorType::DescriptorFields2:
      ret = "DescriptorFields2";
      break;
    case Parameters::MultiChannelExtractorType::BitPlanes:
      ret = "BitPlanes";
      break;
  }
  return ret;
}

std::ostream &operator<<(std::ostream &os, const Parameters &p) {
  os << "MultiChannelFunction = " << ToString(p.multi_channel_function) << "\n";
  os << "ParameterTolerance = " << p.parameter_tolerance << "\n";
  os << "FunctionTolerance = " << p.function_tolerance << "\n";
  os << "NumLevels = " << p.num_levels << "\n";
  os << "sigma = " << p.sigma << "\n";
  os << "verbose = " << p.verbose << "\n";
  os << "subsampling = " << p.subsampling;
  return os;
}
NAMESPACE_END

