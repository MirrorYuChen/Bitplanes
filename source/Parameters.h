/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 14:40
 * @Description: Parameters
 * @FilePath: Marker//Parameters.h
 */
#pragma once

#include "API.h"
#include <iosfwd>
#include <string>

NAMESPACE_BEGIN
class Parameters {
public:
  /**
`  * minimum pixels to attempt alignment. Used for auto pyramid levels
   */
  static constexpr const int MIN_NUM_PIXELS_TO_WORK = 100 * 100 / 16.0f;

  /**
   * multi-channel extraction function type
   */
  enum class MultiChannelExtractorType {
    IntensityGrayChannel, //< single channel grayscale
    GradientAbsMag,       //< single channel gradient absolute magnitude
    IntensityAndGradient, //< 2 channels, intensity + gradient constraint
    CensusChannel,        //< single channel LBP signature
    DescriptorFields1,    //< 1-st order descriptor fields
    DescriptorFields2,    //< 2-nd order descriptor fields
    BitPlanes             //< BitPlanes (8-channels)
  };

  enum class LinearizerType {
    InverseCompositional, //< IC algorithm
    ForwardCompositional, //< FC algorithm
  };

  /**
   * Type of the motion to estimate
   */
  enum class MotionType {
    Translation,
    Affine,
    Homography
  };

  /**
   * number of pyramid levels. A negative value means 'Auto'
   * A value of 1 means a single level (no pyramid)
   */
  int num_levels = -1;

  /**
   * maximum number of iterations
   */
  int max_iterations = 50;

  /**
   * parameter tolerance. If the relative magnitude of parameters falls belows
   * this we converge
   */
  float parameter_tolerance = 5e-6;

  /**
   * function value tolerance. If the the relative function value falls below
   * this, we converge
   */
  float function_tolerance = 5e-5;

  /**
   * std. deviation of an isotropic Gaussian to pre-smooth images prior to
   * computing the channels
   */
  float sigma = 1.2f;

  /**
   * print information
   */
  bool verbose = true;

  /**
   * Process the template by skipping every nth pixel.
   *
   * For example,
   * if 'subsampling' == 2, then we process the template by every other pixel
   * If 'subsampling' == 1, then all pixels will be processed
   *
   */
  int subsampling = 1;

  /**
   * Multi-channel function to use
   *
   * Currently, this is the only one supported. NOTE: other channels have been
   * removed to focus on BitPlanes for iOS
   */
  MultiChannelExtractorType multi_channel_function =
    MultiChannelExtractorType::BitPlanes;

  /**
   * linearization algorithm
   *
   * NOTE: this is the only linearizer supported. In the future, we will add
   * more linearizers (e.g. ForwardCompositional, and ESM)
   */
  LinearizerType linearizer = LinearizerType::InverseCompositional;

  friend std::ostream &operator<<(std::ostream&, const Parameters& p);
};

NAMESPACE_END
