/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 17:13
 * @Description: Census
 * @FilePath: Bitplanes/source/LBP.h
 */
#pragma once

#include "API.h"
#include "Types.h"
#include <opencv2/opencv.hpp>

NAMESPACE_BEGIN
namespace simd {
/**
 * \param src the source image
 * \param roi region of interset in the source image
 * \param dst destination image
 *
 * The roi must be inside the image with at least 1 pixel off the border
 */
void LBP(const cv::Mat &src, const cv::Rect &roi, cv::Mat &dst);

} // namespace simd

NAMESPACE_END
