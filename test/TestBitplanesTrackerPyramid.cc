/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 17:40
 * @Description: Test Tracker Pyramid
 * @FilePath: Bitplanes/test/TestBitplanesTrackerPyramid.cc
 */
#include "MotionModel.h"
#include "Timer.h"
#include "PyramidTracker.h"
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <cassert>

std::vector<cv::Mat> LoadData() {
  static const char *DATA_DIR = "../data/";

  std::vector<cv::Mat> ret(50);
  for (int i = 0; i < 50; ++i) {
    char fn[128];
    snprintf(fn, sizeof(fn) - 1, "%s/%05d.png", DATA_DIR, i);
    ret[i] = cv::imread(fn, cv::IMREAD_GRAYSCALE);
    assert(!ret[i].empty());
  }

  return ret;
}

/**
 * returns the tranformed corners of the bounding box
 */
static inline
std::array<cv::Point2f, 4> RectToPoints(const cv::Rect &r, const float *H_ptr) {
  std::array<cv::Point2f, 4> ret;
  const Eigen::Matrix3f H = Eigen::Matrix3f::Map(H_ptr);

  Eigen::Vector3f p;
  p = H * Eigen::Vector3f(r.x, r.y, 1.0);
  p /= p[2];
  ret[0] = cv::Point2f(p.x(), p.y());

  p = H * Eigen::Vector3f(r.x + r.width, r.y, 1.0);
  p /= p[2];
  ret[1] = cv::Point2f(p.x(), p.y());

  p = H * Eigen::Vector3f(r.x + r.width, r.y + r.height, 1.0);
  p /= p[2];
  ret[2] = cv::Point2f(p.x(), p.y());

  p = H * Eigen::Vector3f(r.x, r.y + r.height, 1.0);
  p /= p[2];
  ret[3] = cv::Point2f(p.x(), p.y());

  return ret;
}

void DrawTrackingResult(cv::Mat &dst, const cv::Mat &src, const cv::Rect &r,
                        const float *H, int thickness = 4, int type = 16,
                        int shift = 0) {
  if (src.channels() == 1)
    cv::cvtColor(src, dst, CV_GRAY2BGRA);
  else
    src.copyTo(dst);

  const auto cv_clr = cv::Scalar(0, 0, 255, 128);
  const auto x = RectToPoints(r, H);

  cv::line(dst, x[0], x[1], cv_clr, thickness, type, shift);
  cv::line(dst, x[1], x[2], cv_clr, thickness, type, shift);
  cv::line(dst, x[2], x[3], cv_clr, thickness, type, shift);
  cv::line(dst, x[3], x[0], cv_clr, thickness, type, shift);
  cv::line(dst, x[0], x[2], cv_clr, thickness, type, shift);
  cv::line(dst, x[1], x[3], cv_clr, thickness, type, shift);
}

using namespace NAMESPACE;

int main() {
  const auto images = LoadData();

  Parameters params;
  params.num_levels = 3;
  params.max_iterations = 50;
  params.parameter_tolerance = 1e-5;
  params.function_tolerance = 1e-4;
  params.verbose = false;

  cv::Rect bbox = cv::Rect(120, 110, 300, 230);
  std::cout << bbox << std::endl;

  PyramidTracker<Homography> tracker(params);
  tracker.setTemplate(images[0], bbox);
  cv::Mat dimg;
  Matrix33f H(Matrix33f::Identity());
  double total_time = 0.0;
  for (size_t i = 1; i < images.size(); ++i) {
    Timer timer;
    auto result = tracker.Track(images[i], H);
    total_time += timer.stop().count();
    H = result.T;

    DrawTrackingResult(dimg, images[i], bbox, H.data());
    cv::imshow("bitplanes", dimg);
    int k = 0xff & cv::waitKey(5);
    if (k == 'q') {
      break;
    }
  }
  std::cout << "Runtime: " << images.size() / (total_time / 1000.0f) << " HZ.\n";
  return 0;
}