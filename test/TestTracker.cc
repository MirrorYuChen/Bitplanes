/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/21 17:40
 * @Description: Test Tracker Pyramid
 * @FilePath: Bitplanes/test/TestBitplanesTrackerPyramid.cc
 */
#include "MotionModel.h"
#include "Timer.h"
#include "Tracker.h"
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
  auto x1 = static_cast<float>(r.x);
  auto y1 = static_cast<float>(r.y);
  auto x2 = static_cast<float>(r.x + r.width);
  auto y2 = static_cast<float>(r.y + r.height);

  Eigen::Vector3f p;
  p = H * Eigen::Vector3f(x1, y1, 1.0);
  p /= p[2];
  ret[0] = cv::Point2f(p.x(), p.y());

  p = H * Eigen::Vector3f(x2, y1, 1.0);
  p /= p[2];
  ret[1] = cv::Point2f(p.x(), p.y());

  p = H * Eigen::Vector3f(x2, y2, 1.0);
  p /= p[2];
  ret[2] = cv::Point2f(p.x(), p.y());

  p = H * Eigen::Vector3f(x1, y2, 1.0);
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

static bool g_ldown = false;
static bool g_lup = false;
static cv::Point g_corner1;
static cv::Point g_corner2;
static cv::Mat g_cur_frame;
static std::vector<cv::Rect> g_boxes;

static void mouse_callback(int event, int x, int y, int, void*)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    g_ldown = true;
    g_corner1.x = x;
    g_corner1.y = y;
  }
  if (event == cv::EVENT_LBUTTONUP)
  {
    if (abs(x - g_corner1.x) > 20 && abs(y - g_corner1.y) > 20)
    {
      g_lup = true;
      g_corner2.x = x;
      g_corner2.y = y;
      //std::cout << "Corner 2 recorded at " << corner2 << std::endl << std::endl;
    }
    else
    {
      std::cout << "Please select a bigger region" << std::endl;
      g_ldown = false;
    }
  }
  if (g_ldown == true && g_lup == false)
  {
    cv::Point pt;
    pt.x = x;
    pt.y = y;

    cv::Mat local_img = g_cur_frame.clone();
    rectangle(local_img, g_corner1, pt, cv::Scalar(0, 0, 255));

    for (int i = 0; i < g_boxes.size(); ++i)
    {
      rectangle(local_img, g_boxes[i].tl(), g_boxes[i].br(), cv::Scalar(255, 0, 0));
    }

    cv::imshow("cur_frame", local_img);
  }

  if (g_ldown == true && g_lup == true)
  {
    cv::Rect box;
    box.width = abs(g_corner1.x - g_corner2.x);
    box.height = abs(g_corner1.y - g_corner2.y);
    box.x = std::min(g_corner1.x, g_corner2.x);
    box.y = std::min(g_corner1.y, g_corner2.y);
    g_boxes.push_back(box);

    g_ldown = false;
    g_lup = false;
  }
}

void user_get_box(const cv::Mat& frame)
{
  g_cur_frame = frame.clone();

  cv::imshow("cur_frame", frame);
  cv::setMouseCallback("cur_frame", mouse_callback);
  std::cout << "Please select some regions you want to track and press 'q'" << std::endl;
  while (char(cv::waitKey(1)) != 'q' && char(cv::waitKey(1)) != 'Q') {}
  cv::destroyWindow("cur_frame");
}

using namespace NAMESPACE;

int main() {
  const auto images = LoadData();

  // 1.Initialize the Tracker
  Parameters params;
  params.num_levels = 3;
  params.max_iterations = 50;
  params.parameter_tolerance = 1e-5;
  params.function_tolerance = 1e-4;
  params.verbose = false;
  PyramidTracker<Homography> tracker(params);

  // 2.Initialize the init pose
  Matrix33f H(Matrix33f::Identity());
  cv::VideoCapture cam("../data/videos/test.mp4");
  if (!cam.isOpened()) {
    std::cout << "Failed open camera." << std::endl;
    return -1;
  }
  int frame_count = 0;
  double time_cost = 0.0;
  cv::Mat frame;
  while (true) {
    cam >> frame;
    if (frame.empty()) {
      break;
    }
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    static bool first_flag = true;
    if (first_flag) {
      user_get_box(frame);
      if (!g_boxes.empty()) {
        first_flag = false;
        tracker.setTemplate(frame, g_boxes[0]);
      }
    } else {
      Timer timer;
      auto result = tracker.Track(frame, H);
      time_cost += static_cast<double>(timer.stop().count());
      H = result.T;
      DrawTrackingResult(frame, frame, g_boxes[0], H.data());
    }
    cv::imshow("result", frame);
    int k = 0xff & cv::waitKey(5);
    if (k == 'q') {
      break;
    }
    frame_count++;
  }
  cam.release();
  std::cout << "Runtime: " << static_cast<float>(frame_count) / (time_cost / 1000.0f) << " HZ.\n";
  return 0;
}