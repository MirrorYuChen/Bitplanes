/*
 * @Author: chenjingyu
 * @Contact: 2458006366@qq.com
 * @Date: 2023/11/24 15:20
 * @Description: Demo
 * @FilePath: Practice/test/Demo.cc
 */
#include "Demo.h"

#include "MotionModel.h"
#include "Tracker.h"
#include "BoundedBuffer.h"

#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>


typedef std::unique_ptr<cv::Mat> ImagePtr;

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

typedef struct ResultForDisplay {
  ImagePtr image;
  NAMESPACE::Result tracker_result;
  int time_ms;
} ResultForDisplay;

typedef struct GuiData {
  ImagePtr image;
  ImagePtr gray;
  NAMESPACE::Result result;

  GuiData() : image(new cv::Mat), gray(new cv::Mat) {}

  void swap(GuiData &&other) {
    image.swap(other.image);
    gray.swap(other.gray);
    std::swap(result, other.result);
  }
} GuiData;

typedef NAMESPACE::BoundedBuffer<std::unique_ptr<GuiData>> ImageBufferType;

struct DemoLiveCapture::Impl {
  typedef NAMESPACE::PyramidTracker<NAMESPACE::Homography> TrackerType;
  Impl() : cap_() {
    if (!cap_.isOpened()) {
      cap_.open(0);
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920 / 2);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080 / 2);
    }

    NAMESPACE::Parameters params;
    params.num_levels = 2;
    params.max_iterations = 50;
    params.subsampling = 2;
    params.verbose = false;

    tracker_.reset(new TrackerType(params));
    main_thread_.reset(new std::thread(&DemoLiveCapture::Impl::mainThread, this));
  }

  ~Impl() {
    stop_requested_ = true;

    if (main_thread_ && main_thread_->joinable())
      main_thread_->join();

    if (display_thread_ && display_thread_->joinable())
      display_thread_->join();

    if (data_thread_ && data_thread_->joinable())
      data_thread_->join();
  }

  inline bool isRunning() const { return stop_requested_ == false; }

  inline void stop() { stop_requested_ = true; }

  cv::VideoCapture cap_;

  std::atomic<bool> stop_requested_{false};
  std::unique_ptr<TrackerType> tracker_;
  cv::Rect roi_;

  std::unique_ptr<std::thread> main_thread_;
  std::unique_ptr<std::thread> display_thread_;
  std::unique_ptr<std::thread> data_thread_;

  std::unique_ptr<ImageBufferType> data_buffer_;
  std::unique_ptr<ImageBufferType> results_buffer_;

  void displayThread();

  void mainThread();

  void dataThread();
}; // DemoLiveCapture::Impl

DemoLiveCapture::DemoLiveCapture()
  : impl_(new Impl()) {}

DemoLiveCapture::~DemoLiveCapture() {
  delete impl_;
}

bool DemoLiveCapture::isRunning() const {
  return impl_->isRunning();
}

void DemoLiveCapture::stop() { impl_->stop(); }

struct MouseHandleData {
  std::atomic<bool> start_selection{false};
  std::atomic<bool> has_template{false};
  cv::Point origin;
  cv::Rect roi;
}; // MouseHandleData

void onMouse(int event, int x, int y, int /*flags*/, void *data_) {
  auto *data = reinterpret_cast<MouseHandleData *>(data_);
  assert(data != NULL);

  if (data->start_selection) {
    data->roi = cv::Rect(
      std::min(x, data->origin.x),
      std::min(y, data->origin.y),
      std::abs(x - data->origin.x),
      std::abs(y - data->origin.y));
  }
  switch (event) {
    case cv::EVENT_LBUTTONDOWN: {
      data->origin = cv::Point(x, y);
      data->roi = cv::Rect(x, y, 0, 0);
      data->start_selection = true;
      printf("start\n");
    } break;

    case cv::EVENT_LBUTTONUP: {
      data->start_selection = false;
      if (data->roi.area() > 0)
        data->has_template = true;
      printf("got template\n");
    } break;
  }
}

void DemoLiveCapture::Impl::mainThread() {
  //
  // get the template from the user input
  //
  MouseHandleData handle_data;
  const char *window_name = "Select ROI";
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback(window_name, onMouse, &handle_data);

  cv::Mat image, image_copy;
  int k = 0;
  while (k != 'q' && !handle_data.has_template) {
    cap_ >> image;
    if (image.empty()) {
      printf("could not poll camera\n");
      break;
    }

    image.copyTo(image_copy);

    if (handle_data.start_selection && handle_data.roi.area() > 0) {
      cv::Mat roi(image_copy, handle_data.roi);
      cv::bitwise_not(roi, roi);
    }

    cv::imshow(window_name, image_copy);
    k = cv::waitKey(5) & 0xff;
  }

  if (!handle_data.has_template) {
    printf("Terminated... exiting\n");
    stop_requested_ = true;
    return;
  }

  data_buffer_.reset(new ImageBufferType(10));
  results_buffer_.reset(new ImageBufferType(10));

  cv::cvtColor(image, image_copy, cv::COLOR_BGR2GRAY);
  tracker_->setTemplate(image_copy, handle_data.roi);

  cv::destroyWindow(window_name);
  data_thread_.reset(new std::thread(&DemoLiveCapture::Impl::dataThread, this));

  cv::namedWindow("bitplanes");

  roi_ = handle_data.roi;
  cv::Mat dimg;
  NAMESPACE::Matrix33f tform(NAMESPACE::Matrix33f::Identity());
  while (!stop_requested_) {

    std::unique_ptr<GuiData> data;
    if (data_buffer_->pop(&data)) {
      data->result = tracker_->Track(*data->gray, tform);

      tform = data->result.T;

      DrawTrackingResult(dimg, *data->image, roi_, tform.data());
      cv::imshow("bitplanes", dimg);

      int k = 0xff & cv::waitKey(5);
      if (k == 'q') {
        stop_requested_ = true;
      }
    }
  }
}

void DemoLiveCapture::Impl::dataThread() {
  std::unique_ptr<GuiData> data(new GuiData);
  while (!stop_requested_) {

    cap_ >> *data->image;
    if (data->image->empty()) {
      printf("failed to get image\n");
    }

    cv::cvtColor(*data->image, *data->gray, cv::COLOR_BGR2GRAY);
    data_buffer_->push(std::move(data));
    data.reset(new GuiData);
  }
}



