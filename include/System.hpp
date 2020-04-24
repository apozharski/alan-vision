#pragma once
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include "System.h"
#include <utility>
#include <tuple>
namespace alan
{
class AlanVisionSystem
{
 private:
  cv::VideoCapture left_camera, right_camera;
  double timestamp;
  ORB_SLAM2::System* SLAM;
  std::thread* tFrameGrabber;
  static void FrameGrabber(AlanVisionSystem*);
  void setLastTime(double);
 public:
  AlanVisionSystem(int, int);
  void initSlam(const string&, const string&, const bool);
  cv::Mat testFrames();
  cv::Mat kpOnFrames();
  std::tuple<cv::Mat, cv::Mat, double> retrieveFramePair();
  bool grabFramePair();
  void setFPS(double);
  cv::Mat TrackStereo(cv::Mat, cv::Mat, double);
};
}
