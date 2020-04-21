#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include "System.h"
#include <utility>

namespace alan
{
class AlanVisionSystem
{
 private:
  cv::VideoCapture left_camera, right_camera;
  ORB_SLAM2::System* SLAM;
 public:
  AlanVisionSystem(int, int);
  void initSlam(const string&, const string&, const bool);
  cv::Mat testFrames();
  cv::Mat kpOnFrames();
  std::pair<cv::Mat, cv::Mat> retrieveFramePair();
  bool grabFramePair();
};
}
