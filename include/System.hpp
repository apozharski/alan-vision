#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>

namespace alan
{
class AlanVisionSystem
{
 private:
  cv::VideoCapture left_camera, right_camera;
 public:
  AlanVisionSystem(int, int);  
  cv::Mat testFrames();
  cv::Mat kpOnFrames();
};
}
