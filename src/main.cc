#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "System.hpp"

int main(int, char**)
{
  alan::AlanVisionSystem system(2,0);

  cv::Mat stereo_frame;

  for(;;)
  {
    stereo_frame = system.kpOnFrames();
    cv::imshow("Live", stereo_frame);
    if (cv::waitKey(5) >= 0)
      break;
  }

  return 0;
}
