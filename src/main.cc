#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "System.hpp"

int main(int, char**)
{
  alan::AlanVisionSystem system(2,0);
  // TODO this was just for cmake testing. get our own yaml.
  string vocab = "../ORB_SLAM2/Vocabulary/ORBvoc.txt";
  string yaml = "../ORB_SLAM2/Examples/Stereo/KITTI00-02.yaml";

  system.initSlam(vocab, yaml, false);
  cv::Mat stereo_frame;

  for(;;)
  {
    stereo_frame = system.kpOnFrames();
    cv::imshow("Live", stereo_frame);
    cv::waitKey(5);
  }

  return 0;
}
