#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "System.hpp"
#include <utility> 
#include <thread>
#include <unistd.h>
#include <stdlib.h> 
// TODO: Should be done better.
// Global System 
alan::AlanVisionSystem av_system(2,0);

void frameGrabLoop()
{
  for(;;)
  {
    bool grabbed = av_system.grabFramePair();
    if(!grabbed)
      exit(EXIT_FAILURE);
    std::cout << "Grabbing Frames" << std::endl;
    usleep(1000000);
  }
}

int main(int, char**)
{
  std::thread grabber(frameGrabLoop);
  std::pair<cv::Mat, cv::Mat> frames;

  for(;;)
  {
    cv::waitKey(5);
    frames = av_system.retrieveFramePair();
    cv::imshow("Left", frames.first);
    cv::imshow("Right", frames.second);
  }

  return 0;
}
