#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "System.hpp"
#include <utility>
#include <tuple>

int main(int, char**)
{
  std::cout << "Loading" << std::endl;
  cv::FileStorage fsSettings("test.yml", cv::FileStorage::READ);

  cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
  std::vector<int> S;
    
  fsSettings["Left"]["K"] >> K_l;
  fsSettings["Right"]["K"] >> K_r;

  fsSettings["Left"]["P"] >> P_l;
  fsSettings["Right"]["P"] >> P_r;
  
  fsSettings["Left"]["R"] >> R_l;
  fsSettings["Right"]["R"] >> R_r;
  
  fsSettings["Left"]["D"] >> D_l;
  fsSettings["Right"]["D"] >> D_r;
  fsSettings["Size"] >> S;

  cv::Size image_size(S[0],S[1]);
  
  std::cout << "Initing Undistort" << std::endl;
  cv::Mat M1l,M2l,M1r,M2r;
  cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),image_size,CV_32F,M1l,M2l);
  cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),image_size,CV_32F,M1r,M2r);
  
  std::cout << "Starting ALAN-Vision" << std::endl;
  alan::AlanVisionSystem av_system(2,0);
  // TODO this was just for cmake testing. get our own yaml.
  string vocab = "../ORB_SLAM2/Vocabulary/ORBvoc.txt";
  string yaml = "test.yml";

  av_system.initSlam(vocab, yaml, true);
  std::tuple<cv::Mat, cv::Mat, double> frames;

  cv::Mat imLeftRect, imRightRect;
  for(;;)
  {
    frames = av_system.retrieveFramePair();
    cv::remap(std::get<0>(frames), imLeftRect,M1l,M2l,cv::INTER_LINEAR);
    cv::remap(std::get<1>(frames),imRightRect,M1r,M2r,cv::INTER_LINEAR);
    cv::waitKey(10); // TODO: Move this to another thread and synchronize? lots of questions on how to do this right
    av_system.TrackStereo(imLeftRect,imRightRect,std::get<2>(frames));
  }
  
  return 0;
}
