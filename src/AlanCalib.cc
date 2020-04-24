#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "System.hpp"
#include <utility> 
#include <thread>
#include <unistd.h>
#include <stdlib.h>
#include <atomic>
#include <tuple>
// TODO: Should be done better.
// Global System 
alan::AlanVisionSystem av_system(2,0);
// TODO: Make this a function of Vision system
std::atomic_bool stop_grabbing(false);

// TODO: Should be a feature of Alan Vision system
void frameGrabLoop()
{
  for(;;)
  {
    if(stop_grabbing)
      continue;
    bool grabbed = av_system.grabFramePair();
    if(!grabbed)
      exit(EXIT_FAILURE);
    usleep(100000);
  }
}

int main(int, char**)
{
  // Setup frame data structires
  std::tuple<cv::Mat, cv::Mat, double> frame_pair;
  std::vector<std::tuple<cv::Mat, cv::Mat, double>> frames;

  // TODO: Make this dynamic
  // Get board specs
  int board_width = 10, board_height = 7;
  cv::Size board_size(board_width, board_height);
  float square_size = 20;
  
  // Setup system FPS
  av_system.setFPS(10);

  // Setup grabber 
  //std::thread grabber(frameGrabLoop);
  //grabber.detach();

  // Capture images
  for(;;)
  {
    if(cv::waitKey(0) == 27)
      break;
    frame_pair = av_system.retrieveFramePair();
    cv::imshow("Left", std::get<0>(frame_pair));
    cv::imshow("Right", std::get<1>(frame_pair));
    frames.push_back(frame_pair);
  }
  
  // Stop grabber thread.
  stop_grabbing = true;

  //
  vector<cv::Point2f> lcorners, rcorners;
  vector<vector<cv::Point3f>> shared_object_points, left_object_points, right_object_points;
  vector<vector<cv::Point2f>> left_image_points, right_image_points;
  vector<vector<cv::Point2f>> shared_left_image_points, shared_right_image_points;
  bool lfound, rfound; 
  for(auto calib_pair : frames)
  {
    // Convert to gray
    cv::Mat lgray, rgray;
    cv::cvtColor(std::get<0>(calib_pair), lgray, CV_BGR2GRAY);
    cv::cvtColor(std::get<1>(calib_pair), rgray, CV_BGR2GRAY);
    // Find chessboard corners
    lfound = cv::findChessboardCorners(lgray, board_size, lcorners,
                                       CV_CALIB_CB_ADAPTIVE_THRESH);
    rfound = cv::findChessboardCorners(rgray, board_size, rcorners,
                                       CV_CALIB_CB_ADAPTIVE_THRESH);

    // Generate 3d object 
    vector<cv::Point3f> obj;
    for(int i = 0; i < board_height; i++)
      for(int j = 0; j < board_width; j++)
        obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0));
    
    // Refine chessboard corners
    if (lfound)
    {
      cv::cornerSubPix(lgray, lcorners, cv::Size(5, 5), cv::Size(-1, -1),
                       cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.5));
      cv::drawChessboardCorners(std::get<0>(calib_pair), board_size, lcorners, lfound);
      left_image_points.push_back(lcorners);
      left_object_points.push_back(obj);
    }
    if (rfound)
    {
      cv::cornerSubPix(rgray, rcorners, cv::Size(5, 5), cv::Size(-1, -1),
                       cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.5));
      cv::drawChessboardCorners(std::get<1>(calib_pair), board_size, rcorners, rfound);
      right_image_points.push_back(rcorners);
      right_object_points.push_back(obj);
    }

    // If we succeeded we put the matches in given we found them
    if (lfound && rfound) {
      std::cout << "Found corners" << std::endl;
      shared_left_image_points.push_back(lcorners);
      shared_right_image_points.push_back(rcorners);
      shared_object_points.push_back(obj);
    }
  }

  // TODO do single calibration first?
  // Setup calibration matrices
  cv::Mat K1, K2, R;
  cv::Mat E, F;
  std::vector<cv::Mat> rvec1, rvec2, tvec1, tvec2;
  cv::Mat T;
  cv::Mat D1, D2;
  int mono_flag = 0;
  mono_flag |= cv::CALIB_RATIONAL_MODEL;
  double lrms = cv::calibrateCamera(left_object_points, left_image_points, std::get<0>(frame_pair).size(),
                                    K1, D1, rvec1, tvec1, mono_flag);
  std::cout << "left rms" << lrms << std::endl;
  double rrms = cv::calibrateCamera(right_object_points, right_image_points, std::get<0>(frame_pair).size(),
                                    K2, D2, rvec2, tvec2, mono_flag);
  std::cout << "right rms" << rrms << std::endl;
  int stereo_flag = 0;
  stereo_flag |= cv::CALIB_RATIONAL_MODEL;
  double rms = cv::stereoCalibrate(shared_object_points, shared_left_image_points, shared_right_image_points,
                                   K1, D1, K2, D2, std::get<0>(frame_pair).size(), R, T, E, F, stereo_flag);
  //cv::TermCriteria(3, 12, 0)
                                 
  std::cout << "rms" << rms << std::endl;

  cv::Mat R1, R2, P1, P2, Q;
  cv::stereoRectify(K1, D1, K2, D2, std::get<0>(frame_pair).size(), R, T, R1, R2, P1, P2, 
                    Q, CV_CALIB_ZERO_DISPARITY, -1,std::get<0>(frame_pair).size());

  std::cout << "Q" << Q << std::endl;


  cv::FileStorage fs("out.yml", cv::FileStorage::WRITE);



  fs << "Camera fx" << P1.at<double>(0,0);
  fs << "Camera fy" << P1.at<double>(1,1);
  fs << "Camera cx" << P1.at<double>(0,2);
  fs << "Camera cy" << P1.at<double>(1,2);

  fs << "Camera k1" << 0;
  fs << "Camera k2" << 0;
  fs << "Camera p1" << 0;
  fs << "Camera p2" << 0;
  
  fs << "Left" << "{";
  fs << "K" << K1;
  fs << "D" << D1;
  fs << "R" << R1;
  fs << "P" << P1;
  fs << "}";
  fs << "Right" << "{";
  fs << "K" << K2;
  fs << "D" << D2;
  fs << "R" << R2;
  fs << "P" << P2;
  fs << "}";

  fs << "R" << R;
  fs << "T" << T;
  fs << "Size" << std::get<0>(frame_pair).size();
  
  stop_grabbing = false;
  for(;;)
  {
    if(cv::waitKey(5) == 27)
      break;
    frame_pair = av_system.retrieveFramePair();
    cv::Mat left, right;
    cv::undistort(std::get<0>(frame_pair), left, K1, D1);
    cv::undistort(std::get<1>(frame_pair), right, K2, D2);
    cv::imshow("Left", left);
    cv::imshow("Right", right);
    frames.push_back(frame_pair);
  }
  return 0;
}
