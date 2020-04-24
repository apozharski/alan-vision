#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include "System.hpp"
#include <thread>
#include <unistd.h>
#include <tuple>
#include <chrono>
namespace alan {

AlanVisionSystem::AlanVisionSystem(int l_camera_id, int r_camera_id)
{
  int api_id = cv::CAP_ANY; // TODO: Make this a parameter for testing 
  left_camera.open(l_camera_id + api_id);
  if(!left_camera.isOpened())
  {
    std::cout << "Left camera failed to open"<< std::endl;
  }
  right_camera.open(r_camera_id + api_id);
  if(!right_camera.isOpened())
  {
    std::cout << "Right camera failed to open"<< std::endl;
  }

  tFrameGrabber = new std::thread(AlanVisionSystem::FrameGrabber, this);
}

cv::Mat AlanVisionSystem::testFrames()
{
  cv::Mat right_frame,left_frame, stereo_frame;
  left_camera.read(left_frame);
  right_camera.read(right_frame);
  cv::hconcat(left_frame, right_frame, stereo_frame);
  return stereo_frame;
}

cv::Mat AlanVisionSystem::kpOnFrames()
{
  // Initialize required structures
  cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
  cv::Mat right_frame,left_frame, stereo_frame;
  std::vector<cv::KeyPoint> lkpts, rkpts;
  cv::Mat ldesc, rdesc;

  // Read frames from cameras
  left_camera.read(left_frame);
  right_camera.read(right_frame);

  // Detect 
  akaze->detectAndCompute(left_frame, cv::noArray(), lkpts, ldesc);
  akaze->detectAndCompute(right_frame, cv::noArray(), rkpts, rdesc);

  cv::BFMatcher matcher(cv::NORM_HAMMING);
  float nn_match_ratio = 0.8f;
  std::vector<std::vector<cv::DMatch> > nn_matches;
  matcher.knnMatch(ldesc, rdesc, nn_matches, 2);
  
  std::vector<cv::KeyPoint> lmatched, rmatched;
  std::vector<cv::DMatch> matched;
  for(size_t i = 0; i < nn_matches.size(); i++) {
    cv::DMatch first = nn_matches[i][0];
    float dist1 = nn_matches[i][0].distance;
    float dist2 = nn_matches[i][1].distance;
    if(dist1 < nn_match_ratio * dist2) {
      int new_i = static_cast<int>(lmatched.size());
      matched.push_back(cv::DMatch(new_i, new_i, 0));
      lmatched.push_back(lkpts[first.queryIdx]);
      rmatched.push_back(rkpts[first.trainIdx]);
    }
  }
  //std::cout << lmatched.size() << " " << rmatched.size() << " " << matched.size() << std::endl;
  drawMatches(left_frame, lmatched, right_frame, rmatched, matched, stereo_frame);

  return stereo_frame;
}

void AlanVisionSystem::initSlam(const string &orb_vocab_path,
                                  const string &orb_settings_path,
                                  const bool view = false)
{
  SLAM = new ORB_SLAM2::System(orb_vocab_path, orb_settings_path, ORB_SLAM2::System::STEREO, view);
}

std::tuple<cv::Mat, cv::Mat, double> AlanVisionSystem::retrieveFramePair()
{
  std::tuple<cv::Mat, cv::Mat,double> frames;
  left_camera.retrieve(std::get<0>(frames));
  right_camera.retrieve(std::get<1>(frames));
  std::get<2>(frames) = timestamp;
  return frames;
}

bool AlanVisionSystem::grabFramePair()
{
  bool success = left_camera.grab() & right_camera.grab();
  std::chrono::time_point<std::chrono::high_resolution_clock>
    tp = std::chrono::high_resolution_clock::now();
  double micros = tp.time_since_epoch().count();
  setLastTime(micros);
  return success;
}

void AlanVisionSystem::setFPS(double FPS)
{
  left_camera.set(cv::CAP_PROP_FPS, FPS);
  right_camera.set(cv::CAP_PROP_FPS, FPS);
}
void AlanVisionSystem::FrameGrabber(AlanVisionSystem* system)
{
  for(;;)
  {
    bool grabbed = system->grabFramePair();
    if(!grabbed)
      exit(EXIT_FAILURE);
    usleep(100000);// TODO Make this dynamic based on framerate. (currently, this is dumb).
  }
}

void AlanVisionSystem::setLastTime(double ts)
{
  timestamp = ts;
}

cv::Mat AlanVisionSystem::TrackStereo(cv::Mat left, cv::Mat right, double ts)
{
  return SLAM->TrackStereo(left, right, ts);
}
}
