cmake_minimum_required(VERSION 3.0)

# set the project name
project(alan-vision VERSION 0.1)

# Orb-slam deps
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)


# Compile to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# required packages
find_package(OpenCV REQUIRED)

# Build orbslam shared library.
add_custom_target(
    ORB_SLAM2
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/ORB_SLAM2/
    COMMAND ${CMAKE_SOURCE_DIR}/ORB_SLAM2/build.sh
)

# alan-vision library
add_library(AlanVisionSystem src/System.cc)
add_dependencies(AlanVisionSystem ORB_SLAM2)

# Main alan-vision executable
add_executable(Main src/main.cc)
include_directories(include
  ORB_SLAM2
  ORB_SLAM2/include
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  )

target_link_libraries(Main
  ${OpenCV_LIBS}
  AlanVisionSystem
  ${CMAKE_SOURCE_DIR}/ORB_SLAM2/lib/libORB_SLAM2.so
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
  )


configure_file(src/AlanVisionConfig.hpp.in AlanVisionConfig.hpp)


target_include_directories(Main PUBLIC
  "${PROJECT_BINARY_DIR}"
  "${PROJECT_BINARY_DIR}/ORB_SLAM2"
  )

# Calibration
add_executable(Calib src/AlanCalib.cc)

target_link_libraries(Calib
  ${OpenCV_LIBS}
  AlanVisionSystem
  ${CMAKE_SOURCE_DIR}/ORB_SLAM2/lib/libORB_SLAM2.so
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
  )
