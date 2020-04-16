cmake_minimum_required(VERSION 3.0)

# set the project name
project(alan-vision VERSION 0.1)
# required packages
find_package(OpenCV REQUIRED)

# alan-vision library
add_library(AlanVisionSystem src/System.cc)

# Main alan-vision executable
add_executable(Main src/main.cc)
include_directories(include)
target_link_libraries(Main ${OpenCV_LIBS} AlanVisionSystem)

configure_file(src/AlanVisionConfig.hpp.in AlanVisionConfig.hpp)


target_include_directories(Main PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           )
