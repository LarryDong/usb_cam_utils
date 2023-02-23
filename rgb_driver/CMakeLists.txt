cmake_minimum_required(VERSION 3.0.2)
project(rgb_driver)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED) 

catkin_package()

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)


link_directories(
    # "/home/larrydong/lenovo_dvs_ws/src/celex_ros/libs"
)

add_executable(rgb_driver src/main.cpp)
target_link_libraries(rgb_driver ${catkin_LIBRARIES} ${OpenCV_LIBS} gflags)
