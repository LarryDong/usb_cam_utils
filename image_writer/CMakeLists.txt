cmake_minimum_required(VERSION 3.0.2)
project(image_writer)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)


link_directories(
    # "/home/larrydong/lenovo_dvs_ws/src/celex_ros/libs"
)

# image online writer on ROS
add_executable(online_writer src/online_writer.cpp)
target_link_libraries(online_writer ${catkin_LIBRARIES} ${OpenCV_LIBS})

# image offline writer from rosbag
add_executable(offline_writer src/offline_writer.cpp)
target_link_libraries(offline_writer ${catkin_LIBRARIES} ${OpenCV_LIBS})

# image and imu offline writer from rosbag
add_executable(offline_imu_writer src/offline_imu_writer.cpp)
target_link_libraries(offline_imu_writer ${catkin_LIBRARIES} ${OpenCV_LIBS})
