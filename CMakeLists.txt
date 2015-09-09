cmake_minimum_required(VERSION 2.8.12)
project(hummingbird_demo)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  quadrotor_msgs
  mav_manager
  std_msgs
  std_srvs
  cmake_modules
  message_generation)

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mav_manager
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    quadrotor_msgs
    mav_manager
    std_msgs
    message_runtime
  DEPENDS Eigen)

add_executable(hummingbird_demo src/sample.cpp src/trajectory.cpp)
target_link_libraries(hummingbird_demo mav_manager ${catkin_LIBRARIES})
