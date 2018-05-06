cmake_minimum_required(VERSION 2.8.3)
project(mad_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  OpenCV
  cv_bridge
  #dynamic_reconfigure
  #tictoc_profiler
  #at_drone_interface
)

#find_package(OpenCV 3.3.0 REQUIRED)
#generate_dynamic_reconfigure_options(
#  cfg/Gains.cfg
#  cfg/FinalStage.cfg
  #...
#)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES landing
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs #OpenCV
   DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #/usr/include/eigen3/
  #/home/kpluckter/workspace/eigen-eigen-5a0156e40feb/
)


add_library(LKSemi src/LKSemi.cpp)
target_link_libraries(LKSemi ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(LKSemi ${PROJECT_NAME}_gencfg)

add_executable(lk_semi src/lk_semi.cpp)
target_link_libraries(lk_semi LKSemi ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})