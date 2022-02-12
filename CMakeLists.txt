cmake_minimum_required(VERSION 3.5)
project(bike_sentry)
# locate dependencies
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  yaml
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    yaml
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS src/audio_recorder.py src/audio_classifier.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
