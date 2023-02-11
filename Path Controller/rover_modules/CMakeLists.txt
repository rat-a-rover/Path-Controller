cmake_minimum_required(VERSION 2.8.3)
project(rover_modules)

set(CMAKE_BUILD_TYPE Release) 

find_package(catkin REQUIRED COMPONENTS
rospy
std_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_python_setup()
# Node: catkin_package macro is required
catkin_package(
    CATKIN_DEPENDS 
    )

