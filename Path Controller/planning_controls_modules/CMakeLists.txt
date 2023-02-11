cmake_minimum_required(VERSION 2.8.3)
project(planning_controls_modules)

set(CMAKE_BUILD_TYPE Release) 

find_package(catkin REQUIRED COMPONENTS
rospy
rover_msgs
rover_modules
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_python_setup()
# Node: catkin_package macro is required
catkin_package(
     CATKIN_DEPENDS rospy rover_msgs rover_modules
    )

