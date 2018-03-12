cmake_minimum_required(VERSION 2.8.3)
project(geom_cast_extra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  geom_cast
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV)
find_package(PCL)

#catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp geom_cast
  DEPENDS Eigen Boost OpenCV PCL
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)

#target_link_libraries(${PY_PROJECT_NAME}
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
#)
