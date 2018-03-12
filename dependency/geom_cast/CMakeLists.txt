cmake_minimum_required(VERSION 2.8.3)
project(geom_cast)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  cmake_modules
)

# need boost for metatemplate stuff
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs tf
  DEPENDS Eigen Boost
  #CFG_EXTRAS config_extras.cmake
)

###########
## Build ##
###########

# TODO I think it's harmless to have include's
# if they are not present
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

#############
## Testing ##
#############

#if (CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(${PROJECT_NAME}_test utest/test.cpp)
#    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_COMMON_LIBRARIES})
#endif()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_geom_cast.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
