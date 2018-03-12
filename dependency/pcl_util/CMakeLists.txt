cmake_minimum_required(VERSION 2.8.3)
project(pcl_util)

# TODO remove this
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    geom_cast
    geom_cast_extra
    roscpp
    pcl_ros
    pcl_conversions
    cmake_modules
    )
find_package(Eigen REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    DEPENDS Eigen PCL OpenCV Boost
    CATKIN_DEPENDS geom_cast geom_cast_extra roscpp pcl_ros pcl_conversions
    INCLUDE_DIRS include
    LIBRARIES
)

###########
## Build ##
###########

#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

catkin_add_gtest(${PROJECT_NAME}_gtests test/gtests.cpp)
target_link_libraries(${PROJECT_NAME}_gtests ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES}) 

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_IO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_examples src/examples.cpp)
target_link_libraries(${PROJECT_NAME}_examples
    ${OpenCV_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_pc2_iterator_examples src/pc2_iterator_examples.cpp)
target_link_libraries(${PROJECT_NAME}_pc2_iterator_examples
    ${OpenCV_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_rgb src/rgb.cpp)
target_link_libraries(${PROJECT_NAME}_rgb
    ${OpenCV_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
