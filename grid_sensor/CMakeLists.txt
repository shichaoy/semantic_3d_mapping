cmake_minimum_required(VERSION 2.8.3)
project(grid_sensor)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  pcl_ros
  pcl_conversions
  scrollgrid
  gridmapping2
  cv_bridge
  dense_crf
  tf
  tf_conversions  
)

  

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Boost REQUIRED COMPONENTS filesystem system thread)

find_package(PCL REQUIRED)

find_package(Eigen REQUIRED)

find_package(OpenCV 2.4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rgbd_grid_sensor
  CATKIN_DEPENDS cmake_modules roscpp pcl_ros pcl_conversions scrollgrid
  gridmapping2   dense_crf  cv_bridge   tf  tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

# set the build type: "Choose the type of build. Options are: None, Debug, Release, RelWithDebInfo, MinSizeRel"
set(CMAKE_BUILD_TYPE Release)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/grid_sensor.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES} 
  rt
)

## Declare a cpp executable
add_executable(stereo_img_node src/main_stereo_imgs.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rgbd_grid_sensor_node rgbd_grid_sensor_generate_messages_cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(stereo_img_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES} 
  rt
)
#add_executable(${PROJECT_NAME}_create_visualize_example
#  examples/create_visualize_example.cpp)

# target_link_libraries( ${PROJECT_NAME}_create_visualize_example
#  ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${Boost_LIBRARIES} rt
# )

message("---------------------------------------------")
message(${PROJECT_NAME} " Configuration Summary")
message("Build type:                    ${CMAKE_BUILD_TYPE}")
message("---------------------------------------------")
message("Opencv dir"   ${OpenCV_INCLUDE_DIRS})