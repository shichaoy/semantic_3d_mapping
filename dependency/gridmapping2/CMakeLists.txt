cmake_minimum_required(VERSION 2.8.3)
project(gridmapping2)

find_package(catkin REQUIRED COMPONENTS 
    roscpp pcl_util geom_cast scrollgrid cmake_modules)

find_package(Boost REQUIRED COMPONENTS filesystem system thread)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS Boost rt pthread
    CATKIN_DEPENDS roscpp pcl_util geom_cast scrollgrid
    INCLUDE_DIRS include
#    LIBRARIES 
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

# set the build type: "Choose the type of build. Options are: None, Debug, Release, RelWithDebInfo, MinSizeRel"
set(CMAKE_BUILD_TYPE Release)

add_executable(${PROJECT_NAME}_empty src/empty.cpp)
target_link_libraries(${PROJECT_NAME}_empty ${Boost_LIBRARIES} ${catkin_LIBRARIES} rt pthread)

add_executable(${PROJECT_NAME}_shared_gridmap_node src/shared_gridmap_node.cpp)
target_link_libraries(${PROJECT_NAME}_shared_gridmap_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} rt pthread)

add_executable(${PROJECT_NAME}_proxy_example1 examples_src/proxy_example1.cpp)
target_link_libraries(${PROJECT_NAME}_proxy_example1 ${Boost_LIBRARIES}  ${catkin_LIBRARIES} rt pthread)

add_executable(${PROJECT_NAME}_proxy_example2 examples_src/proxy_example2.cpp)
target_link_libraries(${PROJECT_NAME}_proxy_example2 ${Boost_LIBRARIES}  ${catkin_LIBRARIES} rt pthread)

add_executable(${PROJECT_NAME}_test_synch examples_src/test_synch.cpp)
target_link_libraries(${PROJECT_NAME}_test_synch ${Boost_LIBRARIES} ${catkin_LIBRARIES} rt pthread)

add_executable(${PROJECT_NAME}_shared_grid_example
  examples_src/shared_grid_example.cpp)
target_link_libraries(${PROJECT_NAME}_shared_grid_example ${Boost_LIBRARIES} ${catkin_LIBRARIES} rt pthread)


message("---------------------------------------------")
message(${PROJECT_NAME} " Configuration Summary")
message("Build type:                    ${CMAKE_BUILD_TYPE}")
message("---------------------------------------------")