cmake_minimum_required(VERSION 2.8.3)
project(dense_crf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roslib  
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED) 
find_package(OpenCV 2.4 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cmake_modules roslib
#  DEPENDS system_lib
)
  
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  external/liblbfgs/include
  external  
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   libs/densecrf.cpp libs/util.cpp  libs/permutohedral.cpp  libs/unary.cpp  libs/pairwise.cpp  libs/objective.cpp  libs/labelcompatibility.cpp
)
 
add_library( lbfgs
	external/liblbfgs/lib/lbfgs.c
) 
add_library( optimization
	libs/optimization.cpp 
)
target_link_libraries( optimization lbfgs )

add_library( img_proc_lib
	utils/ppm.cpp  utils/common.cpp
)

add_library( superpixel_lib
	libs/superpixel.cpp
)

target_link_libraries( ${PROJECT_NAME}  superpixel_lib)

# target_link_libraries( ${PROJECT_NAME} densecrf_lib optimization img_proc_lib)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(dense_crf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(dense_crf_node src/dense_crf_node.cpp)
add_executable( dense_inference    nodes/dense_inference.cpp )
target_link_libraries( dense_inference ${PROJECT_NAME} img_proc_lib )

add_executable( comare_hier_dense_crf  src/comare_hier_dense_crf.cpp )
target_link_libraries( comare_hier_dense_crf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME} img_proc_lib)


add_executable( dense_learning  nodes/dense_learning.cpp )
target_link_libraries( dense_learning  ${PROJECT_NAME}  optimization img_proc_lib)


add_executable( test_optimization  nodes/test_optimization.cpp )
target_link_libraries( test_optimization optimization )

add_executable( test_permutohedral  nodes/test_permutohedral.cpp )
target_link_libraries( test_permutohedral ${PROJECT_NAME} optimization )

add_executable( test_pairwise  nodes/test_pairwise.cpp )
target_link_libraries( test_pairwise ${PROJECT_NAME} optimization )
