#Project
cmake_minimum_required(VERSION 2.4)
project(stereoslic)

# Directories
set (INC_DIR libraries)
set (PNG_INC_DIR libraries/png/libpng)
set (PNG_LIB_DIR libraries/png)

# Include directories
include_directories(${INC_DIR} ${PNG_INC_DIR})

# Library directories
link_directories(${PNG_LIB_DIR})

# Flags
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")
set(CMAKE_BUILD_TYPE Release)

# Sources
file(GLOB STEREOSLIC_SRC_FILES "*.cpp")

add_executable(stereoslic ${STEREOSLIC_SRC_FILES})
target_link_libraries(stereoslic png)
