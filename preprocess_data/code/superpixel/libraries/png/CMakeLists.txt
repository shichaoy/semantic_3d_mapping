cmake_minimum_required(VERSION 2.4)

project(libpng)

include_directories(libpng zlib)

set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")
set(CMAKE_BUILD_TYPE Release)

file(GLOB LIBPNG_SRC_FILES "libpng/*.c" "zlib/*.c")

add_library(png ${LIBPNG_SRC_FILES})
