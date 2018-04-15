cmake_minimum_required(VERSION 3.5)
project(cSfM)

find_package(OpenCV 3.4.1 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres QUIET)
find_package(GTSAM REQUIRED)
find_package(GTSAMCMakeTools)

set(CMAKE_CXX_STANDARD 11)

add_definitions("-DCERES_FOUND=1")
list(APPEND LIBMV_LIGHT_LIBS simple_pipeline)
list(APPEND LIBMV_LIGHT_INCLUDES "${CERES_INCLUDE_DIR}")

list (FIND CERES_COMPILED_COMPONENTS "C++11" _index)
if (${_index} GREATER -1)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${GTSAM_INCLUDE_DIR})
set(SOURCE_FILES main.cpp)
add_executable(cSfM main.cpp cSfM.h)
target_link_libraries(cSfM ${OpenCV_LIBS} gtsam)