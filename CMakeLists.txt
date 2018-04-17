cmake_minimum_required(VERSION 3.5)
project(cSfM)

find_package(OpenCV 3.4.1 REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Ceres QUIET)
find_package(GTSAM REQUIRED)
find_package(GTSAMCMakeTools)
#find_package(Boost REQUIRED)

set(CMAKE_CXX_STANDARD 11)

#set(Boost_USE_STATIC_LIBS OFF)
#set(Boost_USE_MULTITHREADED ON)
#set(Boost_USE_STATIC_RUNTIME OFF)
#find_package(Boost)
#
#if(Boost_FOUND)
#    include_directories(${Boost_INCLUDE_DIRS})
#    add_executable(progname main.cpp cSfM.h)
#    target_link_libraries(progname ${Boost_LIBRARIES})
#endif()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${GTSAM_INCLUDE_DIR})
#include_directories(${BOOST_INCLUDE_DIR})
set(SOURCE_FILES main.cpp)
add_executable(cSfM main.cpp cSfM.h)
target_link_libraries(cSfM ${OpenCV_LIBS} gtsam) #, ${Boost_LIBRARIES})

#set(CMAKE_SHARED_LINKER_FLAGS "-lboost_system")