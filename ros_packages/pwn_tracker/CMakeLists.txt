cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/../../cmake_modules)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Qt4 REQUIRED)
#SET(QT_USE_QTOPENGL TRUE)
#include(${QT_USE_FILE})

#find g2o
FIND_PACKAGE(G2O REQUIRED)
IF(G2O_FOUND)
  SET(${PROJECT_NAME}_G2O_INCLUDE ${G2O_INCLUDE_DIR} CACHE PATH "Directory of G2O include")
  SET(${PROJECT_NAME}_G2O_LIBRARY_PATH ${G2O_LIBRARY_PATH} CACHE PATH "Directory of G2O libraries")
  MESSAGE("g2o library found.")
ELSE(G2O_FOUND)
  SET(${PROJECT_NAME}_G2O_INCLUDE "" CACHE PATH "Directory of G2O include")
  MESSAGE("g2o library NOT found.")
ENDIF(G2O_FOUND)

#set the path for g2o_frontend
set(G2O_FRONTEND $ENV{G2O_FRONTEND_ROOT})
link_directories(${G2O_FRONTEND}/lib)

#find eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
IF(EIGEN3_FOUND)
  MESSAGE("Eigen3 library found.")
ELSE(EIGEN3_FOUND)
  MESSAGE("Eigen3 library NOT found.")
ENDIF(EIGEN3_FOUND)

include_directories(${G2O_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${G2O_FRONTEND} ${QT_INCLUDES})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(pwn_tracker_node src/pwn_tracker_ros.cpp  src/pwn_tracker_node.cpp )
target_link_libraries(pwn_tracker_node g2o_frontend_pwn_tracker g2o_frontend_pwn_core g2o_frontend_boss_map g2o_frontend_boss)
rosbuild_link_boost(pwn_tracker_node  signals)

#rosbuild_add_executable(cfind src/cfind.cpp)
