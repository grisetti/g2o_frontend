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

#find eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
IF(EIGEN3_FOUND)
  MESSAGE("Eigen3 library found.")
ELSE(EIGEN3_FOUND)
  MESSAGE("Eigen3 library NOT found.")
ENDIF(EIGEN3_FOUND)


FIND_PACKAGE(OpenNI2 REQUIRED)
FIND_PACKAGE(ZLIB REQUIRED)

#set the path for g2o_frontend
set(G2O_FRONTEND $ENV{G2O_FRONTEND_ROOT})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${G2O_FRONTEND} ${OPENNI2_INCLUDE_DIR} ${ZLIB_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

rosbuild_add_executable(openni2_node src/openni2_node.cpp )
target_link_libraries(openni2_node ${OPENNI2_LIBRARY} ${ZLIB_LIBRARY} )

link_directories(${G2O_FRONTEND}/lib)
rosbuild_add_executable(openni2_client src/openni2_client.cpp )
target_link_libraries(openni2_client ${ZLIB_LIBRARY} boss_map g2o_frontend_boss)
