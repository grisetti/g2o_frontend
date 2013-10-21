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


#set the path for g2o_frontend
set(G2O_FRONTEND $ENV{G2O_FRONTEND_ROOT})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${G2O_FRONTEND} ${EIGEN3_INCLUDE_DIR})
rosbuild_add_boost_directories()

link_directories(${G2O_FRONTEND}/lib)
rosbuild_add_executable(boss_logger_executable 
  src/ros_message_context.h src/ros_message_context.cpp
  src/ros_message_handler.cpp src/ros_message_handler.h
  src/ros_transform_message_handler.cpp src/ros_transform_message_handler.h
  src/ros_pinholeimagedata_message_handler.cpp src/  src/ros_pinholeimagedata_message_handler.h
  src/ros_laser_message_handler.cpp src/  src/ros_laser_message_handler.h
  src/ros_imu_data_message_handler.cpp src/  src/ros_imu_data_message_handler.h
  src/boss_logger.cpp )
target_link_libraries(boss_logger_executable boss_logger g2o_frontend_boss)
SET_TARGET_PROPERTIES(boss_logger_executable PROPERTIES OUTPUT_NAME boss_logger)
rosbuild_link_boost(boss_logger_executable  signals)

rosbuild_add_executable(boss_publisher
	src/ros_sensor_publisher_handler.cpp src/ros_sensor_publisher_handler.h
	src/ros_laser_publisher_handler.cpp src/ros_laser_publisher_handler.h
	src/ros_pinhole_camera_publisher_handler.cpp src/ros_pinhole_camera_publisher_handler.h
	src/ros_imu_publisher_handler.cpp src/ros_imu_publisher_handler.h
	src/boss_publisher.cpp )
target_link_libraries(boss_publisher boss_logger g2o_frontend_boss)
SET_TARGET_PROPERTIES(boss_publisher PROPERTIES OUTPUT_NAME boss_publisher)
