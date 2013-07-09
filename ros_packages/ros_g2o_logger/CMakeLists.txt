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

FIND_PACKAGE(Qt4)
INCLUDE(${QT_USE_FILE})

FIND_PACKAGE(SuiteSparse)
IF(CSPARSE_FOUND)
MESSAGE(STATUS "CSPARSE FOUND")  
ELSE(CSPARSE_FOUND)
MESSAGE(STATUS "CSPARSE NOT FOUND")
ENDIF(CSPARSE_FOUND)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories(${G2O_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${G2O_FRONTEND} ${QT_INCLUDES} ${CSPARSE_INCLUDE_DIR} 
	#${QT_INCLUDES} ${QT_QTCORE_INCLUDE_DIR} ${QT_QTCORE_INCLUDE_DIR}/..
	#${QGLVIEWER_INCLUDE_DIR} 
	)
link_directories(${G2O_FRONTEND}/lib)

rosbuild_add_boost_directories()
rosbuild_add_library(SensorHandlers  src/SensorHandlerRGBDCamera.cpp )
rosbuild_add_library(SensorHandlers  src/SensorHandlerLaserRobot.cpp )
rosbuild_add_library(SensorHandlers  src/SensorHandlerOmnicam.cpp )
rosbuild_add_library(SensorHandlers 
src/SensorHandlerImu.cpp
)


rosbuild_link_boost(SensorHandlers thread)
rosbuild_add_executable(logger src/Logger.cpp)
rosbuild_link_boost(logger signals)
target_link_libraries(logger 
  ${G2O_CORE_LIBRARY}
  ${G2O_OPENGL_HELPER_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_STUFF_LIBRARY}
  ${G2O_CORE_LIBRARY}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${CSPARSE_LIBRARY}
  SensorHandlers 
  g2o_frontend_sensor_data 
  g2o_frontend_pwn2_library 
  g2o_frontend_pwn_viewer 
  #g2o_frontend_traverse_library 
  pwn_mapper
)

rosbuild_add_executable(bag2img src/bag_2_img.cpp)
rosbuild_link_boost(bag2img signals)
target_link_libraries(bag2img ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} g2o_types_slam3d SensorHandlers g2o_frontend_sensor_data
)
