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
include_directories(${EIGEN3_INCLUDE_DIR})

#find g2o
FIND_PACKAGE(G2O REQUIRED)
IF(G2O_FOUND)
  SET(${PROJECT_NAME}_G2O_INCLUDE ${G2O_INCLUDE_DIR} CACHE PATH "Directory of G2O include")
  SET(${PROJECT_NAME}_G2O_LIBRARY_PATH ${G2O_LIBRARY_PATH} CACHE PATH "Directory of G2O libraries")
  include_directories(${G2O_INCLUDE_DIR})
  MESSAGE("g2o library found.")
ELSE(G2O_FOUND)
  SET(${PROJECT_NAME}_G2O_INCLUDE "" CACHE PATH "Directory of G2O include")
  MESSAGE("g2o library NOT found.")
ENDIF(G2O_FOUND)

#find pcl
set(PCL_DIR $ENV{PCL_ROOT})
find_package(PCL 1.7 REQUIRED)
IF(PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
ENDIF(PCL_FOUND)

# Find RobotEye
FIND_PACKAGE( RobotEye REQUIRED )
IF(RobotEye_FOUND)
   include_directories(${RobotEye_INCLUDE_DIR})
   #link_directories(${RobotEye_LIBRARIES})
ENDIF(RobotEye_FOUND)

rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
rosbuild_genmsg()

#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

# rosbuild_add_executable(robot_eye_test src/eyebot_test.cpp)
# target_link_libraries(robot_eye_test ${RobotEye_LIBRARIES} ${G2O_STUFF_LIBRARY} ${PCL_LIBRARIES})

# rosbuild_add_executable(robot_eye_driver src/roboteye_scan_publisher.cpp)
# target_link_libraries(robot_eye_driver ${RobotEye_LIBRARIES} ${G2O_STUFF_LIBRARY} ${PCL_LIBRARIES})

rosbuild_add_executable(robot_eye_node src/main.cpp src/mutex.cpp src/roboteye_node.cpp src/robot_eye_callback.cpp)
target_link_libraries(robot_eye_node ${RobotEye_LIBRARIES} ${G2O_STUFF_LIBRARY} ${PCL_LIBRARIES})
