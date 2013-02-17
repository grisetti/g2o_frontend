cmake_minimum_required(VERSION 2.8)
include_directories(${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(sensor_data_library 
  data_queue.cpp
  data_queue.h
  imu_data.cpp
  imu_data.h
  laser_robot_data.cpp
  laser_robot_data.h
  mutexed.cpp
  mutexed.h
  omnicam_data.cpp
  omnicam_data.h
  priority_data_queue.cpp
  priority_data_queue.h
  priority_synchronous_data_queue.cpp
  priority_synchronous_data_queue.h
  rgbd_data.cpp
  rgbd_data.h
  sensor.cpp
  sensor_data.cpp
  sensor_data.h
  sensor.h
  sensor_handler.cpp
  sensor_handler.h
  sensor_imu.cpp
  sensor_imu.h
  sensor_laser_robot.cpp
  sensor_laser_robot.h
  sensor_omnicam.cpp
  sensor_omnicam.h
  sensor_rgbd_camera.cpp
  sensor_rgbd_camera.h
  sequential_data_queue.cpp
  sequential_data_queue.h
  sequential_synchronous_data_queue.cpp
  sequential_synchronous_data_queue.h
)

SET_TARGET_PROPERTIES(sensor_data_library  PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_sensor_data)

IF(CMAKE_BUILD_TYPE MATCHES Release)
  TARGET_LINK_LIBRARIES(sensor_data_library
    ${G2O_TYPES_DATA}
    ${G2O_TYPES_SLAM2D}
    ${G2O_TYPES_SLAM3D}
    ${G2O_STUFF_LIBRARY}
    ${G2O_CORE_LIBRARY}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${OPENGL_gl_LIBRARY}
    pthread
  )
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  TARGET_LINK_LIBRARIES(sensor_data_library
    ${G2O_TYPES_DATA_DEBUG}
    ${G2O_TYPES_SLAM2D_DEBUG}
    ${G2O_TYPES_SLAM3D_DEBUG}
    ${G2O_STUFF_LIBRARY_DEBUG} 
    ${G2O_CORE_LIBRARY_DEBUG} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${OPENGL_gl_LIBRARY}
    pthread
  )
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

ADD_EXECUTABLE(sequential_read sequential_read.cpp)
SET_TARGET_PROPERTIES(sequential_read PROPERTIES OUTPUT_NAME sequential_read)
TARGET_LINK_LIBRARIES(sequential_read 
  sensor_data_library  
  ${G2O_TYPES_DATA}
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_STUFF_LIBRARY}
  ${G2O_CORE_LIBRARY}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${CSPARSE_LIBRARY}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${OPENGL_gl_LIBRARY}
  pthread
  )

ADD_EXECUTABLE(add_imu add_imu.cpp)
SET_TARGET_PROPERTIES(add_imu PROPERTIES OUTPUT_NAME add_imu)
TARGET_LINK_LIBRARIES(add_imu 
  sensor_data_library  
  ${G2O_TYPES_DATA}
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_STUFF_LIBRARY}
  ${G2O_CORE_LIBRARY}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${CSPARSE_LIBRARY}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${OPENGL_gl_LIBRARY}
  pthread
  )
