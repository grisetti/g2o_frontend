cmake_minimum_required(VERSION 2.8)

ADD_LIBRARY(sensor_data_library 
	DataQueue.cpp DataQueue.h
	Mutexed.cpp Mutexed.h
	PriorityDataQueue.cpp PriorityDataQueue.h
	PrioritySynchronousDataQueue.cpp PrioritySynchronousDataQueue.h
	Sensor.cpp Sensor.h
	SensorHandler.cpp SensorHandler.h
	SensorRGBDCamera.cpp SensorRGBDCamera.h
	SensorOmnicam.cpp SensorOmnicam.h
	SequentialDataQueue.cpp SequentialDataQueue.h
	SequentialSynchronousDataQueue.cpp SequentialSynchronousDataQueue.h
	SensorData.cpp SensorData.h  	
	RGBDData.cpp RGBDData.h
	OmnicamData.cpp OmnicamData.h
	LaserRobotData.cpp LaserRobotData.h 
	SensorLaserRobot.cpp SensorLaserRobot.h
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

