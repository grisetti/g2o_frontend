ADD_LIBRARY(thesis ${G2O_FRONTEND_LIB_THESIS}
	DataQueue.cpp DataQueue.h
	Mutexed.cpp Mutexed.h
	PriorityDataQueue.cpp PriorityDataQueue.h
	PrioritySynchronousDataQueue.cpp PrioritySynchronousDataQueue.h
	Sensor.cpp Sensor.h
	SensorHandler.cpp SensorHandler.h
	SensorRGBDCamera.cpp SensorRGBDCamera.h
	SequentialDataQueue.cpp SequentialDataQueue.h
	SequentialSynchronousDataQueue.cpp SequentialSynchronousDataQueue.h
	)

SET_TARGET_PROPERTIES(thesis PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_thesis)

IF(CMAKE_BUILD_TYPE MATCHES Release)
  TARGET_LINK_LIBRARIES(thesis
    ${G2O_TYPES_DATA_LIBRARY}
    ${G2O_TYPES_SLAM2D_LIBRARY}
    ${G2O_STUFF_LIBRARY} 
    ${G2O_CORE_LIBRARY} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${OPENGL_gl_LIBRARY}
  )
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  TARGET_LINK_LIBRARIES(thesis
    ${G2O_TYPES_DATA_LIBRARY_DEBUG}
    ${G2O_TYPES_SLAM2D_LIBRARY_DEBUG}
    ${G2O_STUFF_LIBRARY_DEBUG} 
    ${G2O_CORE_LIBRARY_DEBUG} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${OPENGL_gl_LIBRARY}
  )
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

