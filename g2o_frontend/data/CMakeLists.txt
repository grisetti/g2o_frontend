include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

ADD_LIBRARY(types_data ${G2O_FRONTEND_LIB_TYPE}
  point_cloud_data.cpp point_cloud_data.h
  #image_data.cpp image_data.h
  #rgbd_image_data.cpp rgbd_image_data.h
  depth_registered_data.cpp depth_registered_data.h
  depth_image_data.cpp depth_image_data.h
  feature_data.cpp feature_data.h # IMPORTANT
)

SET_TARGET_PROPERTIES(types_data PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_types_data)



IF(CMAKE_BUILD_TYPE MATCHES Release)
  TARGET_LINK_LIBRARIES(types_data
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
  TARGET_LINK_LIBRARIES(types_data
    ${G2O_TYPES_DATA_LIBRARY_DEBUG}
    ${G2O_TYPES_SLAM2D_LIBRARY_DEBUG}
    ${G2O_STUFF_LIBRARY_DEBUG} 
    ${G2O_CORE_LIBRARY_DEBUG} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${OPENGL_gl_LIBRARY}
  )
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

