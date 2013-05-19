ADD_LIBRARY(types_data ${G2O_FRONTEND_LIB_TYPE}
  depth_image_data.cpp depth_image_data.h
  feature_data.cpp feature_data.h # IMPORTANT
)

SET_TARGET_PROPERTIES(types_data PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_types_data)


IF(CMAKE_BUILD_TYPE MATCHES Release)
  TARGET_LINK_LIBRARIES(types_data
    ${G2O_TYPES_DATA}
    ${G2O_TYPES_SLAM2D}
    ${G2O_STUFF_LIBRARY} 
    ${G2O_CORE_LIBRARY} 
    ${OpenCV_LIBS}
    ${OPENGL_gl_LIBRARY}
    pwn_library
  )
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  TARGET_LINK_LIBRARIES(types_data
    ${G2O_TYPES_DATA_DEBUG}
    ${G2O_TYPES_SLAM2D_DEBUG}
    ${G2O_STUFF_LIBRARY_DEBUG} 
    ${G2O_CORE_LIBRARY_DEBUG} 
    ${OpenCV_LIBS}
    ${OPENGL_gl_LIBRARY}
    pwn_library
  )
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

