ADD_LIBRARY(ransac ${G2O_FRONTEND_LIB_TYPE}
  ../basemath/bm_se3.h
  ransac.cpp ransac.h
  alignment_horn2d.cpp alignment_horn2d.h
  alignment_horn3d.cpp alignment_horn3d.h
  alignment_se2.cpp alignment_se2.h
  alignment_se3.cpp alignment_se3.h
  alignment_line3d_linear.cpp alignment_line3d_linear.h
	alignment_line2d_linear.cpp alignment_line2d_linear.h
  distance_correspondence_validator.cpp distance_correspondence_validator.h
)

SET_TARGET_PROPERTIES(ransac PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_ransac)


IF(CMAKE_BUILD_TYPE MATCHES Release)
  TARGET_LINK_LIBRARIES(ransac
    ${G2O_TYPES_SLAM2D}
    ${G2O_TYPES_SLAM3D}
    ${G2O_TYPES_SLAM2D_ADDONS}
    ${G2O_TYPES_SLAM3D_ADDONS}
    ${G2O_STUFF_LIBRARY} 
    ${G2O_CORE_LIBRARY} 
  )
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  TARGET_LINK_LIBRARIES(ransac
    ${G2O_TYPES_SLAM2D_DEBUG}
    ${G2O_TYPES_SLA3MD_DEBUG}
    ${G2O_TYPES_SLAM2D_ADDONS_DEBUG}
    ${G2O_TYPES_SLAM3D_ADDONS_DEBUG}
    ${G2O_STUFF_LIBRARY_DEBUG} 
    ${G2O_CORE_LIBRARY_DEBUG} 
  )
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

ADD_EXECUTABLE(alignment_test   alignment_test.cpp)

TARGET_LINK_LIBRARIES(alignment_test
  ransac
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM2D_ADDONS}
  ${G2O_TYPES_SLAM3D_ADDONS}
  ${G2O_STUFF_LIBRARY} 
  ${G2O_CORE_LIBRARY} 
  )
SET_TARGET_PROPERTIES(alignment_test PROPERTIES OUTPUT_NAME g2o_alignment_test${EXE_POSTFIX})

ADD_EXECUTABLE(ransac_test   ransac_test.cpp)

TARGET_LINK_LIBRARIES(ransac_test
  ransac
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM2D_ADDONS}
  ${G2O_TYPES_SLAM3D_ADDONS}
  ${G2O_STUFF_LIBRARY} 
  ${G2O_CORE_LIBRARY} 
  )
SET_TARGET_PROPERTIES(ransac_test PROPERTIES OUTPUT_NAME g2o_ransac_test${EXE_POSTFIX})


ADD_EXECUTABLE(ellipse_descriptor   ellipse_descriptor.cpp)

TARGET_LINK_LIBRARIES(ellipse_descriptor
  ransac
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM2D_ADDONS}
  ${G2O_TYPES_SLAM3D_ADDONS}
  ${G2O_STUFF_LIBRARY} 
  ${G2O_CORE_LIBRARY} 
  )
SET_TARGET_PROPERTIES(ellipse_descriptor PROPERTIES OUTPUT_NAME g2o_ellipse_descriptor${EXE_POSTFIX})
