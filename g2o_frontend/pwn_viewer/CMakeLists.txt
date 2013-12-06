INCLUDE(${QT_USE_FILE})
SET(UI_HEADERS gl_parameter.h)
QT4_WRAP_CPP(UI_SOURCES ${UI_HEADERS})

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${QT_INCLUDES} ${QGLVIEWER_INCLUDE_DIR})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

ADD_LIBRARY(pwn_viewer 
  drawable.cpp drawable.h
  drawable_correspondences.cpp drawable_correspondences.h
  drawable_covariances.cpp drawable_covariances.h
  drawable_frame.cpp drawable_frame.h
  drawable_normals.cpp drawable_normals.h
  drawable_points.cpp drawable_points.h
  drawable_trajectory.cpp drawable_trajectory.h
  drawable_transform_covariance.cpp drawable_transform_covariance.h
  gl_parameter.cpp gl_parameter.h
  gl_parameter_correspondences.cpp gl_parameter_correspondences.h
  gl_parameter_covariances.cpp gl_parameter_covariances.h
  gl_parameter_frame.cpp gl_parameter_frame.h 
  gl_parameter_normals.cpp gl_parameter_normals.h
  gl_parameter_points.cpp gl_parameter_points.h
  gl_parameter_trajectory.cpp gl_parameter_trajectory.h 
  gl_parameter_transform_covariance.cpp gl_parameter_transform_covariance.h
  pwn_imageview.cpp pwn_imageview.h
  pwn_qglviewer.cpp pwn_qglviewer.h
  ${UI_HEADERS}
  ${UI_SOURCES}
)
SET_TARGET_PROPERTIES(pwn_viewer PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn_viewer)

TARGET_LINK_LIBRARIES(pwn_viewer pwn_core ${OpenCV_LIBS} ${QGLVIEWER_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY})