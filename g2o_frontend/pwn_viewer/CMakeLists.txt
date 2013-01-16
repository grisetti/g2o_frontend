INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES})

ADD_LIBRARY(pwn_viewer_library 
  pwn_imageview.cpp pwn_imageview.h
  drawable.cpp drawable.h
  pwn_qglviewer.cpp pwn_qglviewer.h
  gl_parameter.cpp gl_parameter.h

  drawable_points.cpp drawable_points.h
  gl_parameter_points.cpp gl_parameter_points.h
  
  drawable_normals.cpp drawable_normals.h
  gl_parameter_normals.cpp gl_parameter_normals.h

  gl_parameter_covariances.cpp gl_parameter_covariances.h
  drawable_covariances.cpp drawable_covariances.h

  gl_parameter_correspondences.cpp gl_parameter_correspondences.h
  drawable_correspondences.cpp drawable_correspondences.h
)
SET_TARGET_PROPERTIES(pwn_viewer_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn_viewer)
TARGET_LINK_LIBRARIES(pwn_viewer_library
  pwn_library
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
  ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}
)




