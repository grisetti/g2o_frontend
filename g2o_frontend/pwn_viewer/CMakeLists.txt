INCLUDE(${QT_USE_FILE})
SET(UI_HEADERS gl_parameter.h)
QT4_WRAP_CPP(UI_SOURCES ${UI_HEADERS})

FIND_PACKAGE(OpenMP)

if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES})

ADD_LIBRARY(pwn_viewer_library 
  pwn_imageview.cpp pwn_imageview.h
  drawable.cpp drawable.h
  drawable_frame_hac.cpp drawable_frame_hac.h
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

  ${UI_HEADERS}
  ${UI_SOURCES}
)
SET_TARGET_PROPERTIES(pwn_viewer_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn_viewer)
TARGET_LINK_LIBRARIES(pwn_viewer_library
  pwn2_library
  #traverse_library
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
  ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${OpenCV_LIBS}
)


ADD_EXECUTABLE(pwn_simpleViewer pwn_simpleViewer.cpp )
TARGET_LINK_LIBRARIES(pwn_simpleViewer  pwn_viewer_library pwn2_library #traverse_library
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
  ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY})
