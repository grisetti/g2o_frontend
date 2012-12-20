INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UI_HEADERS dm_base_main_window.ui)
QT4_WRAP_CPP(UI_SOURCES dm_main_window.h)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES})

ADD_LIBRARY(dm_viewer_library 
	#dm_qglviewer.cpp dm_qglviewer.h
	#dm_main_window.cpp dm_main_window.h 
	drawable.cpp drawable.h
	drawable_points.cpp drawable_points.h
	drawable_normals.cpp drawable_normals.h
	drawable_covariances.cpp drawable_covariances.h
	drawable_correspondences.cpp drawable_correspondences.h
	gl_parameter.cpp gl_parameter.h
	gl_parameter_points.cpp gl_parameter_points.h
	gl_parameter_normals.cpp gl_parameter_normals.h
	gl_parameter_covariances.cpp gl_parameter_covariances.h
	gl_parameter_correspondences.cpp gl_parameter_correspondences.h
)
SET_TARGET_PROPERTIES(dm_viewer_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_dm_viewer)
TARGET_LINK_LIBRARIES(dm_viewer_library ${G2O_OPENGL_HELPER_LIBRARY} ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${OPENGL_gl_LIBRARY})

ADD_EXECUTABLE(dm_qglviewer_test
  dm_qglviewer.cpp 
  dm_qglviewer.h
  dm_main_window.cpp
  dm_qglviewer_test.cpp
  ${UI_HEADERS}
  ${UI_SOURCES}
)

SET_TARGET_PROPERTIES(dm_qglviewer_test PROPERTIES OUTPUT_NAME dm_qglviewer_test)

INCLUDE_DIRECTORIES(${QGLVIEWER_INCLUDE_DIR} ${CMAKE_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR}  ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}  ${QT_QTCORE_INCLUDE_DIR}/..)

TARGET_LINK_LIBRARIES(dm_qglviewer_test pwn_library dm_viewer_library pwn_viewer_library ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY})
