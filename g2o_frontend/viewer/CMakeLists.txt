INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UI_HEADERS base_pwn_window.ui)
QT4_WRAP_CPP(UI_SOURCES pwn_main_window.h)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES})

ADD_LIBRARY(dm_viewer_library 
	dm_qglviewer.cpp dm_qglviewer.h
	drawable.cpp drawable.h
	drawable_points.cpp drawable_points.h
	drawable_normals.cpp drawable_normals.h
	gl_parameter.cpp gl_parameter.h
	gl_parameter_points.cpp gl_parameter_points.h
	gl_parameter_normals.cpp gl_parameter_normals.h
	gl_parameter_covariances.cpp gl_parameter_covariances.h
)
SET_TARGET_PROPERTIES(dm_viewer_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_dm_viewer)
TARGET_LINK_LIBRARIES(dm_viewer_library ${G2O_OPENGL_HELPER_LIBRARY} ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} ${OPENGL_gl_LIBRARY})
