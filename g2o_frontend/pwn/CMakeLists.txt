INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UI_HEADERS base_pwn_window.ui)
QT4_WRAP_CPP(UI_SOURCES pwn_main_window.h)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES})

ADD_LIBRARY(pwn_library
  pwn_cloud.cpp  pwn_cloud.h   
  pwn_defs.cpp pwn_defs.h 
  pwn_math.h
  pwn_normals.cpp  pwn_normals.h 
  pwn_solve.cpp pwn_solve.h
)

SET_TARGET_PROPERTIES(pwn_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}pwn)


ADD_LIBRARY(pwn_viewer_library 
	pwn_qglviewer.cpp pwn_qglviewer.h
#   pwn_imageviewer.cpp pwn_imageviewer.h
#   ${UI_HEADERS} ${UI_SOURCES}
 )

SET_TARGET_PROPERTIES(pwn_viewer_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}pwn_viewer)

TARGET_LINK_LIBRARIES(pwn_viewer_library pwn_library  ${G2O_OPENGL_HELPER_LIBRARY} ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES})

ADD_EXECUTABLE(pwn_test
  pwn_test.cpp
)

TARGET_LINK_LIBRARIES(pwn_test pwn_library pwn_viewer_library ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES})
