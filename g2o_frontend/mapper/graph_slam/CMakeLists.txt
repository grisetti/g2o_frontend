INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UI_HEADERS ui.ui)
QT4_WRAP_CPP(UI_SOURCES interface.h)

# find_package(GLUT REQUIRED)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES} ${CSPARSE_INCLUDE_DIR}
 #${GLUT_INCLUDE_DIRS}
)

ADD_LIBRARY(graph_viewer_library
            graph_qglviewer.cpp graph_qglviewer.h
            graph_gui.cpp graph_gui.h
)

SET_TARGET_PROPERTIES(graph_viewer_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_graph_viewer)

TARGET_LINK_LIBRARIES(graph_viewer_library
        ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM2D_ADDONS}
        ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SLAM3D_ADDONS}
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY}
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
  ${PCL_LIBRARIES}
  )


ADD_EXECUTABLE(graph_viewer graph_viewer.cpp)
SET_TARGET_PROPERTIES(graph_viewer PROPERTIES OUTPUT_NAME graph_viewer)
TARGET_LINK_LIBRARIES(graph_viewer
                        graph_viewer_library
                        sensor_data_library
  ${G2O_STUFF_LIBRARY} 	${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${CSPARSE_LIBRARY}
        #${GLUT_LIBRARY}
  #pthread
)


ADD_LIBRARY(graph_slam_library

            graph_slam.cpp      graph_slam.h
)


SET_TARGET_PROPERTIES(graph_slam_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_graph_slam_library)
