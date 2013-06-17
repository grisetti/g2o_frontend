INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UI_HEADERS ui.ui)
QT4_WRAP_CPP(UI_SOURCES interface.h)

# find_package(GLUT REQUIRED)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES} ${CSPARSE_INCLUDE_DIR} 
 ${GLUT_INCLUDE_DIRS}
)

ADD_LIBRARY(map_viewer_library
            map_qglviewer.cpp map_qglviewer.h
            map_gui.cpp map_gui.h
)

SET_TARGET_PROPERTIES(map_viewer_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_map_viewer)

TARGET_LINK_LIBRARIES(map_viewer_library
        ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM2D_ADDONS}
        ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SLAM3D_ADDONS}
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
  ${PCL_LIBRARIES}
  )


ADD_EXECUTABLE(map_viewer map_viewer.cpp)
SET_TARGET_PROPERTIES(map_viewer PROPERTIES OUTPUT_NAME map_viewer)
TARGET_LINK_LIBRARIES(map_viewer
                        map_viewer_library
                        sensor_data_library
  ${G2O_STUFF_LIBRARY} 	${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${CSPARSE_LIBRARY}
	#${GLUT_LIBRARY}
  #pthread
)


ADD_EXECUTABLE(manipulated_frame try_manipulated_frame.cpp manipulated_frame.cpp)
SET_TARGET_PROPERTIES(manipulated_frame PROPERTIES OUTPUT_NAME manipulated_frame)
TARGET_LINK_LIBRARIES(manipulated_frame
                        map_viewer_library
                        sensor_data_library
  ${G2O_STUFF_LIBRARY} 	${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${CSPARSE_LIBRARY}
        #${GLUT_LIBRARY}
  #pthread
)

ADD_EXECUTABLE(selection try_select.cpp select.cpp)
SET_TARGET_PROPERTIES(selection PROPERTIES OUTPUT_NAME selection)
TARGET_LINK_LIBRARIES(selection
                        map_viewer_library
                        sensor_data_library
  ${G2O_STUFF_LIBRARY} 	${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${CSPARSE_LIBRARY}
        #${GLUT_LIBRARY}
  #pthread
)


ADD_EXECUTABLE(multiSelection try_multiSelect.cpp manipulatedFrameSetConstraint.cpp drawable_object.cpp multiSelect.cpp)
SET_TARGET_PROPERTIES(multiSelection PROPERTIES OUTPUT_NAME multiSelection)
TARGET_LINK_LIBRARIES(multiSelection
                        map_viewer_library
                        sensor_data_library
  ${G2O_STUFF_LIBRARY} 	${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${CSPARSE_LIBRARY}
        #${GLUT_LIBRARY}
  #pthread
)
