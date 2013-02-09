INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UI_HEADERS viewer.ui)
QT4_WRAP_CPP(UI_SOURCES viewer.h)

# find_package(GLUT REQUIRED)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES} ${CSPARSE_INCLUDE_DIR} 
 #${GLUT_INCLUDE_DIRS}
)

ADD_LIBRARY(line_extraction_library
  Edge.cpp Edge.h
  IEdgesExtractor.cpp IEdgesExtractor.h
  leQGLViewer.cpp leQGLViewer.h
  line_extraction2d.cpp line_extraction2d.h
  #moc_viewerGUI.cpp
  RansacEE.cpp RansacEE.h
  SplitMergeEE.cpp SplitMergeEE.h
  viewerGUI.cpp viewerGUI.h
  #viewer.h
)

SET_TARGET_PROPERTIES(line_extraction_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_line_extraction)

TARGET_LINK_LIBRARIES(line_extraction_library
  sensor_data_library
	${G2O_TYPES_SLAM2D_ADDONS}
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
  ${PCL_LIBRARIES}
  )


ADD_EXECUTABLE(lineEx_viewer lineEx_viewer.cpp)
SET_TARGET_PROPERTIES(lineEx_viewer PROPERTIES OUTPUT_NAME lineEx_viewer)
TARGET_LINK_LIBRARIES( lineEx_viewer 
  line_extraction_library
  sensor_data_library
  ${G2O_STUFF_LIBRARY} 	${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${CSPARSE_LIBRARY} 
	#${GLUT_LIBRARY}
  #pthread
)

ADD_EXECUTABLE(toGraphSE2 toGraphSE2.cpp)
SET_TARGET_PROPERTIES(toGraphSE2 PROPERTIES OUTPUT_NAME toGraphSE2)
TARGET_LINK_LIBRARIES( toGraphSE2 
  line_extraction_library
  sensor_data_library
  types_data
  ${G2O_STUFF_LIBRARY} 	${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${CSPARSE_LIBRARY} 

)

# ADD_EXECUTABLE(lineEx_viewer_old lineEx_viewer_old.cpp)
# SET_TARGET_PROPERTIES(lineEx_viewer_old PROPERTIES OUTPUT_NAME lineEx_viewer_old)
# TARGET_LINK_LIBRARIES( lineEx_viewer_old 
#   line_extraction_library
#   thesis_library
#   ${G2O_STUFF_LIBRARY} 	${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
#   ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${CSPARSE_LIBRARY} 
#   #pthread
# )


