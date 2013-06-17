INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES} ${CSPARSE_INCLUDE_DIR})


ADD_LIBRARY(graph_slam_library

            graph_slam.cpp      graph_slam.h
            sub_map.cpp         sub_map.h
)


SET_TARGET_PROPERTIES(graph_slam_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_graph_slam_library)
TARGET_LINK_LIBRARIES(graph_slam_library
                       ${CSPARSE_LIBRARY}
#${G2O_TYPES_DATA}
${G2O_SOLVER_CSPARSE_EXTENSION}
${G2O_SOLVER_CSPARSE}
${G2O_CORE_LIBRARY}
#${G2O_STUFF_LIBRARY}
#${G2O_TYPES_SLAM2D}
)

ADD_EXECUTABLE(graph_slam main.cpp)
SET_TARGET_PROPERTIES(graph_slam PROPERTIES OUTPUT_NAME graph_slam)
TARGET_LINK_LIBRARIES(graph_slam
                      graph_slam_library
                      graph_viewer_library
                      matching_library
                      sensor_data_library
  ${G2O_STUFF_LIBRARY} 	${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${CSPARSE_LIBRARY}
        #${GLUT_LIBRARY}
  #pthread
)
