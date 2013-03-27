INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(graph_slam_library

            graph_slam.cpp      graph_slam.h
)


SET_TARGET_PROPERTIES(graph_slam_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_graph_slam_library)
