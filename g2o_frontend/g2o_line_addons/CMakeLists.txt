cmake_minimum_required(VERSION 2.8)
include_directories(${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(g2o_line_addons_library 
  vertex_extreme_point_xy.cpp vertex_extreme_point_xy.h
  g2o_line_addons.cpp g2o_line_addons.h
)

TARGET_LINK_LIBRARIES(g2o_line_addons_library 
  ${G2O_TYPES_SLAM2D}
  ${G2O_STUFF_LIBRARY}
  ${G2O_CORE_LIBRARY}
  ${OpenCV_LIBS}
  ${OPENGL_gl_LIBRARY}
  )

SET_TARGET_PROPERTIES(g2o_line_addons_library  PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_g2o_line_addons)


