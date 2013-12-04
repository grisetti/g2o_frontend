INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

ADD_LIBRARY(boss_map_building
  boss_map_g2o_reflector.cpp boss_map_g2o_reflector.h
)

SET_TARGET_PROPERTIES(boss_map_building PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_boss_map_building)
TARGET_LINK_LIBRARIES(boss_map_building ${OpenCV_LIBS} boss boss_map)
