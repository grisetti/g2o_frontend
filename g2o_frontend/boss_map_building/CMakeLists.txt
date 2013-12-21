INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

ADD_LIBRARY(boss_map_building
  map_g2o_reflector.cpp map_g2o_reflector.h
  map_closer.cpp map_closer.h
  base_tracker.cpp base_tracker.h
  cache.hpp cache.h
  #map_g2o_wrapper.cpp map_g2o_wrapper.h
)

INCLUDE_DIRECTORIES(boss_map_building  ${CSPARSE_INCLUDE_DIR})

SET_TARGET_PROPERTIES(boss_map_building PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_boss_map_building)

TARGET_LINK_LIBRARIES(boss_map_building 
  boss 
  boss_map 
  ${G2O_TYPES_SLAM3D} 
  ${G2O_CORE_LIBRARY} 
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_OPENGL_HELPER_LIBRARY}
  ${CSPARSE_LIBRARY}
  ${OpenCV_LIBS}
)
 
  