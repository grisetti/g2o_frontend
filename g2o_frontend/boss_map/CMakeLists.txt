INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

ADD_LIBRARY(boss_map
  bframe.cpp bframe.h
  bframerelation.cpp  bframerelation.h
  bimagesensor.cpp  bimagesensor.h
  bimusensor.cpp bimusensor.h
  blasersensor.cpp blasersensor.h
  blinked_tree.cpp blinked_tree.h
  blocalmap.cpp blocalmap.h
  boss_map.cpp boss_map.h
  boss_map_manager.cpp boss_map_manager.h
  boss_map_utils.cpp boss_map_utils.h
  brobot_configuration.cpp brobot_configuration.h
  bsensor.cpp bsensor.h
  bsynchronizer.cpp bsynchronizer.h
  btree.cpp btree.h
  map_node_processor.cpp map_node_processor.h
  sensing_frame_node.cpp sensing_frame_node.h
)

SET_TARGET_PROPERTIES(boss_map PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_boss_map)
TARGET_LINK_LIBRARIES(boss_map ${OpenCV_LIBS} boss)
