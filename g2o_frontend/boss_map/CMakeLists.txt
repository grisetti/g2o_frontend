INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

ADD_LIBRARY(boss_map
  stream_processor.cpp stream_processor.h
  linked_tree.cpp linked_tree.h
  reference_frame.cpp reference_frame.h
  reference_frame_relation.cpp  reference_frame_relation.h
  sensor.cpp sensor.h
  laser_sensor.cpp laser_sensor.h
  imu_sensor.cpp imu_sensor.h
  image_sensor.cpp  image_sensor.h
  robot_configuration.cpp robot_configuration.h
  sensor_data_synchronizer.cpp sensor_data_synchronizer2.cpp   sensor_data_synchronizer.h
  map_core.cpp map_core.h
  map_manager.cpp map_manager.h
  local_map.cpp local_map.h
  map_utils.cpp map_utils.h
  #btree.cpp btree.h
  #map_node_processor.cpp map_node_processor.h
  sensor_data_node.cpp sensor_data_node.h  
  sensing_frame_node.cpp sensing_frame_node.h
)

SET_TARGET_PROPERTIES(boss_map PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_boss_map)
TARGET_LINK_LIBRARIES(boss_map ${OpenCV_LIBS} boss)
