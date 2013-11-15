FIND_PACKAGE(OpenMP)

if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(boss_map  
  boss_map.cpp boss_map.h
  boss_map_manager.cpp boss_map_manager.h
  sensing_frame_node.cpp sensing_frame_node.h
  map_node_processor.cpp map_node_processor.h 
  boss_map_utils.cpp
  boss_map_g2o_reflector.cpp boss_map_g2o_reflector.h
  )

SET_TARGET_PROPERTIES(boss_map PROPERTIES OUTPUT_NAME boss_map)

TARGET_LINK_LIBRARIES(boss_map boss_logger  boss ${G2O_TYPES_SLAM3D}  ${G2O_CORE_LIBRARY}  )

ADD_executable(boss_sensing_frame_test boss_sensing_frame_test.cpp)

SET_TARGET_PROPERTIES(boss_sensing_frame_test PROPERTIES OUTPUT_NAME boss_sensing_frame_test)

TARGET_LINK_LIBRARIES(boss_sensing_frame_test boss_map boss_logger  boss ${G2O_TYPES_SLAM3D}  ${G2O_CORE_LIBRARY} )

