INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(pwn_tracker2_library 
  base_tracker.cpp base_tracker.h 
  pwn_cloud_cache.cpp pwn_cloud_cache.h
  pwn_matcher_base.cpp pwn_matcher_base.h
  pwn_tracker.cpp pwn_tracker.h
  pwn_closer.cpp pwn_closer.h
)

TARGET_LINK_LIBRARIES(pwn_tracker2_library 
  pwn_boss
  pwn_core
  boss_map_building 
  boss_map 
  boss
  ${G2O_OPENGL_HELPER_LIBRARY}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_TYPES_SLAM3D} 
  ${G2O_CORE_LIBRARY} 
  ${G2O_STUFF_LIBRARY} 
  ${CSPARSE_LIBRARY})

ADD_executable(boss_pwn_tracker_test boss_pwn_tracker_test.cpp)

TARGET_LINK_LIBRARIES(boss_pwn_tracker_test 
  pwn_tracker2_library
  pwn_boss
  pwn_core
  boss_map_building 
  boss_map 
  boss
  ${G2O_OPENGL_HELPER_LIBRARY}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_TYPES_SLAM3D} 
  ${G2O_CORE_LIBRARY} 
  ${G2O_STUFF_LIBRARY} 
  ${CSPARSE_LIBRARY})
