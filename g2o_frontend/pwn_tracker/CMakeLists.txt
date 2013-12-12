INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES})

ADD_LIBRARY(pwn_tracker_library 
  cache.h cache.hpp
  #map_g2o_wrapper.cpp map_g2o_wrapper.h
  #map_closer.cpp map_closer.h
  pwn_tracker_frame.h pwn_tracker_frame.cpp
  pwn_tracker_relation.h pwn_tracker_relation.cpp
  pwn_tracker_cache.h pwn_tracker_cache.cpp
  #cache.cpp cache.h 
  pwn_tracker.cpp pwn_tracker.h
  pwn_tracker_actions.cpp pwn_tracker_actions.h
  pwn_closer.cpp 
  #pwn_closer_imagecheck.cpp
  pwn_closer.h
  pwn_tracker_viewer.cpp pwn_tracker_viewer.h
)

SET_TARGET_PROPERTIES(pwn_tracker_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn_tracker)



# ADD_EXECUTABLE(pwn_tracker_log_viewer pwn_tracker_log_viewer.cpp)
# TARGET_LINK_LIBRARIES(pwn_tracker_log_viewer pwn_tracker_library pwn_viewer  pwn_core boss_map boss_map_building boss
#     ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
#     ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
#     ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}
#   )


# ADD_EXECUTABLE(pwn_tracker_log_follower pwn_tracker_log_follower.cpp)
# TARGET_LINK_LIBRARIES(pwn_tracker_log_follower pwn_tracker_library pwn_viewer  pwn_core boss_map boss_map_building boss
#     ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
#     ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
#     ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}
#   )


INCLUDE_DIRECTORIES(pwn_tracker_library ${CSPARSE_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(pwn_tracker_library  
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
  ${CSPARSE_LIBRARY}
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
)



ADD_EXECUTABLE(pwn_tracker_app pwn_tracker_app.cpp)

TARGET_LINK_LIBRARIES(pwn_tracker_app pwn_tracker_library pwn_viewer pwn_core boss_map boss_map_building boss
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
  ${CSPARSE_LIBRARY}
  ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} 
  )

