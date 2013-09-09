INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

ADD_EXECUTABLE(pwn_performances_analyzer pwn_performances_analyzer.cpp)
SET_TARGET_PROPERTIES(pwn_performances_analyzer PROPERTIES OUTPUT_NAME pwn_performances_analyzer)
TARGET_LINK_LIBRARIES(pwn_performances_analyzer
  pwn2_library
  pwn2_utils
  ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}
)

ADD_EXECUTABLE(pwnnn_performances_analyzer pwnnn_performances_analyzer.cpp)
SET_TARGET_PROPERTIES(pwnnn_performances_analyzer PROPERTIES OUTPUT_NAME pwnnn_performances_analyzer)
TARGET_LINK_LIBRARIES(pwnnn_performances_analyzer
  pwn2_library
  pwn2_utils
  ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}
)

ADD_EXECUTABLE(icp_performances_analyzer icp_performances_analyzer.cpp)
SET_TARGET_PROPERTIES(icp_performances_analyzer PROPERTIES OUTPUT_NAME icp_performances_analyzer)
TARGET_LINK_LIBRARIES(icp_performances_analyzer
  pwn2_library
  pwn2_utils
  ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}
  ${PCL_LIBRARIES}
)

ADD_EXECUTABLE(dvo_performances_analyzer dvo_performances_analyzer.cpp)
SET_TARGET_PROPERTIES(dvo_performances_analyzer PROPERTIES OUTPUT_NAME dvo_performances_analyzer)
TARGET_LINK_LIBRARIES(dvo_performances_analyzer
  pwn2_library
  pwn2_utils
  ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}
  ${OpenCV_LIBRARIES}
)