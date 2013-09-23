INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

ADD_EXECUTABLE(pwn_performances_test pwn_performances_test.cpp
  performances_analyzer.cpp performances_analyzer.h
  pwn_performances_analyzer.cpp pwn_performances_analyzer.h)
SET_TARGET_PROPERTIES(pwn_performances_test PROPERTIES OUTPUT_NAME pwn_performances_test)
TARGET_LINK_LIBRARIES(pwn_performances_test
  pwn2_library
  pwn2_utils
  ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}
)

ADD_EXECUTABLE(dvo_performances_test dvo_performances_test.cpp)
SET_TARGET_PROPERTIES(dvo_performances_test PROPERTIES OUTPUT_NAME dvo_performances_test)
TARGET_LINK_LIBRARIES(dvo_performances_test
  pwn2_library
  pwn2_utils
  ${G2O_STUFF_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}
  ${OpenCV_LIBRARIES}
)