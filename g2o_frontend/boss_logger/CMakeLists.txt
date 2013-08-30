INCLUDE_DIRECTORIES(. ${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(boss_logger ${G2O_FRONTEND_LIB_TYPE}
  blinked_tree.cpp blinked_tree.h
  bframe.cpp bframe.h
  bframerelation.cpp bframerelation.h
  bsensor.cpp bsensor.h
  bimagesensor.cpp bimagesensor.h
  blasersensor.cpp blasersensor.h
  bimusensor.cpp bimusensor.h
)

TARGET_LINK_LIBRARIES(boss_logger  boss ${OpenCV_LIBS})

ADD_EXECUTABLE(boss_frame_test boss_frame_test.cpp)
TARGET_LINK_LIBRARIES(boss_frame_test boss_logger boss)

ADD_EXECUTABLE(boss_playback boss_playback.cpp )
TARGET_LINK_LIBRARIES(boss_playback boss_logger boss)

ADD_EXECUTABLE(openni2_playback openni2_playback.cpp)
TARGET_LINK_LIBRARIES(openni2_playback boss_logger boss ${OpenCV_LIBS})

