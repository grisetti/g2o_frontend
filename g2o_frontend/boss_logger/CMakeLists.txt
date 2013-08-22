INCLUDE_DIRECTORIES(. ${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(boss_logger ${G2O_FRONTEND_LIB_TYPE}
  bframe.cpp bframe.h
  bframerelation.cpp bframerelation.h
  bsensor.cpp bsensor.h
  bimagesensor.cpp bimagesensor.h
)

TARGET_LINK_LIBRARIES(boss_logger  boss ${OpenCV_LIBS})


ADD_EXECUTABLE(boss_frame_test boss_frame_test.cpp)
TARGET_LINK_LIBRARIES(boss_frame_test boss_logger boss)

ADD_EXECUTABLE(openni2_playback openni2_playback.cpp)
TARGET_LINK_LIBRARIES(openni2_playback boss_logger boss ${OpenCV_LIBS})

