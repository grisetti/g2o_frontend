INCLUDE_DIRECTORIES(. ${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(boss_logger ${G2O_FRONTEND_LIB_TYPE}
  blinked_tree.cpp blinked_tree.h
  bframe.cpp bframe.h
  bframerelation.cpp bframerelation.h
  bsensor.cpp bsensor.h
  bimagesensor.cpp bimagesensor.h
  blasersensor.cpp blasersensor.h
  bimusensor.cpp bimusensor.h
  bsynchronizer.cpp bsynchronizer.h
  brobot_configuration.cpp brobot_configuration.h
)

TARGET_LINK_LIBRARIES(boss_logger  boss ${OpenCV_LIBS})

ADD_EXECUTABLE(boss_frame_test boss_frame_test.cpp)
TARGET_LINK_LIBRARIES(boss_frame_test pwn_boss pwn2_library boss_logger boss)

ADD_EXECUTABLE(boss_playback boss_playback.cpp )
TARGET_LINK_LIBRARIES(boss_playback pwn_boss pwn2_library boss_logger boss)

ADD_EXECUTABLE(boss_synchronizer boss_synchronizer.cpp )
TARGET_LINK_LIBRARIES(boss_synchronizer pwn_boss pwn2_library boss_logger boss)

ADD_EXECUTABLE(boss_path_generator boss_path_generator.cpp )
TARGET_LINK_LIBRARIES(boss_path_generator pwn2_library pwn_boss boss_logger boss)



