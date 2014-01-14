INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_EXECUTABLE(boss_sample boss_sample.cpp)
TARGET_LINK_LIBRARIES(boss_sample boss)

ADD_EXECUTABLE(boss_frame_test boss_frame_test.cpp)
TARGET_LINK_LIBRARIES(boss_frame_test pwn_boss pwn_core boss_map boss)

ADD_EXECUTABLE(boss_playback boss_playback.cpp )
TARGET_LINK_LIBRARIES(boss_playback pwn_boss pwn_core boss_map boss)

# ADD_EXECUTABLE(boss_synchronizer boss_synchronizer.cpp )
# TARGET_LINK_LIBRARIES(boss_synchronizer pwn_boss pwn_core boss_map boss)

ADD_EXECUTABLE(boss_synchronizer2 boss_synchronizer2.cpp )
TARGET_LINK_LIBRARIES(boss_synchronizer2 pwn_boss pwn_core boss_map boss)

# ADD_EXECUTABLE(boss_path_generator boss_path_generator.cpp )
# TARGET_LINK_LIBRARIES(boss_path_generator pwn_core pwn_boss boss_map boss)

# ADD_executable(boss_sensing_frame_test boss_sensing_frame_test.cpp)
# TARGET_LINK_LIBRARIES(boss_sensing_frame_test boss_map boss_map  boss ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY})


# ADD_executable(boss_synchronizer_framegen boss_synchronizer_framegen.cpp)
# TARGET_LINK_LIBRARIES(boss_synchronizer_framegen boss_map boss_map  boss ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY})
