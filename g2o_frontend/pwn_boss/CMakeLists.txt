FIND_PACKAGE(OpenMP)

if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})


ADD_EXECUTABLE(pwn_boss_process_dir pwn_boss_process_dir.cpp)
TARGET_LINK_LIBRARIES(pwn_boss_process_dir  pwn2_library boss_map boss_logger  boss  ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(pwn_boss_process_log pwn_boss_process_log.cpp)
TARGET_LINK_LIBRARIES(pwn_boss_process_log  pwn2_library boss_map boss_logger  boss ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(pwn_boss_add_sensing_frames pwn_boss_add_sensing_frames.cpp)
TARGET_LINK_LIBRARIES(pwn_boss_add_sensing_frames  pwn2_library boss_map boss_logger  boss ${G2O_STUFF_LIBRARY})

