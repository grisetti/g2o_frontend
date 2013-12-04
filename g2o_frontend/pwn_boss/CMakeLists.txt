FIND_PACKAGE(OpenMP)

if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(pwn_boss
  pwn_sensor_data.cpp pwn_sensor_data.h
  two_depthimage_aligner_node.cpp two_depthimage_aligner_node.h 
  )

SET_TARGET_PROPERTIES(pwn_boss PROPERTIES OUTPUT_NAME pwn_boss)
TARGET_LINK_LIBRARIES(pwn_boss pwn2_library boss boss_map ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(pwn_boss_make_pwns pwn_boss_make_pwns.cpp)
TARGET_LINK_LIBRARIES(pwn_boss_make_pwns pwn_boss pwn2_library boss_map boss ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(pwn_boss_align_two_images pwn_boss_align_two_images.cpp)
TARGET_LINK_LIBRARIES(pwn_boss_align_two_images pwn_boss pwn2_library boss_map boss ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(pwn_boss_process_dir pwn_boss_process_dir.cpp)
TARGET_LINK_LIBRARIES(pwn_boss_process_dir pwn2_library boss_map boss ${G2O_STUFF_LIBRARY})

# ADD_EXECUTABLE(pwn_boss_process_log pwn_boss_process_log.cpp)
# TARGET_LINK_LIBRARIES(pwn_boss_process_log  pwn2_library boss_map boss ${G2O_STUFF_LIBRARY})



