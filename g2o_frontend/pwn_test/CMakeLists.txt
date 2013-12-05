IF(CUDA_FOUND)  
  set(BUILD_SHARED_LIBS OFF) 
endif(CUDA_FOUND)

FIND_PACKAGE(OpenMP)

if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${QT_INCLUDES})

ADD_EXECUTABLE(multiprojectortest multiprojectortest.cpp)
SET_TARGET_PROPERTIES(multiprojectortest PROPERTIES OUTPUT_NAME multiprojectortest)
TARGET_LINK_LIBRARIES(multiprojectortest pwn_core ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(fileconvertertest fileconvertertest.cpp)
SET_TARGET_PROPERTIES(fileconvertertest PROPERTIES OUTPUT_NAME fileconvertertest)
TARGET_LINK_LIBRARIES(fileconvertertest pwn_utils)

ADD_EXECUTABLE(cyilindricalprojector_test cyilindricalprojector_test.cpp)
SET_TARGET_PROPERTIES(cyilindricalprojector_test PROPERTIES OUTPUT_NAME cyilindricalprojector_test)
TARGET_LINK_LIBRARIES(cyilindricalprojector_test pwn_core ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(pwn_fast_normal_computation pwn_fast_normal_computation.cpp)
SET_TARGET_PROPERTIES(pwn_fast_normal_computation PROPERTIES OUTPUT_NAME pwn_fast_normal_computation)
TARGET_LINK_LIBRARIES(pwn_fast_normal_computation pwn_core ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(03-align 03-align.cpp)
SET_TARGET_PROPERTIES(03-align PROPERTIES OUTPUT_NAME 03-align)
TARGET_LINK_LIBRARIES(03-align pwn_core ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(pwn_dtcovariance_test pwn_dtcovariance_test.cpp)
SET_TARGET_PROPERTIES(pwn_dtcovariance_test PROPERTIES OUTPUT_NAME pwn_dtcovariance_test)
TARGET_LINK_LIBRARIES(pwn_dtcovariance_test pwn_viewer pwn_core ${G2O_STUFF_LIBRARY})