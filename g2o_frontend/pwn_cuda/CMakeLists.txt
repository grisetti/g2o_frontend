FIND_PACKAGE(OpenMP)

if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

find_package(CUDA)
if(CUDA_FOUND)
  set(CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30")
  set(BUILD_SHARED_LIBS OFF) 
endif(CUDA_FOUND)

CUDA_ADD_LIBRARY(pwn_cuda_library 
  cudaaligner.cu
  cudaaligner.h
  cudaaligner.cuh  
  cudamatrix.cuh  
  cudaptr.cuh  
  cudasla.cuh
  cualigner.cpp cualigner.h
)

TARGET_LINK_LIBRARIES(pwn_cuda_library pwn_library )

SET_TARGET_PROPERTIES(pwn_cuda_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}pwn_cuda_library)

