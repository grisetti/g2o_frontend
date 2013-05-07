set(CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30")
set(BUILD_SHARED_LIBS OFF) 

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

