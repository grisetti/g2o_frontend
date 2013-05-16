set(BUILD_SHARED_LIBS OFF) 

CUDA_ADD_LIBRARY(pwn_cuda_library 
  cualigner.cpp cualigner.h
  cudaaligner.cu cudaaligner.cuh  cudaaligner.h
  cudautils.cuh
  cudamatrix.cuh  
  cudasla.cuh
)

TARGET_LINK_LIBRARIES(pwn_cuda_library pwn_library )

SET_TARGET_PROPERTIES(pwn_cuda_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}pwn_cuda_library)

