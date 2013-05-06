FIND_PACKAGE(OpenMP)

if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})


find_package(CUDA)
if(CUDA_FOUND)
  set(CUDA_NVCC_FLAGS "-O3 -gencode arch=compute_30,code=sm_30")
  set(BUILD_SHARED_LIBS OFF) 
  CUDA_ADD_LIBRARY(pwn_aligner_cuda_library cudaaligner.cu)
endif(CUDA_FOUND)

ADD_LIBRARY(pwn_library
	../basemath/bm_se3.h
	aligner.cpp aligner.h
  depthimage.cpp depthimage.h
  pointprojector.cpp pointprojector.h
  pinholepointprojector.cpp pinholepointprojector.h
  homogeneouspoint3fintegralimage.cpp homogeneouspoint3fintegralimage.h
  normalgenerator.h	
  #homogeneouspoint3fscene.cpp 
  homogeneouspoint3fscene.h
  homogeneouspoint3fstatsgenerator.cpp homogeneouspoint3fstatsgenerator.h
  homogeneouspoint3faccumulator.h
  homogeneouspoint3fstats.h
  homogeneouspoint3fomega.h
  omegagenerator.h omegagenerator.cpp
  homogeneousvector4f.h
  pixelmapper.cpp pixelmapper.h 
  gaussian3.cpp gaussian3.h 
  pointwithnormal.cpp pointwithnormal.h 
  integralpointimage.cpp integralpointimage.h
  linearizer.cpp linearizer.h
  pointwithnormalstatsgenerator.cpp pointwithnormalstatsgenerator.h
  pointwithnormalaligner.cpp pointwithnormalaligner.h
  pointwithnormalmerger.cpp pointwithnormalmerger.h
  scene.cpp scene.h
  scene_merger.cpp scene_merger.h
  correspondencegenerator.cpp correspondencegenerator.h 
)

SET_TARGET_PROPERTIES(pwn_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}pwn_library)
TARGET_LINK_LIBRARIES( pwn_library pwn_aligner_cuda_library)

ADD_EXECUTABLE(pwn_test pwn_test.cpp)
SET_TARGET_PROPERTIES(pwn_test PROPERTIES OUTPUT_NAME pwn_test)
TARGET_LINK_LIBRARIES( pwn_test pwn_library ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(pwn_align pwn_align.cpp)
SET_TARGET_PROPERTIES(pwn_align PROPERTIES OUTPUT_NAME pwn_align)
TARGET_LINK_LIBRARIES( pwn_align pwn_library  ${G2O_STUFF_LIBRARY} )

ADD_EXECUTABLE(pwn_g2o pwn_g2o.cpp)
SET_TARGET_PROPERTIES(pwn_g2o PROPERTIES OUTPUT_NAME pwn_g2o)
TARGET_LINK_LIBRARIES( pwn_g2o pwn_library  ${G2O_STUFF_LIBRARY} )

ADD_EXECUTABLE(pwn_g3o pwn_g3o.cpp)
SET_TARGET_PROPERTIES(pwn_g3o PROPERTIES OUTPUT_NAME pwn_g3o)
TARGET_LINK_LIBRARIES( pwn_g3o pwn_library  ${G2O_STUFF_LIBRARY} )

ADD_EXECUTABLE(pwn_g2o_localmap pwn_g2o_localmap.cpp)
SET_TARGET_PROPERTIES(pwn_g2o_localmap PROPERTIES OUTPUT_NAME pwn_g2o_localmap)
TARGET_LINK_LIBRARIES( pwn_g2o_localmap pwn_library  ${G2O_STUFF_LIBRARY} )

ADD_EXECUTABLE(pwn_g2o_movingRef pwn_g2o_movingRef.cpp)
SET_TARGET_PROPERTIES(pwn_g2o_movingRef PROPERTIES OUTPUT_NAME pwn_g2o_movingRef)
TARGET_LINK_LIBRARIES( pwn_g2o_movingRef pwn_library  ${G2O_STUFF_LIBRARY} )

ADD_EXECUTABLE(homogeneous_normals homogeneous_normals.cpp)
SET_TARGET_PROPERTIES(homogeneous_normals PROPERTIES OUTPUT_NAME homogeneous_normals)
TARGET_LINK_LIBRARIES( homogeneous_normals pwn_library  ${G2O_STUFF_LIBRARY} pwn_aligner_cuda_library ) 

