FIND_PACKAGE(OpenMP)

if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(pwn2_library
  aligner.cpp aligner.h
  ../basemath/bm_se3.h
  correspondencefinder.cpp correspondencefinder.h
  depthimage.cpp depthimage.h
  depthimageconverter.cpp depthimageconverter.h
  frame.cpp frame.h
  gaussian3.cpp gaussian3.h
  g2o_frame.h
  homogeneousvector4f.h
  informationmatrix.h
  informationmatrixcalculator.h informationmatrixcalculator.cpp
  linearizer.cpp linearizer.h
  merger.cpp merger.h
  pinholepointprojector.cpp pinholepointprojector.h
  pointaccumulator.h
  pointintegralimage.cpp pointintegralimage.h
  pointprojector.cpp pointprojector.h
  se3_prior.cpp se3_prior.h
  stats.h
  statscalculator.cpp statscalculator.h
)

SET_TARGET_PROPERTIES(pwn2_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn2_library)