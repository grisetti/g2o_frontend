FIND_PACKAGE(OpenMP)

if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(pwn2_library
  homogeneousvector4f.h
  informationmatrix.h
  pointprojector.cpp pointprojector.h
  pinholepointprojector.cpp pinholepointprojector.h
  multipointprojector.cpp multipointprojector.h 
  cylindricalpointprojector.cpp cylindricalpointprojector.h
  ../basemath/bm_se3.h
  correspondencefinder.cpp correspondencefinder.h
  depthimage.cpp depthimage.h
  depthimageconverter.cpp depthimageconverter.h
  frame.cpp frame.h
  gaussian3.cpp gaussian3.h
  voxelcalculator.cpp voxelcalculator.h
  aligner.cpp aligner.h
  informationmatrixcalculator.h informationmatrixcalculator.cpp
  linearizer.cpp linearizer.h
  merger.cpp merger.h
  pointaccumulator.h
  pointintegralimage.cpp pointintegralimage.h
  se3_prior.cpp se3_prior.h
  stats.h
  statscalculator.cpp statscalculator.h
)

TARGET_LINK_LIBRARIES(pwn2_library boss ${OpenCV_LIBS})

SET_TARGET_PROPERTIES(pwn2_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn2_library)

# ADD_EXECUTABLE(pwn_boss pwn_boss.cpp)
# TARGET_LINK_LIBRARIES(pwn_boss pwn2_library boss  ${G2O_STUFF_LIBRARY})
