FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

ADD_LIBRARY(pwn_core
  pwn_typedefs.h
  pwn_static.cpp pwn_static.h
  aligner.cpp aligner.h
  correspondencefinder.cpp correspondencefinder.h
  cylindricalpointprojector.cpp cylindricalpointprojector.h
  depthimageconverter.cpp depthimageconverter.h
  depthimageconverterintegralimage.cpp depthimageconverterintegralimage.h
  cloud.cpp cloud.h
  gaussian3.cpp gaussian3.h
  homogeneousvector4f.h
  informationmatrix.h
  informationmatrixcalculator.h informationmatrixcalculator.cpp
  linearizer.cpp linearizer.h
  merger.cpp merger.h
  multipointprojector.cpp multipointprojector.h 
  pinholepointprojector.cpp pinholepointprojector.h
  pointaccumulator.h
  pointintegralimage.cpp pointintegralimage.h
  pointprojector.cpp pointprojector.h
  se3_prior.cpp se3_prior.h
  stats.h
  statscalculator.cpp statscalculator.h
  statscalculatorintegralimage.cpp statscalculatorintegralimage.h
  voxelcalculator.cpp voxelcalculator.h
)
SET_TARGET_PROPERTIES(pwn_core PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn_core)
TARGET_LINK_LIBRARIES(pwn_core ${OpenCV_LIBS})

ADD_EXECUTABLE(pwn_simple_aligner pwn_simple_aligner.cpp )
SET_TARGET_PROPERTIES(pwn_simple_aligner PROPERTIES OUTPUT_NAME pwn_simple_aligner)
TARGET_LINK_LIBRARIES(pwn_simple_aligner pwn_core ${OpenCV_LIBS})

ADD_EXECUTABLE(pwn_aligner pwn_aligner.cpp )
SET_TARGET_PROPERTIES(pwn_aligner PROPERTIES OUTPUT_NAME pwn_aligner)
TARGET_LINK_LIBRARIES(pwn_aligner pwn_core ${OpenCV_LIBS})