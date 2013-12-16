FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

ADD_LIBRARY(pwn_boss
  aligner.cpp aligner.h
  correspondencefinder.cpp correspondencefinder.h
  cylindricalpointprojector.cpp cylindricalpointprojector.h
  depthimageconverter.cpp depthimageconverter.h
  depthimageconverterintegralimage.cpp depthimageconverterintegralimage.h
  cloud.cpp cloud.h
  informationmatrixcalculator.h informationmatrixcalculator.cpp
  linearizer.cpp linearizer.h
  merger.cpp merger.h
  multipointprojector.cpp multipointprojector.h 
  pinholepointprojector.cpp pinholepointprojector.h
  pointprojector.cpp pointprojector.h
  statscalculator.cpp statscalculator.h
  statscalculatorintegralimage.cpp statscalculatorintegralimage.h
  voxelcalculator.cpp voxelcalculator.h
  pwn_io.cpp pwn_io.h
)
SET_TARGET_PROPERTIES(pwn_boss PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn_boss)
TARGET_LINK_LIBRARIES(pwn_boss ${OpenCV_LIBS} pwn_core boss)
