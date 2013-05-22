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
  homogeneousvector4f.h
  informationmatrix.h
  informationmatrixfinder.h informationmatrixfinder.cpp
  linearizer.cpp linearizer.h
  pinholepointprojector.cpp pinholepointprojector.h
  pointaccumulator.h
  pointintegralimage.cpp pointintegralimage.h
  pointprojector.cpp pointprojector.h
  pointstats.h
  se3_prior.cpp se3_prior.h
  statsfinder.cpp statsfinder.h
)

SET_TARGET_PROPERTIES(pwn2_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn2_library)
TARGET_LINK_LIBRARIES(pwn2_library traverse_library)

ADD_EXECUTABLE(pwn_g2o_parser pwn_g2o_parser.cpp)
INCLUDE_DIRECTORIES(pwn_g2o_parser ${CSPARSE_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(
  pwn_g2o_parser
  pwn2_library
  traverse_library
  sensor_data_library
  ${G2O_TYPES_DATA}
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_STUFF_LIBRARY}
  ${G2O_CORE_LIBRARY}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${CSPARSE_LIBRARY}
)
