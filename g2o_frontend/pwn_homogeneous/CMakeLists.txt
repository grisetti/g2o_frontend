FIND_PACKAGE(OpenMP)

if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(pwn_homogeneous_library
  aligner.h aligner.cpp
  depthimage.h depthimage.cpp
	homogeneouspoint3faccumulator.h
	homogeneouspoint3fintegralimage.h homogeneouspoint3fintegralimage.cpp
	homogeneouspoint3fstats.h
	homogeneouspoint3fstatsgenerator.h homogeneouspoint3fstatsgenerator.cpp
	homogeneousvector4f.h
	linearizer.h linearizer.cpp
	pinholepointprojector.h pinholepointprojector.cpp
	pixelmapper.h pixelmapper.cpp
	pointprojector.h pointprojector.cpp
	pointwithnormal.h pointwithnormal.cpp
	transformable_vector.h
)

SET_TARGET_PROPERTIES(pwn_homogeneous_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}pwn_homogeneous_library)
