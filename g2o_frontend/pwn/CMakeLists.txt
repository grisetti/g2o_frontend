#FIND_PACKAGE(OpenMP)

#if(OPENMP_FOUND)
#  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
#  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif(OPENMP_FOUND)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(pwn_library
  depthimage.cpp depthimage.h
  pixelmapper.cpp pixelmapper.h 
  pointwithnormal.cpp pointwithnormal.h 
  integralpointimage.cpp integralpointimage.h
  pointwithnormalstatsgenerator.cpp pointwithnormalstatsgenerator.h
  pointwithnormalaligner.cpp pointwithnormalaligner.h
  pointwithnormalmerger.cpp pointwithnormalmerger.h
)


SET_TARGET_PROPERTIES(pwn_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn_library)

ADD_EXECUTABLE(pwn_test pwn_test.cpp)
SET_TARGET_PROPERTIES(pwn_test PROPERTIES OUTPUT_NAME pwn_test)
TARGET_LINK_LIBRARIES( pwn_test pwn_library  ${G2O_STUFF_LIBRARY} )

ADD_EXECUTABLE(pwn_align pwn_align.cpp)
SET_TARGET_PROPERTIES(pwn_align PROPERTIES OUTPUT_NAME pwn_align)
TARGET_LINK_LIBRARIES( pwn_align pwn_library  ${G2O_STUFF_LIBRARY} )

ADD_EXECUTABLE(pwn_g2o pwn_g2o.cpp)
SET_TARGET_PROPERTIES(pwn_g2o PROPERTIES OUTPUT_NAME pwn_g2o)
TARGET_LINK_LIBRARIES( pwn_g2o pwn_library  ${G2O_STUFF_LIBRARY} )




