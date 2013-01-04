INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(pwn_library
  depthimage.cpp depthimage.h
  pixelmapper.cpp pixelmapper.h 
  pointwithnormal.cpp pointwithnormal.h 
  integralpointimage.cpp integralpointimage.h
  pointwithnormalstatsgenerator.cpp pointwithnormalstatsgenerator.h
)


SET_TARGET_PROPERTIES(pwn_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn_library)

ADD_EXECUTABLE(pwn_test pwn_test.cpp)
SET_TARGET_PROPERTIES(pwn_test PROPERTIES OUTPUT_NAME pwn_test)
TARGET_LINK_LIBRARIES( pwn_test pwn_library  ${G2O_STUFF_LIBRARY} )




