INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(pwn_library
  depthimage.cpp depthimage.h
  pixelmapper.cpp pixelmapper.h 
  pointwithnormal.cpp pointwithnormal.h 
  integralpointimage.cpp integralpointimage.h
  pointwithnormalstatsgenerator.cpp pointwithnormalstatsgenerator.h
  pointwithnormalaligner.cpp pointwithnormalaligner.h
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




