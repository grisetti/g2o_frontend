INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(basemath
  bm_defs.h
  bm_se2.h
  bm_se3.h
  multivariate_vector_function.h
  se3_offset_error_function.cpp se3_offset_error_function.h
)

SET_TARGET_PROPERTIES(basemath PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_basemath)





