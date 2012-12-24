INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(dm_optimization_library
  dm_cloud.cpp  dm_cloud.h   
  dm_defs.cpp dm_defs.h 
  dm_math.h
  dm_normals.cpp  dm_normals.h 
  dm_solve.cpp dm_solve.h
  dm_utils.cpp dm_utils.h
  dm_cloud_aligner.cpp dm_cloud_aligner.h	
)

SET_TARGET_PROPERTIES(dm_optimization_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_dm_optimization)





