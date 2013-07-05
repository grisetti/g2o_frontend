INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(boss_logger ${G2O_FRONTEND_LIB_TYPE}
  bframe.cpp bframe.h
  bframerelation.cpp bframerelation.h
  bsensor.cpp bsensor.h
  bimagesensor.cpp bimagesensor.h
)

