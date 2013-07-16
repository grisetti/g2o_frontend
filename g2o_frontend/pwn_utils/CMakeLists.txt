INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(pwn2_utils
  pwn_file_format_converter.h
  pwn_utils.h
)

SET_TARGET_PROPERTIES(pwn2_utils PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_pwn2_utils LINKER_LANGUAGE CXX)

TARGET_LINK_LIBRARIES(pwn2_utils
  pwn2_library
  ${PCL_LIBRARIES}
)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES})

ADD_EXECUTABLE(pwn_calibration pwn_calibration.cpp)
SET_TARGET_PROPERTIES(pwn_calibration PROPERTIES OUTPUT_NAME pwn_calibration)
TARGET_LINK_LIBRARIES(pwn_calibration pwn_viewer_library pwn2_library ${OpenCV_LIBS} ${G2O_STUFF_LIBRARY})