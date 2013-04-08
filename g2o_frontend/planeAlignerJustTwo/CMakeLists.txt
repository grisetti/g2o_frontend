
ADD_EXECUTABLE(
  plane_alignerForTwo
  plane_aligner.cpp
)
SET_TARGET_PROPERTIES(plane_alignerForTwo PROPERTIES OUTPUT_NAME plane_alignerForTwo)
INCLUDE_DIRECTORIES(plane_alignerForTwo ${CSPARSE_INCLUDE_DIR} )

TARGET_LINK_LIBRARIES(plane_alignerForTwo
    sensor_data_library
    types_data
    ransac
    ${QT_LIBRARIES}
    ${G2O_TYPES_SLAM2D}
    ${G2O_TYPES_SLAM2D_ADDONS}
    ${G2O_TYPES_SLAM3D}
    ${G2O_TYPES_SLAM3D_ADDONS}
    ${G2O_TYPES_DATA}
    ${G2O_CORE_LIBRARY}
    ${G2O_SOLVER_CSPARSE}
    ${G2O_STUFF_LIBRARY}
    ${G2O_SOLVER_CSPARSE_EXTENSION}
    ${CSPARSE_LIBRARY}
  )
