ADD_EXECUTABLE(
  planeAlignerMultiTest
  planeAlignerMultiTest.cpp
  fileReading.h
  printHelper.h
  planesTransformations.h
  graphUtils.h
  ransacDeclarations.h
)
SET_TARGET_PROPERTIES(planeAlignerMultiTest PROPERTIES OUTPUT_NAME planeAlignerMultiTest)
INCLUDE_DIRECTORIES(planeAlignerMultiTest ${CSPARSE_INCLUDE_DIR} )

TARGET_LINK_LIBRARIES(planeAlignerMultiTest
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
