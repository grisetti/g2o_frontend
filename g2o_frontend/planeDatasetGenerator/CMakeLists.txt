ADD_EXECUTABLE(
  planeDatasetGenerator
  planeDatasetGenerator.cpp
  fileReading.h
  printHelper.h
  planesTransformations.h
  graphUtils.h
  ransacDeclarations.h
)
SET_TARGET_PROPERTIES(planeDatasetGenerator PROPERTIES OUTPUT_NAME planeDatasetGenerator)
INCLUDE_DIRECTORIES(planeDatasetGenerator ${CSPARSE_INCLUDE_DIR} )

TARGET_LINK_LIBRARIES(planeDatasetGenerator
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
