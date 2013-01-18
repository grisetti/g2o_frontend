# ADD_LIBRARY(plane_library ${G2O_FRONTEND_LIB_TYPE}
#   CloudUtils.cpp
#   CloudUtils.h
#   interface.cpp
#   interface.h
#   main.cpp
#   OverridedViewer.cpp
#   OverridedViewer.h
#   viewer_slider.h
# )

# SET_TARGET_PROPERTIES(plane_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_slam)


# TARGET_LINK_LIBRARIES(plane_library
#   types_data
#   ${G2O_TYPES_SLAM2D}
#   ${G2O_TYPES_SLAM3D}
#   ${G2O_TYPES_DATA}
#   ${G2O_CORE_LIBRARY}
#   ${G2O_STUFF_LIBRARY}
# )


ADD_EXECUTABLE(plane_g2o_application
  plane_g2o.cpp
)

INCLUDE_DIRECTORIES(plane_g2o_application ${CSPARSE_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(plane_g2o_application
    slam_library
    ${G2O_TYPES_SLAM3D}
    ${G2O_TYPES_SLAM3D_ADDONS}
    ${G2O_TYPES_DATA}
    ${G2O_CORE_LIBRARY}
    ${G2O_SOLVER_CSPARSE}
    ${G2O_STUFF_LIBRARY}
    ${G2O_SOLVER_CSPARSE_EXTENSION}
    ${CSPARSE_LIBRARY}
    types_data
  )
