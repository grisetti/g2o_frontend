 
ADD_LIBRARY(plane_library ${G2O_FRONTEND_LIB_TYPE}
   CloudUtils.cpp
   CloudUtils.h
   interface.cpp
   interface.h
   main.cpp
   moc_interface.cpp
   OverridedViewer.cpp
   OverridedViewer.h
   viewer_slider.h
 )

SET_TARGET_PROPERTIES(plane_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_slam)


INCLUDE_DIRECTORIES(${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES})


 TARGET_LINK_LIBRARIES(plane_library
   types_data
   ${G2O_TYPES_SLAM2D}
   ${G2O_TYPES_SLAM3D}
   ${G2O_TYPES_DATA}
   ${G2O_CORE_LIBRARY}
   ${G2O_STUFF_LIBRARY}
   ${QT_LIBRARIES}
 )

ADD_EXECUTABLE(PlaneEx_gui
  main.cpp
)

INCLUDE_DIRECTORIES(${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES})

TARGET_LINK_LIBRARIES(PlaneEx_gui
   types_data
   ${G2O_TYPES_SLAM2D}
   ${G2O_TYPES_SLAM3D}
   ${G2O_TYPES_DATA}
   ${G2O_CORE_LIBRARY}
   ${G2O_STUFF_LIBRARY}
   ${QT_LIBRARIES}
 )
