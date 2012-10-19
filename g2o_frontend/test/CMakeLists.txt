# ADD_EXECUTABLE(g2o_frontend_cli_application g2o.cpp)

# TARGET_LINK_LIBRARIES(g2o_frontend_cli_application ${G2O_STUFF_LIBRARY} ${G2O_CLI_LIBRARY} ${G2O_CORE_LIBRARY} ${PCL_LIBRARIES})
# IF(UNIX)
#   TARGET_LINK_LIBRARIES(g2o_frontend_cli_application dl)
# ENDIF(UNIX)

# include_directories(g2o_frontend_cli_application ${G2O_FRONTEND_G2O_INCLUDE})

# SET_TARGET_PROPERTIES(g2o_frontend_cli_application PROPERTIES OUTPUT_NAME g2o_frontend_test)

ADD_EXECUTABLE(register_sequence register_sequence)
TARGET_LINK_LIBRARIES(register_sequence ${G2O_STUFF_LIBRARY} ${G2O_CLI_LIBRARY} ${G2O_CORE_LIBRARY} ${PCL_LIBRARIES})
SET_TARGET_PROPERTIES(register_sequence PROPERTIES OUTPUT_NAME register_sequence)

add_executable(openni_grabber openni_grabber.cpp )
target_link_libraries (openni_grabber ${G2O_STUFF_LIBRARY} ${PCL_LIBRARIES})

# ADD_EXECUTABLE(pairwise_incremental_registration pairwise_incremental_registration.cpp)
# TARGET_LINK_LIBRARIES(pairwise_incremental_registration ${PCL_LIBRARIES})
# SET_TARGET_PROPERTIES(pairwise_incremental_registration PROPERTIES OUTPUT_NAME pairwise_incremental_registration)

add_executable(openni_image openni_image.cpp )
target_link_libraries (openni_image ${PCL_LIBRARIES})


add_executable(openni_cvcapture openni_cvcapture.cpp )
target_link_libraries (openni_cvcapture ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(matcher_simple matcher_simple.cpp )
target_link_libraries (matcher_simple ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(rgbdodometry rgbdodometry.cpp )
target_link_libraries (rgbdodometry ${OpenCV_LIBS} ${PCL_LIBRARIES})

# add_executable(test_feature_estimation test_feature_estimation.cpp )
# target_link_libraries (test_feature_estimation ${PCL_LIBRARIES})

# add_executable(pcl_openni_grabber2 pcl_openni_grabber2.cpp )
# target_link_libraries (pcl_openni_grabber2 ${PCL_LIBRARIES})



# INSTALL(g2o_cli_application
#   RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
#   LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#   ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
# )

# FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
# INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o_frontend/)
