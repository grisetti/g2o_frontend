INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(matching_library            

            matcher.cpp			matcher.h
            scan_matcher.cpp 		scan_matcher.h
            gradient_matcher.cpp        gradient_matcher.h
            correlative_matcher.cpp     correlative_matcher.h
#            hierarchical_matcher.cpp 	hierarchical_matcher.h
)


SET_TARGET_PROPERTIES(matching_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_matching_library)
TARGET_LINK_LIBRARIES(matching_library
                      sensor_data_library ${G2O_TYPES_DATA} ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM3D}
                        ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
                        ${CSPARSE_LIBRARY} ${OPENGL_gl_LIBRARY} pthread)
