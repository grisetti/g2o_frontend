ADD_EXECUTABLE(correlative_matcher_test correlative_matcher_test.cpp)
SET_TARGET_PROPERTIES(correlative_matcher_test PROPERTIES OUTPUT_NAME correlative_matcher_test)
TARGET_LINK_LIBRARIES(correlative_matcher_test matching_library reader_library ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(gradient_matcher_test gradient_matcher_test.cpp)
SET_TARGET_PROPERTIES(gradient_matcher_test PROPERTIES OUTPUT_NAME gradient_matcher_test)
TARGET_LINK_LIBRARIES(gradient_matcher_test matching_library reader_library ${G2O_STUFF_LIBRARY})

#ADD_EXECUTABLE(hierarchical_matcher_test hierarchical_matcher_test.cpp)
#SET_TARGET_PROPERTIES(hierarchical_matcher_test PROPERTIES OUTPUT_NAME hierarchical_matcher_test)
#TARGET_LINK_LIBRARIES(hierarchical_matcher_test matching_library reader_library ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(structures_test structures_test.cpp)
SET_TARGET_PROPERTIES(structures_test PROPERTIES OUTPUT_NAME structures_test)
TARGET_LINK_LIBRARIES(structures_test matching_library reader_library ${G2O_STUFF_LIBRARY})

ADD_EXECUTABLE(sub_sampler_test sub_sampler_test.cpp)
SET_TARGET_PROPERTIES(sub_sampler_test PROPERTIES OUTPUT_NAME sub_sampler_test)
TARGET_LINK_LIBRARIES(sub_sampler_test matching_library reader_library ${G2O_STUFF_LIBRARY})


INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_EXECUTABLE(openLoopSLAM openLoopSLAM.cpp)
SET_TARGET_PROPERTIES(openLoopSLAM PROPERTIES OUTPUT_NAME openLoopSLAM)
TARGET_LINK_LIBRARIES(openLoopSLAM matching_library sensor_data_library ${G2O_TYPES_DATA} ${G2O_TYPES_SLAM2D}
                        ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION}
                        ${CSPARSE_LIBRARY} ${OPENGL_gl_LIBRARY} pthread
)
