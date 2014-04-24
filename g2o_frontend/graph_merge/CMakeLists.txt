INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES} ${CSPARSE_INCLUDE_DIR})

SET(TMP_DIR ${CMAKE_CURRENT_BINARY_DIR})

SET(CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR})
#INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
SET(my_FORMS graph_gui.ui)
QT4_WRAP_UI(my_FORMS_HEADERS ${my_FORMS})
SET(CMAKE_CURRENT_BINARY_DIR ${TMP_DIR})

SET(my_HEADERS graph_gui.h viewer.h)
QT4_WRAP_CPP(my_HEADERS_MOC ${my_HEADERS})
INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})



ADD_LIBRARY(graph_merging   SHARED
                            drawable.cpp        drawable.h
                            graph.cpp           graph.h
                            graph_matcher.cpp   graph_matcher.h
                            utility.cpp         utility.h
)
SET_TARGET_PROPERTIES(graph_merging PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_graph_merging)
TARGET_LINK_LIBRARIES(graph_merging ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY}
                                    ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
                                    ${G2O_TYPES_SLAM2D} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION}
                                    ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}   ${CSPARSE_LIBRARY}
)



ADD_EXECUTABLE(graph_merge  graph_merge.cpp
                            graph_gui.cpp
                            viewer.cpp
                            ${my_FORMS_HEADERS} ${my_HEADERS_MOC}
)
TARGET_LINK_LIBRARIES(graph_merge   graph_merging
                                    ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY}
                                    ${QT_QTCORE_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${QGLVIEWER_LIBRARY_RELEASE} ${QT_LIBRARIES}
#                                    ${G2O_TYPES_SLAM2D} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION}
#                                    ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}   ${CSPARSE_LIBRARY}
)



ADD_EXECUTABLE(transform_graph  transform_graph.cpp)
TARGET_LINK_LIBRARIES(transform_graph   graph_merging)


ADD_EXECUTABLE(graph_simulator_test     graph_simulator_test.cpp
                                        graph_simulator.cpp
)
TARGET_LINK_LIBRARIES(graph_simulator_test  graph_merging
#                                            ${G2O_TYPES_SLAM2D} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION}
#                                            ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}   ${CSPARSE_LIBRARY}
)


ADD_EXECUTABLE(multi_graph_simulator    multi_graph_simulator.cpp
                                        graph_simulator.cpp
)
TARGET_LINK_LIBRARIES(multi_graph_simulator graph_merging
#                                            ${G2O_TYPES_SLAM2D} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION}
#                                            ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}   ${CSPARSE_LIBRARY}
)


ADD_EXECUTABLE(multi_simulation_merge	multi_simulation_merge.cpp
                                        graph_simulator.cpp
)
TARGET_LINK_LIBRARIES(multi_simulation_merge    graph_merging
#                                                ${G2O_TYPES_SLAM2D} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION}
#                                                ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}   ${CSPARSE_LIBRARY}
)


ADD_EXECUTABLE(match_merge	match_merge.cpp
                                        graph_simulator.cpp
)
TARGET_LINK_LIBRARIES(match_merge    graph_merging
#                                                ${G2O_TYPES_SLAM2D} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION}
#                                                ${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY}   ${CSPARSE_LIBRARY}
)

