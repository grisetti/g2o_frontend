INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES})

ADD_LIBRARY(graph_merging   SHARED
            drawable.cpp        drawable.h
            graph.cpp           graph.h
            utility.cpp         utility.h
            viewer.cpp          viewer.h
)
SET_TARGET_PROPERTIES(graph_merging PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_graph_merging)
TARGET_LINK_LIBRARIES(graph_merging     ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY}
                                        ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)


ADD_EXECUTABLE(graph_merge graph_merge.cpp)
TARGET_LINK_LIBRARIES(graph_merge   graph_merging
                                    ${QGLVIEWER_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY}
                                    ${QT_QTCORE_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)
