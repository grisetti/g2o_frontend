ADD_LIBRARY(voronoi_library

            voronoi_diagram.cpp     voronoi_diagram.h
            voronoi_vertex.cpp      voronoi_vertex.h
            voronoi_edge.cpp        voronoi_edge.h
            voronoi_aligner.cpp     voronoi_aligner.h
)
SET_TARGET_PROPERTIES(voronoi_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_voronoi_library)
TARGET_LINK_LIBRARIES(voronoi_library opencv_core opencv_imgproc opencv_highgui)


#ADD_EXECUTABLE(fake_voronoi_test fake_voronoi_test.cpp)
#ADD_EXECUTABLE(real_voronoi_test real_voronoi_test.cpp)
#ADD_EXECUTABLE(voronoi_aligner_test voronoi_aligner_test.cpp)
ADD_EXECUTABLE(voronoi_extractor voronoi_extractor.cpp)
ADD_EXECUTABLE(my_distrans my_distrans.cpp)
ADD_EXECUTABLE(diagram_test diagram_test.cpp)


#TARGET_LINK_LIBRARIES(fake_voronoi_test voronoi_lib)
#TARGET_LINK_LIBRARIES(real_voronoi_test voronoi_lib)
#TARGET_LINK_LIBRARIES(voronoi_aligner_test voronoi_lib)
TARGET_LINK_LIBRARIES(voronoi_extractor voronoi_library opencv_highgui opencv_core)
TARGET_LINK_LIBRARIES(my_distrans opencv_highgui opencv_core opencv_imgproc)
TARGET_LINK_LIBRARIES(diagram_test voronoi_library)
