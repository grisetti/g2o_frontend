ADD_LIBRARY(voronoi_lib SHARED

            voronoi_diagram.cpp     voronoi_diagram.h
            voronoi_vertex.cpp      voronoi_vertex.h
            voronoi_edge.cpp        voronoi_edge.h
            voronoi_aligner.cpp     voronoi_aligner.h
)
TARGET_LINK_LIBRARIES(voronoi_lib opencv_core opencv_imgproc opencv_highgui)


ADD_EXECUTABLE(voronoi_extractor voronoi_extractor.cpp)
ADD_EXECUTABLE(diagram_test diagram_test.cpp)
ADD_EXECUTABLE(distance_map_extractor distance_map_extractor.cpp)
ADD_EXECUTABLE(voronoi_graph prova.cpp)


TARGET_LINK_LIBRARIES(voronoi_extractor voronoi_lib opencv_highgui opencv_core)
TARGET_LINK_LIBRARIES(voronoi_graph voronoi_lib opencv_highgui opencv_core)
TARGET_LINK_LIBRARIES(diagram_test voronoi_lib)
TARGET_LINK_LIBRARIES(distance_map_extractor voronoi_lib)
