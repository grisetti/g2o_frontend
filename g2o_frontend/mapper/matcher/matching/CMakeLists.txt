INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(matching_library
            matcher.cpp			matcher.h
            scan_matcher.cpp 		scan_matcher.h
            gradient_matcher.cpp        gradient_matcher.h
            correlative_matcher.cpp     correlative_matcher.h
            hierarchical_matcher.cpp 	hierarchical_matcher.h
)


SET_TARGET_PROPERTIES(matching_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_matching_library)
