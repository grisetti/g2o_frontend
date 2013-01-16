INCLUDE_DIRECTORIES(${SRC_PATH})

ADD_LIBRARY(matcher_library SHARED
	    matcher.cpp			matcher.h
	    scanMatcher.cpp		scanMatcher.h
	    correlativeMatcher.cpp	correlativeMatcher.h
	    gradientMatcher.cpp 	gradientMatcher.h
	    hierarchicalMatcher.cpp	hierarchicalMatcher.h
)

SET_TARGET_PROPERTIES(matcher_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}matcher)
TARGET_LINK_LIBRARIES(matcher_library ${G2O_CORE_LIBRARY})
