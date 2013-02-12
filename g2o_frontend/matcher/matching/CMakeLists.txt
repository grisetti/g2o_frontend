INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

ADD_LIBRARY(
	matching_library
	
	matcher.cpp			matcher.h
	scanMatcher.cpp 		scanMatcher.h
	gradientMatcher.cpp 		gradientMatcher.h
	correlativeMatcher.cpp 		correlativeMatcher.h
	hierarchicalMatcher.cpp 	hierarchicalMatcher.h
	charMatcher.cpp			charMatcher.h
	charCorrMatcher.cpp		charCorrMatcher.h
	charGradMatcher.cpp		charGradMatcher.h
)


SET_TARGET_PROPERTIES(matching_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_matching_library)
