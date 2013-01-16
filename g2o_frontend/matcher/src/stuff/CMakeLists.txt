INCLUDE_DIRECTORIES(${SRC_PATH})

ADD_LIBRARY(reader_library SHARED
	    logReader.cpp	logReader.h
	    laserData.cpp	laserData.h
)

SET_TARGET_PROPERTIES(reader_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}reader)
#TARGET_LINK_LIBRARIES(matcher_library ${G2O_CORE_LIBRARY})