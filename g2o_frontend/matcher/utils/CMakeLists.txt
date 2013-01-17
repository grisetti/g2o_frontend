ADD_LIBRARY(
	reader_library

    laserData.cpp	laserData.h
    logReader.cpp	logReader.h
)

SET_TARGET_PROPERTIES(reader_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_reader_library)
