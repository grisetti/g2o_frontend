
include_directories(${OPENNI2_INCLUDE_DIR} ${ZLIB_INCLUDE_DIR})

add_executable(OniQuery niframe.cpp nidevicestream.cpp nidevicewithstreams.cpp nisaverthread.cpp niquery.cpp)

TARGET_LINK_LIBRARIES(OniQuery ${OPENNI2_LIBRARY} ${ZLIB_LIBRARY})
