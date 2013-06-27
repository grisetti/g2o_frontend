include_directories(${OPENNI2_INCLUDE_DIR} ${ZLIB_INCLUDE_DIR})

add_library(oni2wrapper niframe.cpp nidevicestream.cpp nidevicewithstreams.cpp )
TARGET_LINK_LIBRARIES(oni2wrapper ${OPENNI2_LIBRARY} ${ZLIB_LIBRARY} )

add_executable(OniQuery nisaverthread.cpp niquery.cpp)
TARGET_LINK_LIBRARIES(OniQuery ${OPENNI2_LIBRARY} ${ZLIB_LIBRARY}  oni2wrapper)
