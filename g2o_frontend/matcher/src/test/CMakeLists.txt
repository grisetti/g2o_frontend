INCLUDE_DIRECTORIES(${SRC_PATH})

ADD_EXECUTABLE(correlativeTest correlativeTest.cpp)
ADD_EXECUTABLE(gradientTest gradientTest.cpp)
ADD_EXECUTABLE(hierarchicalTest hierarchicalTest.cpp)
ADD_EXECUTABLE(structuresTest structuresTest.cpp)
ADD_EXECUTABLE(subSamplerTest subSamplerTest.cpp)


TARGET_LINK_LIBRARIES(correlativeTest ${LIBRARY_OUTPUT_PATH}/libreader.so ${LIBRARY_OUTPUT_PATH}/libmatcher.so)
TARGET_LINK_LIBRARIES(gradientTest ${LIBRARY_OUTPUT_PATH}/libreader.so ${LIBRARY_OUTPUT_PATH}/libmatcher.so)
TARGET_LINK_LIBRARIES(hierarchicalTest ${LIBRARY_OUTPUT_PATH}/libreader.so ${LIBRARY_OUTPUT_PATH}/libmatcher.so)
TARGET_LINK_LIBRARIES(structuresTest ${LIBRARY_OUTPUT_PATH}/libreader.so ${LIBRARY_OUTPUT_PATH}/libmatcher.so)
TARGET_LINK_LIBRARIES(subSamplerTest ${LIBRARY_OUTPUT_PATH}/libreader.so ${LIBRARY_OUTPUT_PATH}/libmatcher.so)