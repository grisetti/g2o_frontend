ADD_LIBRARY(boss ${G2O_FRONTEND_LIB_TYPE}
  serialization_context.cpp
  serializer.cpp
  deserializer.cpp
  id_placeholder.cpp
  id_context.cpp
  message_data.cpp
  json_message_writer.cpp
  message_writer.cpp
  message_parser.cpp
  logger.cpp
  message.cpp
  object_data.cpp
  serializable.cpp
  identifiable.cpp
  blob.cpp 
  json_message_parser.cpp  
  object_parser.cpp
  object_writer.cpp
  json_object_parser.cpp
  json_object_writer.cpp
)

SET_TARGET_PROPERTIES(boss PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_boss)
TARGET_LINK_LIBRARIES(boss ${Boost_FILESYSTEM_LIBRARY})

ADD_EXECUTABLE(boss_sample boss_sample.cpp)
TARGET_LINK_LIBRARIES(boss_sample boss)
