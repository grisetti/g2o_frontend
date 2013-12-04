ADD_LIBRARY(boss ${G2O_FRONTEND_LIB_TYPE}
  bidirectional_serializer.cpp bidirectional_serializer.h
  blob.cpp blob.h     
  deserializer.cpp deserializer.h
  identifiable.cpp identifiable.h
  id_context.cpp id_context.h   
  id_placeholder.cpp id_placeholder.h
  json_message_parser.cpp json_message_parser.h
  json_message_writer.cpp json_message_writer.h
  json_object_parser.cpp json_object_parser.h
  json_object_writer.cpp json_object_writer.h
  logger.cpp logger.h
  message.cpp message.h
  message_data.cpp message_data.h
  message_parser.cpp message_parser.h
  message_writer.cpp message_writer.h
  object_data.cpp object_data.h
  object_parser.cpp object_parser.h
  object_writer.cpp object_writer.h
  serializable.cpp serializable.h
  serializer.cpp serializer.h
  serialization_context.cpp serialization_context.h
)

SET_TARGET_PROPERTIES(boss PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_boss)
TARGET_LINK_LIBRARIES(boss ${Boost_FILESYSTEM_LIBRARY})
