ADD_LIBRARY(boss ${G2O_FRONTEND_LIB_TYPE}
  serialization_context.cpp
  serializer.cpp
  deserializer.cpp
  id_placeholder.cpp
  id_context.cpp
  message_data.cpp
#  log_writer.cpp
#  log_reader.cpp
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
)

SET_TARGET_PROPERTIES(boss PROPERTIES OUTPUT_NAME ${LIB_PREFIX}_boss)

