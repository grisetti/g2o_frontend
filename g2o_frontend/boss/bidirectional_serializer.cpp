#include "bidirectional_serializer.h"

namespace boss {

  void BidirectionalSerializer::setOutputFilePath(const std::string& fpath){
    Serializer::setFilePath(fpath);
  }

  void BidirectionalSerializer::setOutputBinaryPath(const std::string& fpath) {
    Serializer::setBinaryPath(fpath);
  }


  void BidirectionalSerializer::setInputFilePath(const std::string& fpath){
    Deserializer::setFilePath(fpath);
  }
  
  
  std::ostream* BidirectionalSerializer::getBinaryOutputStream(const std::string& fname){
    return Serializer::getBinaryOutputStream(fname);
  }
  
  std::istream* BidirectionalSerializer::getBinaryInputStream(const std::string& fname){
    return Deserializer::getBinaryInputStream(fname);
  }

  
}
