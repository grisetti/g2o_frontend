/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef BOSS_SERIALIZER_H
#define BOSS_SERIALIZER_H

#include <string>

#include "serialization_context.h"
#include "message.h"
#include "blob.h"

namespace boss {

class MessageWriter;

class Serializer: public SerializationContext {
public:
  Serializer();
  
  /*
   * Set file path for main data file.
   */
  void setFilePath(const std::string& fpath);
  
  /*
   * Set file path for binary object files (BLOBs).
   * Unless an absolute path is specified the path is relative to the main data file directory.
   * to the directory name, if any.
   */
  void setBinaryPath(const std::string& fpath);
  
  /*
   * Set data file format (default is "JSON").
   */
  bool setFormat(const std::string& format);

  /* Write a message.
   * Return false if an error occurred during serialization.
   */
  bool write(Message& message);
  bool write(double timestamp, const std::string& source, Serializable& instance);
  bool write(const std::string& source, Serializable& instance);
  
  virtual std::string createBinaryFilePath(BaseBLOBReference& instance);
  virtual std::ostream* getBinaryOutputStream(const std::string& fname);
  virtual std::istream* getBinaryInputStream(const std::string& fname);

  virtual ~Serializer();
  
protected:
  std::string _dataFileName;
  std::string _blobFileName;
  std::map<std::string, std::string> _envMap;
  
  MessageWriter* _writer;
  std::ostream* _datastream;
};

}

#endif // BOSS_SERIALIZER_H
