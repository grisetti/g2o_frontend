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


#ifndef BOSS_BIDIRECTIONAL_SERIALIZER_H
#define BOSS_BIDIRECTIONAL_SERIALIZER_H

#include <string>

#include "serializer.h"
#include "deserializer.h"

namespace boss {

class MessageWriter;
class ObjectWriter;

  class BidirectionalSerializer: public Serializer, public Deserializer {
  public:
    BidirectionalSerializer();
  
    /*
     * Set file path for main data file.
     */
    void setOutputFilePath(const std::string& fpath);
    /*
     * Set file path for binary object files (BLOBs).
     * Unless an absolute path is specified the path is relative to the main data file directory.
     * to the directory name, if any.
     */
    void setOutputBinaryPath(const std::string& fpath);


  /*
   * Set file path for main data file.
   * Binary files use this path as base directory.
   */
  void setInputFilePath(const std::string& fpath);
  
  /*
   * Set data file format (default is "JSON").
   */
  bool setInputFormat(const std::string& format);


  virtual std::ostream* getBinaryOutputStream(const std::string& fname);
  virtual std::istream* getBinaryInputStream(const std::string& fname);

};

}

#endif // BOSS_SERIALIZER_H
