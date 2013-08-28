/*
    Abstract object parser interface
    Copyright (C) 2013  Daniele Baldassari <daniele@dikappa.org>

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


#ifndef BOSS_OBJECT_PARSER_H
#define BOSS_OBJECT_PARSER_H

#include<iostream>
#include<string>

namespace boss {

class ObjectData;

class ObjectParser {
public:
  /*!
   * \brief Read a single message data.
   * \details Read and parse a single object, returning a pointer to the object data, if any; a null pointer
   * is returned in case of parsing error or EOF. <br/>
   * The caller takes ownership of the returned pointer.
   * \param is input stream to read the message from
   * \return the read object's data
   */
  virtual ObjectData* readObject(std::istream& is, std::string& type)=0;

  virtual ~ObjectParser() {};
};

}

#endif // BOSS_OBJECT_PARSER_H
