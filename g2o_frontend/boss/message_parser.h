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


#ifndef BOSS_MESSAGE_PARSER_H
#define BOSS_MESSAGE_PARSER_H

#include<iostream>

#include "identifiable.h"

namespace boss {

class MessageData;

class MessageParser {
public:
  /*!
   * \brief Read a single message data.
   * \details Read and parse a single message, returning a pointer to the message data, if any; a null pointer
   * is returned in case of parsing error or EOF. <br/>
   * The caller takes ownership of the returned pointer.
   * \param is input stream to read the message from
   * \return the read message data
   */
  virtual MessageData* readMessage(std::istream& is)=0;
  
  virtual ~MessageParser() {};
};

}

#endif // BOSS_MESSAGE_PARSER_H
