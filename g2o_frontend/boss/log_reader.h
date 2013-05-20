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


#ifndef BOSS_LOG_READER_H
#define BOSS_LOG_READER_H

#include <map>
#include <set>
#include <vector>
#include <istream>

#include "id_context.h"

namespace boss {
  
class Serializable;
class ValueData;
class Message;
class MessageParser;
class MessageData;

class LogReader {
public:
  LogReader(MessageParser* parser): _parser(parser) {}
  
  /*!
   * \brief Read a single message and create the related Serializable instance.
   * \details Read and parse a single message, returning a pointer to the message, if any; a null pointer
   * is returned in case of error. <br/>
   * The caller takes ownership of both the returned Message and its associated Serializable instance.
   * \param is input stream to read the message from
   * \return the read message
   */
  Message* readMessage(std::istream& is);
  virtual ~LogReader();
  
protected:
  ValueData* processData(ValueData* vdata, std::vector<int>& danglingRefs, std::vector<int>& declaredIDs);
  IdContext _context;
  MessageParser* _parser;
  
  //Map with Identifiable objects that refers to unresolved pointers
  std::map<Serializable*,std::set<int> > _waitingInstances;
  //Reverse map
  std::map<int,std::set<Serializable*> > _danglingReferences;
};

}

#endif // BOSS_LOG_READER_H
