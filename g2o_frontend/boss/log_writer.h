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


#ifndef BOSS_LOG_WRITER_H
#define BOSS_LOG_WRITER_H

#include "id_context.h"
#include "message.h"

namespace boss {

class MessageWriter;
class ValueData;
class Serializable;
class ObjectData;

class LogWriter {
public:
  LogWriter(MessageWriter* writer): _writer(writer) {}
  void write(std::ostream& os, double timestamp, const std::string& source, Serializable& instance);
  void write(std::ostream& os, const std::string& source, Serializable& instance);
  void write(std::ostream& os, Message& message);
  virtual ~LogWriter();
  
protected:
  ValueData* processData(ValueData* vdata);

  IdContext _context;
  MessageWriter* _writer;
};

}

#endif // BOSS_LOG_WRITER_H
