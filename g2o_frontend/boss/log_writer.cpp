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


//#include<memory>

#include "log_writer.h"
#include "object_data.h"
#include "serializable.h"
#include "identifiable.h"
#include "message_data.h"
#include "message_writer.h"
#include "message.h"


using namespace std;
using namespace boss;

void LogWriter::write(ostream& os, const string& source, Serializable& instance) {
  write(os, Message::getCurrentTime(), source, instance);
}

void LogWriter::write(ostream& os, Message& message) {
  write(os, message.getTimestamp(), message.getSource(), *message.getInstance());
}

void LogWriter::write(ostream& os, double timestamp, const string& source, Serializable& instance) {
  ObjectData* data=new ObjectData();
  instance.serialize(*data,_context);
  processData(data);
  MessageData msgData(timestamp, instance.className(), source, data);
  _writer->writeMessage(os,msgData);
}

ValueData* LogWriter::processData(ValueData* vdata) {
  switch (vdata->type()) {
    case OBJECT: {
      map<string,ValueData*>& v_map=vdata->getMap();
      for (map<string,ValueData*>::iterator v_it=v_map.begin();v_it!=v_map.end();v_it++) {
	ValueData* replaceData=processData((*v_it).second);
	if (replaceData) {
	  delete (*v_it).second;
	  (*v_it).second=replaceData;
	}
      }
      break;
    }
    case ARRAY: {
      vector<ValueData*>& v_array=vdata->getArray();
      for (vector<ValueData*>::iterator v_it=v_array.begin();v_it!=v_array.end();v_it++) {
	ValueData* replaceData=processData(*v_it);
	if (replaceData) {
	  delete *v_it;
	  *v_it=replaceData;
	}
      }
      break;
    }
    case POINTER: {
      Identifiable* identifiable=vdata->getPointer();
      if (identifiable) {
	identifiable->ensureValidId(&_context);
      }
      ObjectData* pointerObject=new ObjectData();
      pointerObject->setInt("#pointer",identifiable?identifiable->getId():-1);
      return pointerObject;
    }
    default:
      //Nothing to do
      break;
  }
  return 0;
}

LogWriter::~LogWriter() {
  delete _writer;
}


