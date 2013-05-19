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

#include <stdexcept>
#include <memory>

#include "log_reader.h"

#include "message_data.h"
#include "object_data.h"
#include "identifiable.h"
#include "message_parser.h"
#include "message.h"

using namespace std;
using namespace boss;

Message* LogReader::readMessage(istream& is) {
  //First parse the raw message data struct
  auto_ptr<MessageData> msgData(_parser->readMessage(is));
  if (!msgData.get()) {
    //Parsing error
    //TODO Notify this occurrence
    return 0;
  }

  //Check if this is an Identifiable object
  ObjectData* odata=msgData->getData();
  ValueData* idValue=odata->getField("#id");
  
  //Identifiable objects are overwritten if another message
  //with the same ID is read, so first check if the ID is already in the context
  Serializable* instance=0;
  if (idValue) {
    instance=_context.getById(idValue->getInt());
  }
  
  if (instance) {
    //Just to ensure that the found instance has right type
    if (instance->className()!=msgData->getType()) {
      stringstream msg;
      msg << "Trying to overwrite " << instance->className() << " (ID " << idValue->getInt() << ") with " << msgData->getType();
      throw logic_error(msg.str()); 
    }
  } else {
    try {
      instance=Serializable::createInstance(msgData->getType());
    } catch (logic_error e) {
      //TODO Notify this occurrence
      return 0;
    }
  }
  
  vector<int> danglingPointers;
  vector<int> declaredIDs;
  
  processData(msgData->getData(),danglingPointers,declaredIDs);
  instance->deserialize(*odata,_context);
  for (vector<int>::iterator dp_it=danglingPointers.begin();dp_it!=danglingPointers.end();dp_it++) {
    _waitingInstances[instance].insert(*dp_it);
    _danglingReferences[*dp_it].insert(instance);
  }
  for (vector<int>::iterator id_it=declaredIDs.begin();id_it!=declaredIDs.end();id_it++) {
    //Further check, just in case the ID was a fake field
    if (_context.getById(*id_it)) {
      map<int,set<Serializable*> >::iterator entry=_danglingReferences.find(*id_it);
      if (entry!=_danglingReferences.end()) {
	set<Serializable*>& instSet=(*entry).second;
	for (set<Serializable*>::iterator instance_it=instSet.begin();instance_it!=instSet.end();instance_it++) {
	  _waitingInstances[*instance_it].erase(*id_it);
	  if (_waitingInstances[*instance_it].empty()) {
	    _waitingInstances.erase(*instance_it);
	    //TODO Notify serialization complete to the instance
	  }
	}
	_danglingReferences.erase(*id_it);
      }
    }
  }
  return new Message(msgData->getTimestamp(),msgData->getSource(),instance);
}

ValueData* LogReader::processData(ValueData* vdata, vector<int>& danglingRefs, vector<int>& declaredIDs) {
  switch (vdata->type()) {
    case OBJECT: {
      ObjectData* data=static_cast<ObjectData*>(vdata);
      ValueData* pfield=data->getField("#pointer");
      if (pfield) {
	int id=pfield->getInt();
	Identifiable* pointer=_context.getById(id);
	if (pointer) {
	  return new PointerData(pointer);
	}
	danglingRefs.push_back(id);
	return new PointerReference(_context.createPlaceHolder(id));
      }
      ValueData* idfield=data->getField("#id");
      if (idfield) {
	declaredIDs.push_back(idfield->getInt());
      }
      map<string,ValueData*>& v_map=vdata->getMap();
      for (map<string,ValueData*>::iterator v_it=v_map.begin();v_it!=v_map.end();v_it++) {
	ValueData* replaceData=processData((*v_it).second, danglingRefs, declaredIDs);
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
	ValueData* replaceData=processData(*v_it, danglingRefs, declaredIDs);
	if (replaceData) {
	  delete *v_it;
	  *v_it=replaceData;
	}
      }
      break;
    }
    default:
      //Nothing to do
      break;
  }
  return 0;
}

LogReader::~LogReader() {
  delete _parser;
}

