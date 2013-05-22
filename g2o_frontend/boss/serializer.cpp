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

#include <ctime>
#include <sstream>

#include "serializer.h"
#include "json_message_writer.h"
#include "object_data.h"

using namespace std;
using namespace boss;

static const string DEFAULT_DATA_FILE="data.log";
static const string DEFAULT_BLOB_FILE="binary/<classname>.<id>";
static const string CURRENT_DIR=".";

static string toString(int i, size_t width, char padding) {
  stringstream ss;
  ss.width(width);
  ss.fill(padding);
  ss <<  i;
  return ss.str();
}

static void loadCurrentTime(map<string,string>& envMap) {
  time_t currentTime=time(0);
  struct tm* timeStruct=localtime(&currentTime);
  envMap["yyyy"]=toString(timeStruct->tm_year+1900,4,'0');
  envMap["yy"]=toString((timeStruct->tm_year+1900)%100,2,'0');
  envMap["mm"]=toString(timeStruct->tm_mon+1,2,'0');
  envMap["dd"]=toString(timeStruct->tm_mday,2,'0');
  envMap["hh"]=toString(timeStruct->tm_hour,2,'0');
  envMap["mi"]=toString(timeStruct->tm_min,2,'0');
  envMap["ss"]=toString(timeStruct->tm_sec,2,'0');
}

static void replaceEnvTags(string& str, map<string,string>& envMap) {
  string::iterator beginTag=str.end();
  for (string::iterator it=str.begin();it!=str.end();it++) {
    if (*it=='<') {
      beginTag=it;
      continue;
    }
    if (*it=='>'&&beginTag<it) {
      string replacement=envMap[str.substr(beginTag+1-str.begin(),it-beginTag-1)];
      str.replace(beginTag,it+1,replacement);
      it=beginTag+replacement.length()-1;
    }
  }
}
 
static ValueData* processDataForWrite(ValueData* vdata, SerializationContext& context) {
  switch (vdata->type()) {
    case OBJECT: {
      ObjectData* o=static_cast<ObjectData*>(vdata);
      const vector<string>& fields=o->fields();
      for (vector<string>::const_iterator f_it=fields.begin();f_it!=fields.end();f_it++) {
        ValueData* fdata=o->getField(*f_it);
        ValueData* replaceData=processDataForWrite(fdata, context);
        if (replaceData) {
          o->setField(*f_it,replaceData);
        }
      }
      break;
    }
    case ARRAY: {
      ArrayData* v_array=static_cast<ArrayData*>(vdata);
      for (vector<ValueData*>::const_iterator v_it=v_array->begin();v_it!=v_array->end();v_it++) {
        ValueData* replaceData=processDataForWrite(*v_it, context);
        if (replaceData) {
          v_array->set(v_it-v_array->begin(),replaceData);
        }
      }
      break;
    }
    case POINTER: {
      Identifiable* identifiable=vdata->getPointer();
      if (identifiable) {
	identifiable->ensureValidId(&context);
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

Serializer::Serializer(): _datastream(0) {
  setFilePath(DEFAULT_DATA_FILE);
  setBinaryPath(DEFAULT_BLOB_FILE);
  _writer=new JSONMessageWriter();
}

void Serializer::setFilePath(const string& fpath) {
  if (fpath.length()==0) {
    return;
  }
  _dataFileName=fpath;
  loadCurrentTime(_envMap);

  //Extract directory
  size_t pos=_dataFileName.rfind('/');
  if (pos==string::npos) {
    _envMap["datadir"]=CURRENT_DIR;
  } else {
    _envMap["datadir"]=_dataFileName.substr(0,pos);
  }
  
  if (_datastream) {
    delete _datastream;
    _datastream=0;
  }
}

void Serializer::setBinaryPath(const string& fpath) {
  if (fpath.length()==0) {
    return;
  }
  _blobFileName=fpath;
}

string Serializer::createBinaryFilePath(Identifiable& instance) {
  _envMap["classname"]=instance.className();
  _envMap["id"]=toString(instance.getId(),5,'0');
  string str=_blobFileName;
  replaceEnvTags(str,_envMap);
  return str;
}

bool Serializer::setFormat(const string& format) {
  //TODO Implementation
  return format=="JSON";
}

bool Serializer::write(const string& source, Serializable& instance) {
  return write(Message::getCurrentTime(), source, instance);
}

bool Serializer::write(Message& message) {
  return write(message.getTimestamp(), message.getSource(), *message.getInstance());
}

bool Serializer::write(double timestamp, const string& source, Serializable& instance) {
  ObjectData* data=new ObjectData();
  instance.serialize(*data,*this);
  
  processDataForWrite(data,*this);

  MessageData msgData(timestamp, instance.className(), source, data);
  if (!_datastream) {
    string str=_dataFileName;
    replaceEnvTags(str,_envMap);
    _datastream=new ofstream(str.c_str());
  }
  if (*_datastream) {
    _writer->writeMessage(*_datastream,msgData);
    //TODO Change writer to get status flag
    return true;
  }
  return false;
}

static void adjustBinaryPath(string& fname, map<string,string>& envMap) {
  //Check if it's an absolute path
  if (fname[0]!='/') {
    fname.insert(0,"/");
    fname.insert(0,envMap["datadir"]);
  }
}

ostream* Serializer::getBinaryOutputStream(const string& fname) {
  if (fname.empty()) {
    return 0;
  }
  string str=fname;
  adjustBinaryPath(str,_envMap);
  return new ofstream(str.c_str());
}

istream* Serializer::getBinaryInputStream(const string& fname) {
  if (fname.empty()) {
    return 0;
  }
  string str=fname;
  adjustBinaryPath(str,_envMap);
  return new ifstream(str.c_str());
}

Serializer::~Serializer() {
  delete _writer;
  if (_datastream) {
    delete _datastream;
  }
}

