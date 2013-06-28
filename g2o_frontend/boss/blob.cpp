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

#include <fstream>
#include <memory>

#include "blob.h"
#include "object_data.h"
#include "serialization_context.h"

using namespace boss;
using namespace std;

BLOB::BLOB() {
  _ref = 0;
}

BLOB::~BLOB() {
  if (_ref) {
    _ref->dataDestroyed();
  }
}

void BaseBLOBReference::dataDestroyed() {
  _instance=0;
}

void BaseBLOBReference::serialize(ObjectData& data, IdContext& context) {
  Identifiable::serialize(data,context);
  if (_instance) {
    //Check if binary file serialization is supported
    SerializationContext* fileContext=dynamic_cast<SerializationContext*>(&context);
    if (fileContext) {
      _fileName=fileContext->createBinaryFilePath(*this);
      auto_ptr<ostream> os(fileContext->getBinaryOutputStream(_fileName));
      if (os.get()) {
	_instance->write(*os);
      }
    }
  }
  data << field("pathName",_fileName);
}

void BaseBLOBReference::deserialize(ObjectData& data, IdContext& context) {
  Identifiable::deserialize(data, context);
  data >> field("pathName",_fileName);
  _instance=0;
}

bool BaseBLOBReference::load(BLOB& instance) {
  //Check if binary file serialization is supported
  SerializationContext* fileContext=dynamic_cast<SerializationContext*>(_context);
  if (fileContext) {
    auto_ptr<istream> is(fileContext->getBinaryInputStream(_fileName));
    if (is.get()) {
      return instance.read(*is);
    }
  }
  return false;
}
