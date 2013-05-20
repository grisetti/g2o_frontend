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

#include "identifiable.h"
#include "id_context.h"
#include "object_data.h"

using namespace boss;
using namespace std;

Identifiable::Identifiable(int id, IdContext* context): _id(id), _context(context) {
  if (_context) {
    if (_id<0) {
      throw logic_error("invalid id value within context: "+_id);
    }
    if (!_context->add(this)) {
      throw logic_error("duplicate id value within context: "+_id);
    }   
  }
}

Identifiable::~Identifiable() {
  if (_context) {
    _context->remove(this);
  }
}

bool Identifiable::setContext(IdContext* context) {
  if (_context==context) {
    return true;
  }
  if (context) {
    if (_id<0) {
      throw logic_error("invalid id value within context: "+_id);
    }
    if (!context->add(this)) {
      return false;
    }
  }
  if (_context) {
    _context->remove(this);
  }
  _context=context;
  return true;
}

bool Identifiable::setId(int id, IdContext* context) {
  if (id==_id) {
    return setContext(context);
  }
  
  int oldId=_id;
  IdContext* oldContext=_context;
  _id=id;
  _context=context;
  
  bool ok=true;
  if (_context) {
    if (_id<0) {
      throw logic_error("invalid id value within context: "+_id);
    }
    if (_context==oldContext) {
      ok=_context->update(this,oldId);
    } else {
      ok=_context->add(this);
    }
  }
  
  if (!ok) {
    _id=oldId;
    _context=oldContext;
    return false;
  }
  
  if (oldContext&&oldContext!=_context) {
    oldContext->remove(this);
  }
  
  return true;
}

int Identifiable::getId() {
  return _id;
}

void Identifiable::ensureValidId(IdContext* context) {
  //Ensure that the current ID is valid in the serialization context and register
  //this instance
  if (_id<0||!setContext(context)) {
    setId(context->generateId(),context);
  }
}

void Identifiable::serialize(ObjectData& data, IdContext& context) {
  ensureValidId(&context);
  data << field("#id",_id);
}

void Identifiable::deserialize(ObjectData& data, IdContext& context) {
  data >> field("#id",_id);
  setContext(&context);
}

