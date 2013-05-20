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


#ifndef BOSS_ID_PLACEHOLDER_H
#define BOSS_ID_PLACEHOLDER_H

#include <vector>

#include "identifiable.h"

namespace boss {

class IdContext;

class IdPlaceholder: virtual public Identifiable {
public:
  void addVariable(Identifiable*& pvar);
  void resolve(Identifiable* instance);

  virtual const std::string& className();

  friend class IdContext;
protected:
  IdPlaceholder(int id, IdContext* context): Identifiable(id, context) {}
  
  std::vector<Identifiable**> _pvars;
};

}

#endif // BOSS_ID_PLACEHOLDER_H
