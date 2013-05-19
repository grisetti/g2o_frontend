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


#ifndef BOSS_LOGGER_H
#define BOSS_LOGGER_H

#include<string>
#include<map>

#include "serializable.h"

namespace boss {

class logger {
public:
  void register_factory(const std::string& type, Serializable* (*func)(ObjectData*));
  void store_object(const std::string& type, const std::string& source, Serializable& obj);
  void load_object(const std::string& type, const std::string& source, Serializable& obj);
  
protected:
  std::map<std::string, Serializable* (*)(ObjectData*)> _factories;
};

}

#endif // BOSS_LOGGER_H
