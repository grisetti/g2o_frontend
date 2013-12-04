/*
 * boss_sample.cpp
 *
 *  Created on: Jun 30, 2013
 *      Author: daniele
 */

#include <vector>

#include "g2o_frontend/boss/blob.h"
#include "g2o_frontend/boss/serializer.h"

using namespace std;
using namespace boss;

class MyBLOB: public BLOB {
public:
  MyBLOB() {
    for (int i=0;i<256;i++) {
      _buf[i]=(char) i;
    }
  }
  virtual bool read(std::istream& is) {
    is.read(_buf,256);
    return is;
  }
  virtual void write(std::ostream& os) {
    os.write(_buf,256);
  }

  virtual const std::string& extension() {
    static string EXTENSION("bin");
    return EXTENSION;
  }
protected:
  char _buf[256];
};

BOSS_REGISTER_BLOB(MyBLOB)

int main(int /*argc*/, char** /*argv*/) {
  Serializer ser;

  //Set main log file path, tags <yyyy><mm><dd> will be replaced with current year, month and day
  //Other possible tags are:
  //- <yy> two-digit year
  //- <hh> hour
  //- <mi> minute
  //- <ss> second
  ser.setFilePath("data/serializer_test.<yyyy>-<mm>-<dd>.log");
  //Set BLOB file pattern, additional possible tags:
  //- <id> unique instance ID in this serialization context
  //- <ext> file extension as returned in BLOB::extension()
  //- <classname> the class name used in BLOB registration
  //Unless an absolute path is specified the BLOB path is always relative to the main file directory
  ser.setBinaryPath("binary/blob-<id>.<ext>");

  //References should be never discarded in order to guarantee proper ID generation
  vector<BLOBReference<MyBLOB>*> refList;

  for (int i=0;i<10;i++) {
    MyBLOB* mb=new MyBLOB;
    BLOBReference<MyBLOB>* mbref=new BLOBReference<MyBLOB>(mb);
    ser.write("dummy",*mbref);
    refList.push_back(mbref);
    //The BLOB can be discarded after serialization, data can be retrieved using BLOBReference::get() method
    delete mb;
  }

  return 0;
}
