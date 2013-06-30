/*
 * boss_sample.cpp
 *
 *  Created on: Jun 30, 2013
 *      Author: daniele
 */

#include <vector>

#include "blob.h"
#include "serializer.h"

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

protected:
  char _buf[256];
};

BOSS_REGISTER_BLOB(MyBLOB)

int main(int /*argc*/, char** /*argv*/) {
  Serializer ser;

  ser.setFilePath("serializer_test.<yyyy>-<mm>-<dd>.log");
  ser.setBinaryPath("blob-<id>.dat");

  vector<BLOBReference<MyBLOB>*> refList;

  for (int i=0;i<10;i++) {
    MyBLOB* mb=new MyBLOB;
    BLOBReference<MyBLOB>* mbref=new BLOBReference<MyBLOB>(mb);
    ser.write("dummy",*mbref);
    refList.push_back(mbref);
    delete mb;
  }

  return 0;
}
