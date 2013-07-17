#include "set";
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "bframe.h"
#include "bframerelation.h"

using namespace boss;
using namespace std;


int main(int argc, char** argv) {
  
  { // object writing
    Serializer ser;
    ser.setFilePath("test.log");
    int numFrames=100;
    Frame* previousFrame=0;
    for (int i=0; i<numFrames; i++) {
      Frame* f=new Frame();
      f->setTransform(Eigen::Isometry3d::Identity());
      ser.write(argv[0],*f);

      if (previousFrame) {
	FrameRelation* rel=new FrameRelation();
	rel->setFromFrame(previousFrame);
	rel->setToFrame(f);
	ser.write(argv[0],*rel);
	delete rel;
	delete previousFrame;
      }

      previousFrame = f;
    }
  }

  { //object reading
    std::set<Message*> messages;
    Deserializer des;
    des.setFilePath("test.log");
    Message* m;
    while (m=des.readMessage()){
      messages.insert(m);
    }
    cout << "I read " << messages.size() << " messages" << endl;

  }

}
