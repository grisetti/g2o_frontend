#include "set"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "bframe.h"
#include "bframerelation.h"

using namespace boss;
using namespace std;


int main(int argc, char** argv) {
  // create an origin that will act s a frame container
  Frame * originFrame = new Frame();
  // create a robot configuration
  Frame* baseFrame = new Frame("base_frame", originFrame);

  Frame* imuFrame  = new Frame("imu_frame", baseFrame);

  Eigen::Isometry3d baseToLaserTransform=Eigen::Isometry3f::Identity();
  baseToLaserTransform.translation() = Vector3f(0.2, 0, 0.2);
  Frame* laserFrame = new Frame("laser_frame", baseToLaserTransform, baseFrame);

  Eigen::Isometry3d baseToRGBDCamTrandform=Eigen::Isometry3f::Identity();
  baseToLaserTransform.translation() = Vector3f(0.0, 0, 0.1);
  Frame* rgbdcam_frame = new Frame("rgb_frame", baseToRGBDCam, laserFrame);




  
  Serializer ser;
  { // object writing
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
    if (previousFrame) {
      delete previousFrame;
    }
  }

  { //object reading
    std::vector<Message*> messages;
    Deserializer des;
    des.setFilePath("test.log");
    Message* m;
    while ((m=des.readMessage())){
      messages.push_back(m);
    }
    cout << "I read " << messages.size() << " messages" << endl;
    ser.setFilePath("test.log.check");
    for (size_t i=0;i<messages.size();i++) {
      ser.write(*messages[i]);
    }
    cout << "I wrote again " << messages.size() << " messages" << endl;
  }

}
