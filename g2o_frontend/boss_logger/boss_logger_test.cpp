#include "set"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "bframe.h"
#include "bframerelation.h"

using namespace boss;
using namespace std;


int main(int argc, char** argv) {
  // create an origin that will act s a frame container
  ReferenceFrame * originReferenceFrame = new ReferenceFrame();
  // create a robot configuration
  ReferenceFrame* baseReferenceFrame = new ReferenceFrame("base_frame", originReferenceFrame);

  ReferenceFrame* imuReferenceFrame  = new ReferenceFrame("imu_frame", baseReferenceFrame);

  Eigen::Isometry3d baseToLaserTransform=Eigen::Isometry3f::Identity();
  baseToLaserTransform.translation() = Vector3f(0.2, 0, 0.2);
  ReferenceFrame* laserReferenceFrame = new ReferenceFrame("laser_frame", baseToLaserTransform, baseReferenceFrame);

  Eigen::Isometry3d baseToRGBDCamTrandform=Eigen::Isometry3f::Identity();
  baseToLaserTransform.translation() = Vector3f(0.0, 0, 0.1);
  ReferenceFrame* rgbdcam_frame = new ReferenceFrame("rgb_frame", baseToRGBDCam, laserReferenceFrame);




  
  Serializer ser;
  { // object writing
    ser.setFilePath("test.log");
    int numReferenceFrames=100;
    ReferenceFrame* previousReferenceFrame=0;
    for (int i=0; i<numReferenceFrames; i++) {
      ReferenceFrame* f=new ReferenceFrame();
      f->setTransform(Eigen::Isometry3d::Identity());
      ser.write(argv[0],*f);

      if (previousReferenceFrame) {
        ReferenceFrameRelation* rel=new ReferenceFrameRelation();
        rel->setFromReferenceFrame(previousReferenceFrame);
        rel->setToReferenceFrame(f);
        ser.write(argv[0],*rel);
        delete rel;
        delete previousReferenceFrame;
      }

      previousReferenceFrame = f;
    }
    if (previousReferenceFrame) {
      delete previousReferenceFrame;
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
