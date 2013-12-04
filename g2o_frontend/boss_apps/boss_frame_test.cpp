#include "set"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss_map/bframe.h"
#include "g2o_frontend/boss_map/bframerelation.h"
#include "g2o_frontend/boss_map/bimagesensor.h"
#include "g2o_frontend/boss_map/blasersensor.h"
#include "g2o_frontend/boss_map/bimusensor.h"

using namespace boss_map;
using namespace boss;
using namespace std;

int main(int argc, char** argv) {

  Serializer ser;

  {
    ser.setFilePath("test.log");
  
    // create an origin that will act as a frame container
    ReferenceFrame * originReferenceFrame = new ReferenceFrame();
    // create a robot configuration
    ReferenceFrame* baseReferenceFrame = new ReferenceFrame("base_frame", Eigen::Isometry3d::Identity(), originReferenceFrame);

    ReferenceFrame* imuReferenceFrame  = new ReferenceFrame("imu_frame", Eigen::Isometry3d::Identity(), baseReferenceFrame);

    Eigen::Isometry3d baseToFrontLaserTransform=Eigen::Isometry3d::Identity();
    baseToFrontLaserTransform.translation() = Eigen::Vector3d(0.2, 0, 0.2);
    ReferenceFrame* frontLaserReferenceFrame = new ReferenceFrame("front_laser_frame", baseToFrontLaserTransform, baseReferenceFrame);

    Eigen::Isometry3d baseToRearLaserTransform=Eigen::Isometry3d::Identity();
    baseToRearLaserTransform.translation() = Eigen::Vector3d(-0.2, 0, 0.2);
    ReferenceFrame* rearLaserReferenceFrame = new ReferenceFrame("rear_laser_frame", baseToRearLaserTransform, baseReferenceFrame);


    Eigen::Isometry3d baseToRGBDCamTrandform=Eigen::Isometry3d::Identity();
    baseToRGBDCamTrandform.translation() = Eigen::Vector3d(0.0, 0, 0.1);
    ReferenceFrame* rgbdReferenceFrame = new ReferenceFrame("rgbd_frame", baseToRGBDCamTrandform, frontLaserReferenceFrame);

    PinholeImageSensor* rgbImageSensor= new PinholeImageSensor;
    rgbImageSensor->setReferenceFrame(rgbdReferenceFrame);
    rgbImageSensor->setTopic("kinect/rgb");

    PinholeImageSensor* depthImageSensor= new PinholeImageSensor;
    depthImageSensor->setReferenceFrame(rgbdReferenceFrame);
    depthImageSensor->setTopic("kinect/depth_registered");

    LaserSensor* frontLaserSensor = new LaserSensor;
    frontLaserSensor->setReferenceFrame(frontLaserReferenceFrame);
    frontLaserSensor->setTopic("front_laser");

    LaserSensor* rearLaserSensor = new LaserSensor;
    rearLaserSensor->setReferenceFrame(rearLaserReferenceFrame);
    rearLaserSensor->setTopic("rear_laser");

    IMUSensor* imuSensor = new IMUSensor;
    imuSensor->setTopic("imu");
    imuSensor->setReferenceFrame(imuReferenceFrame);
  

    // write the frames
    ser.write(argv[0], *originReferenceFrame);
    ser.write(argv[0], *baseReferenceFrame);
    ser.write(argv[0], *imuReferenceFrame);
    ser.write(argv[0], *frontLaserReferenceFrame);
    ser.write(argv[0], *rearLaserReferenceFrame);
    ser.write(argv[0], *rgbdReferenceFrame);
    ser.write(argv[0], *rgbImageSensor);
    // write the sensors
    ser.write(argv[0], *depthImageSensor);
    ser.write(argv[0], *frontLaserSensor);
    ser.write(argv[0], *rearLaserSensor);
    ser.write(argv[0], *imuSensor);

    // do a trivial lookup to see if all is in order
    cerr << "rgb full name: " << rgbdReferenceFrame->path() << endl;
    ReferenceFrame* f = originReferenceFrame->childByName("base_frame/front_laser_frame/rgbd_frame");
    if (f) {
      cerr << "found frame " << f->path() << endl;
    }

  }

  {
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

    // find the origin;
    ReferenceFrame* origin=0;
    for (size_t i=0; i<messages.size(); i++) {
	origin = dynamic_cast<ReferenceFrame*>( messages[i]->getInstance() );
	if (origin && origin->name()==""){
	    cerr << "found origin";
	    break;
	}
    }
    if (origin){
      ReferenceFrame* f = origin->childByName("base_frame/front_laser_frame/rgbd_frame");
      if (f) {
	cerr << "found frame " << f->path() << endl;
      }
    }
  }



}


#if 0


  
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

#endif
