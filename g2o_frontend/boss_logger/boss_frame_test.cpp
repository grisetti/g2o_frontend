#include "set"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "bframe.h"
#include "bframerelation.h"
#include "bimagesensor.h"
#include "blasersensor.h"
#include "bimusensor.h"

using namespace boss;
using namespace std;

int main(int argc, char** argv) {

  Serializer ser;

  {
    ser.setFilePath("test.log");
  
    // create an origin that will act as a frame container
    Frame * originFrame = new Frame();
    // create a robot configuration
    Frame* baseFrame = new Frame("base_frame", Eigen::Isometry3d::Identity(), originFrame);

    Frame* imuFrame  = new Frame("imu_frame", Eigen::Isometry3d::Identity(), baseFrame);

    Eigen::Isometry3d baseToFrontLaserTransform=Eigen::Isometry3d::Identity();
    baseToFrontLaserTransform.translation() = Eigen::Vector3d(0.2, 0, 0.2);
    Frame* frontLaserFrame = new Frame("front_laser_frame", baseToFrontLaserTransform, baseFrame);

    Eigen::Isometry3d baseToRearLaserTransform=Eigen::Isometry3d::Identity();
    baseToRearLaserTransform.translation() = Eigen::Vector3d(-0.2, 0, 0.2);
    Frame* rearLaserFrame = new Frame("rear_laser_frame", baseToRearLaserTransform, baseFrame);


    Eigen::Isometry3d baseToRGBDCamTrandform=Eigen::Isometry3d::Identity();
    baseToRGBDCamTrandform.translation() = Eigen::Vector3d(0.0, 0, 0.1);
    Frame* rgbdFrame = new Frame("rgbd_frame", baseToRGBDCamTrandform, frontLaserFrame);

    ImageSensor* rgbImageSensor= new ImageSensor;
    rgbImageSensor->setFrame(rgbdFrame);
    rgbImageSensor->setTopic("kinect/rgb");

    ImageSensor* depthImageSensor= new ImageSensor;
    depthImageSensor->setFrame(rgbdFrame);
    depthImageSensor->setTopic("kinect/depth_registered");

    LaserSensor* frontLaserSensor = new LaserSensor;
    frontLaserSensor->setFrame(frontLaserFrame);
    frontLaserSensor->setTopic("front_laser");

    LaserSensor* rearLaserSensor = new LaserSensor;
    rearLaserSensor->setFrame(rearLaserFrame);
    rearLaserSensor->setTopic("rear_laser");

    IMUSensor* imuSensor = new IMUSensor;
    imuSensor->setTopic("imu");
    imuSensor->setFrame(imuFrame);
  

    // write the frames
    ser.write(argv[0], *originFrame);
    ser.write(argv[0], *baseFrame);
    ser.write(argv[0], *imuFrame);
    ser.write(argv[0], *frontLaserFrame);
    ser.write(argv[0], *rearLaserFrame);
    ser.write(argv[0], *rgbdFrame);
    ser.write(argv[0], *rgbImageSensor);
    // write the sensors
    ser.write(argv[0], *depthImageSensor);
    ser.write(argv[0], *frontLaserSensor);
    ser.write(argv[0], *rearLaserSensor);
    ser.write(argv[0], *imuSensor);

    // do a trivial lookup to see if all is in order
    cerr << "rgb full name: " << rgbdFrame->fullName() << endl;
    Frame* f = originFrame->childByName("base_frame/front_laser_frame/rgbd_frame");
    if (f) {
      cerr << "found frame " << f->fullName() << endl;
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
    Frame* origin=0;
    for (size_t i=0; i<messages.size(); i++) {
	origin = dynamic_cast<Frame*>( messages[i]->getInstance() );
	if (origin && origin->name()==""){
	    cerr << "found origin";
	    break;
	}
    }
    if (origin){
      Frame* f = origin->childByName("base_frame/front_laser_frame/rgbd_frame");
      if (f) {
	cerr << "found frame " << f->fullName() << endl;
      }
    }
      

  }



}


#if 0


  
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

#endif
