#include <iostream>
#include "ros/ros.h"
#include "ros_message_context.h"
#include "ros_message_handler.h"
#include "ros_transform_message_handler.h"
#include "ros_pinholeimagedata_message_handler.h"
#include "ros_laser_message_handler.h"
#include "ros_imu_data_message_handler.h"


#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "g2o_frontend/boss_logger/blasersensor.h"
#include "g2o_frontend/boss_logger/bimusensor.h"
#include "g2o_frontend/boss_logger/bsynchronizer.h"
using namespace std;


int main(int argc, char** argv){
  bool useSynchronizer = false;
  if (argc <2){
    cerr << "usage: " << argv[0] <<  " <output filename>" << endl;
  }
  if (argc>3)
    useSynchronizer = true;
  boss::Serializer ser;
  ser.setFilePath(argv[1]);
  
  ros::init(argc, argv, "boss_logger");
  ros::NodeHandle nh;

  cerr << "Started the logger on file [" << argv[1] << "] synchronized mode: "<< useSynchronizer << endl; 
  // Add your handlers
  RosMessageContext context(&nh);
  context->setOdomFrameId("/odom");
  context->setBaseFrameId("/base_link");

  RosTransformMessageHandler tfHandler(&context);
  RosPinholeImageDataMessageHandler kDepthLogger(&context,"/kinect/depth_registered", "image_raw", "camera_info");
  RosPinholeImageDataMessageHandler kRGBLogger(&context,"/kinect/rgb", "image_color", "camera_info");
  RosLaserDataMessageHandler        frontLaserLogger(&context,"/front_scan");
  RosIMUDataMessageHandler          imuHandler(&context,"/imu/data");

  // subscribe the stuff
  tfHandler.subscribe();
  kDepthLogger.subscribe();
  kRGBLogger.subscribe();
  frontLaserLogger.subscribe();
  imuHandler.subscribe();

  // create a synchronizer
  boss::Synchronizer sync;
  // create a reframer object, that once all messages have been put together sets them to a unique frame
  boss::Synchronizer::Reframer reframer;
  sync.addOutputHandler(&reframer);

  // create a writer object that dumps on the disk each block of synchronized objects
  boss::Synchronizer::Writer writer(&ser);
  sync.addOutputHandler(&writer);

  // create a deleter object that polishes the memory after writing
  boss::Synchronizer::Deleter deleter;
  sync.addOutputHandler(&deleter);


  // // tell the topics we want to synchronize (done automatically by the addSynctimeCondition)
  // sync.addSyncTopic("/kinect/rgb/image_color");
  // sync.addSyncTopic("/kinect/depth_registered/image_raw");
  // sync.addSyncTopic("/front_scan");
  // sync.addSyncTopic("/imu/data");

  // add time conditions between the topics
  sync.addSyncTimeCondition("/kinect/rgb/image_color","/kinect/depth_registered/image_raw",0.05);
  sync.addSyncTimeCondition("/kinect/rgb/image_color", "/imu/data",0.1);
  sync.addSyncTimeCondition("/kinect/rgb/image_color", "/front_scan", 0.1);

  bool confReady=false;
  while (ros::ok()){
    ros::spinOnce();
    // cerr << "tf: " << tfFrameMap.size() 
    // 	 << ", sensors: "<<sensorTopicMap.size()
    // 	 << ", messages: "<<dataQueue.size() << endl;
    bool isReady = kDepthLogger.sensor() && kRGBLogger.sensor() && imuHandler.sensor();// && frontLaserLogger.sensor() ;
    // we got all transforms and sensors, so we can dump the configuration
    if (! confReady && isReady){
      cerr << endl << "CONF IS NOW READY!!!, STARTING WRITING" << endl;
      for (boss::StringFrameMap::iterator it = context.frameMap().begin(); it!=context.frameMap().end(); it++)
	ser.writeObject(*it->second);
      for (boss::StringSensorMap::iterator it = context.sensorMap().begin(); it!=context.sensorMap().end(); it++)
	ser.writeObject(*it->second);
      confReady = isReady;
    }
    if (confReady){
      while (context.messageQueue().size()){
	boss::BaseSensorData* data = dynamic_cast<boss::BaseSensorData*>(context.messageQueue().front());
	if (! data){
	  cerr << "fatal error, inconsistent data" << endl; 
	  return 0;
	}
	context.messageQueue().pop_front();
	if (useSynchronizer)
	  sync.addSensorData(data);
	else {
	  boss::Frame* f = data->robotFrame();
	  if (f){
	    ser.writeObject(*f);
	  }
	  ser.writeObject(*data);
	  if (f)
	    delete f;
	  delete data;
	  cerr << '.';
	}
      }
    }
    // start emptying the queue of the writable objects
  }
}
