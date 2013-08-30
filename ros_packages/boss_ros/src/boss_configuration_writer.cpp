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

using namespace std;

int main(int argc, char** argv){
  boss::Serializer ser;
  ser.setFilePath("test.log");
  
  ros::init(argc, argv, "bozz");
  ros::NodeHandle nh;

  // Add your handlers
  RosMessageContext context(&nh);
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

  bool confReady=false;
  boss::Frame* previousSavedFrame = 0;
  boss::SerializableQueue trash;
  while (ros::ok()){
    ros::spinOnce();
    // cerr << "tf: " << tfFrameMap.size() 
    // 	 << ", sensors: "<<sensorTopicMap.size()
    // 	 << ", messages: "<<dataQueue.size() << endl;
    bool isReady = kDepthLogger.sensor() && kRGBLogger.sensor() && imuHandler.sensor();// && frontLaserLogger.sensor() ;
    // we got all transforms and sensors, so we can dump the configuration
    if (! confReady && isReady){
      cerr << "CONF IS NOW READY!!!, STARTING WRITING";
      for (boss::StringFrameMap::iterator it = context.frameMap().begin(); it!=context.frameMap().end(); it++)
	ser.writeObject(*it->second);
      for (boss::StringSensorMap::iterator it = context.sensorMap().begin(); it!=context.sensorMap().end(); it++)
	ser.writeObject(*it->second);
      confReady = isReady;
    }
    if (confReady){
      while (context.messageQueue().size()){
	// save the oldest message and put it in the trash
	boss::Serializable* first = context.messageQueue().front();
	context.messageQueue().pop_front();
	ser.writeObject(*first);
	trash.push_back(first);
      }
      while (trash.size()>10){
	// save the oldest message and put it in the trash
	boss::Serializable* first = trash.front();
	trash.pop_front();
	delete first;
      }
      

    }

    // start emptying the queue of the writable objects
  }
}
