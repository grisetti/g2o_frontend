#include <iostream>

#include "ros_message_context.h"
#include "ros_laser_message_handler.h"
#include "ros_pinholeimagedata_message_handler.h"
#include "ros_imu_data_message_handler.h"
#include "ros_transform_message_handler.h"

#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss/serializer.h"

using namespace std;
using namespace boss;
using namespace boss_logger;

int main(int argc, char **argv) {
  // Parse input arguments
  if (argc < 2) {
    cerr << "Missing input filename" << endl;
    cerr << "Usage: rosrun boss_ros boss_publisher input_file.log" << endl;
    return 0;
  }
  
  // Init objects
  ros::init(argc, argv, "boss_publisher");
  ros::NodeHandle nh;

  // Get context
  boss::Deserializer des;
  des.setFilePath(argv[1]);
  std::vector<BaseSensorData*> sensorDatas;
  RobotConfiguration* conf = readLog(sensorDatas, des);
  RosMessageContext* context = new RosMessageContext(&nh, conf);
  cerr << "# frames: " << context->frameMap().size() << endl;
  cerr << "# sensors: " << context->sensorMap().size() << endl;
  cerr << "# sensor datas: " << sensorDatas.size() << endl;

  // Init handlers
  RosTransformMessageHandler* rosTransformMessageHandler = new RosTransformMessageHandler(context);
  StringSensorMap& sensorMap = context->sensorMap();
  for(StringSensorMap::iterator iter = sensorMap.begin(); iter != sensorMap.end(); iter++) {
    // Laser sensor check
    LaserSensor* laserSensor = dynamic_cast<LaserSensor*>(iter->second);
    if(laserSensor) {
      cout << "retrieved laser sensor" << endl;
      context->addHandler("laser", iter->first);
      continue;
    }
    // Camera sensor check
    PinholeImageSensor* cameraSensor = dynamic_cast<PinholeImageSensor*>(iter->second);
    if(cameraSensor) {
      cout << "retrieved camera sensor" << endl;
      context->addHandler("image", iter->first);
      continue;
    }
    // IMU sensor check
    IMUSensor* imuSensor = dynamic_cast<IMUSensor*>(iter->second);
    if(imuSensor) {
      cout << "retrieved IMU sensor" << endl;
      context->addHandler("imu", iter->first);
      continue;
    }
    
    cerr << "WARNING: found an unknown sensor, skipping" << endl;
  }
  context->initPublishers();

  // Publish data
  des.setFilePath(argv[1]);
  boss::Serializable *s;
  while((s = des.readObject()) && ros::ok()) {
    ros::spinOnce();
    
    // Check for data
    boss_logger::BaseSensorData* data = dynamic_cast<boss_logger::BaseSensorData*>(s);
    if(data) {
      rosTransformMessageHandler->publish(data->timestamp());
      RosMessageHandler* messageHandler = context->handler(data->topic());
      messageHandler->publish(data);
      cerr << ".";
      delete(s);
      continue;
    }
    
    // Check for robot pose
    boss_logger::ReferenceFrame* referenceFrame = dynamic_cast<boss_logger::ReferenceFrame*>(s);
    if(referenceFrame) {
      if(referenceFrame->name() == "robotPose") {
	// Update odometry transform
	context->updateOdomReferenceFrame(referenceFrame);
	delete(s);
	continue;
      }
    }
  }
  cout << endl;

  return 0;
}
