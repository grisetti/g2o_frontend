#include <iostream>

#include "ros_laser_publisher_handler.h"
#include "ros_pinhole_camera_publisher_handler.h"
#include "ros_imu_publisher_handler.h"

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
  ros::init(argc, argv, "boss_publisher");
  ros::NodeHandle nh;

  // Init objects
  RosLaserPublisherHandler *laserPublisher = 0;
  LaserSensor *laserSensor = 0;  
  LaserData *laserData = 0;
  RosPinholeCameraPublisherHandler *pinholeCameraPublisher1 = 0;
  RosPinholeCameraPublisherHandler *pinholeCameraPublisher2 = 0;
  PinholeImageSensor *pinholeImageSensor = 0;  
  PinholeImageData *pinholeImageData = 0;
  RosImuPublisherHandler *imuPublisher = 0;
  IMUSensor *imuSensor = 0;  
  IMUData *imuData = 0;

  boss::Deserializer des;
  des.setFilePath(argv[1]);
  boss::Serializable *s;
  while ((s = des.readObject()) && ros::ok()) {
    ros::spinOnce();

    // Check for a laser sensor
    laserSensor = dynamic_cast<LaserSensor*>(s);
    if (laserSensor) {
      cout << "Found a laser sensor" << endl;
      laserPublisher = new RosLaserPublisherHandler(&nh, laserSensor, laserSensor->topic());
      laserPublisher->publish();
      continue;
    }

    // Check for a pinhole camera sensor
    pinholeImageSensor = dynamic_cast<PinholeImageSensor*>(s);
    if (pinholeImageSensor) {
      cout << "Found a pinhole image sensor" << endl;
      if (!pinholeCameraPublisher1) {
	pinholeCameraPublisher1 = new RosPinholeCameraPublisherHandler(&nh, pinholeImageSensor, pinholeImageSensor->topic());
	pinholeCameraPublisher1->publish();
	continue;
      }
      else if (!pinholeCameraPublisher2) {
	pinholeCameraPublisher2 = new RosPinholeCameraPublisherHandler(&nh, pinholeImageSensor, pinholeImageSensor->topic());
	pinholeCameraPublisher2->publish();
	continue;
      }
      else {
	cerr << "WARNING: the last pinhole camera sensor was not configured, skipped" << endl;
      }
    }

    // Check for an IMU sensor
    imuSensor = dynamic_cast<IMUSensor*>(s);
    if (imuSensor) {
      cout << "Found an IMU sensor" << endl;
      imuPublisher = new RosImuPublisherHandler(&nh, imuSensor, imuSensor->topic());
      imuPublisher->publish();
      continue;
    }

    // Check for laser data
    laserData = dynamic_cast<LaserData*>(s);
    if (laserData && laserPublisher) {
      laserPublisher->callback(laserData);
      cout << ".";
      delete(s);
      continue;
    }

    // Check for pinhole image data
    pinholeImageData = dynamic_cast<PinholeImageData*>(s);
    if (pinholeImageData && pinholeImageData->sensor()->topic() == pinholeCameraPublisher1->topic()) {
      pinholeCameraPublisher1->callback(pinholeImageData);
      cout << ".";
      delete(s);
      continue;
    }
    if (pinholeImageData && pinholeImageData->sensor()->topic() == pinholeCameraPublisher2->topic()) {
      pinholeCameraPublisher2->callback(pinholeImageData);
      cout << ".";
      delete(s);
      continue;
    }

    // Check for laser data
    imuData = dynamic_cast<IMUData*>(s);
    if (imuData && laserPublisher) {
      imuPublisher->callback(imuData);
      cout << ".";
      delete(s);
      continue;
    }
  }
  cout << endl;
}
