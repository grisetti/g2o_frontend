#include "ros_laser_message_handler.h"

using namespace std;

RosLaserDataMessageHandler::RosLaserDataMessageHandler(RosMessageContext* context_, std::string topicName_) : RosMessageHandler(context_) {
  _pubQueueSize = 100;
  _topicName = topicName_;
  _sensor = 0;
}

RosLaserDataMessageHandler::~RosLaserDataMessageHandler() {
}

bool RosLaserDataMessageHandler::configReady() const {
  return _sensor;
}

void RosLaserDataMessageHandler::subscribe() {
  _sub = _context->nodeHandle()->subscribe(_topicName, 1, &RosLaserDataMessageHandler::callback, this);
  cerr << "subscribing to topic: [" <<  _topicName << "]" << endl;
}

void RosLaserDataMessageHandler::setSensor(boss_logger::BaseSensor* sensor_) {
  boss_logger::LaserSensor* laserSensor = dynamic_cast<boss_logger::LaserSensor*>(sensor_);
  if(!laserSensor) {
    cerr << "WARNING: tried to set a non laser sensor to a laser message handler, skipping" << endl;
    return;
  }
  _sensor = laserSensor;
}

void RosLaserDataMessageHandler::publish(boss_logger::BaseSensorData* sdata) {
  if(!_sensor) {
    cerr << "WARNING: missing laser sensor, skipping" << endl;
    return;
  }

  boss_logger::LaserData *laserData = dynamic_cast<boss_logger::LaserData*>(sdata);
  if(!laserData) {
    cerr << "WARNING: trying to publish non laser data from a laser message handler, skipping" << endl;
    return;
  }

  // Create message
  sensor_msgs::LaserScan laserScan;
  std_msgs::Header header;
  header.seq = _sequenceID;
  header.stamp = ros::Time(laserData->timestamp());
  header.frame_id = _sensor->frame()->name();
  laserScan.header = header;
  laserScan.angle_min = -1.57079637051f;
  laserScan.angle_max = laserData->fov() + laserScan.angle_min;
  laserScan.angle_increment = 0.00436332309619f; 
  laserScan.time_increment = 1.73611115315e-05;
  laserScan.scan_time = 0.0250000003725;
  laserScan.range_min = _sensor->minRange();
  laserScan.range_max = _sensor->maxRange();
  laserScan.ranges = laserData->ranges();
  laserScan.intensities = laserData->remissions();

  // Publish message
  _pub.publish(laserScan);
  _sequenceID++;
}

void RosLaserDataMessageHandler::advertise() {
  cout << "publishing on topic: [" << _topicName << "]" << endl;
  _pub = _context->nodeHandle()->advertise<sensor_msgs::LaserScan>(_topicName, _pubQueueSize);
}

void RosLaserDataMessageHandler::callback(const sensor_msgs::LaserScanConstPtr& scan) {
  if  (! _sensor) {
    std::map<std::string, boss_logger::ReferenceFrame*>::iterator it = _context->frameMap().find(scan->header.frame_id);
    if (it == _context->frameMap().end()) {
      cerr << "missing transform for frame [" << scan->header.frame_id << "], skipping" << endl;
      return;
    }
    _sensor = new boss_logger::LaserSensor;
    _sensor->setTopic(_topicName);
    _sensor->setReferenceFrame(it->second);
    float fov = scan->angle_max-scan->angle_min;
    int numBeams = scan->ranges.size();
    _sensor->setMinRange(scan->range_min);
    _sensor->setMaxRange(scan->range_max);
    _sensor->setFov(fov);
    _sensor->setNumBeams(numBeams);
    cerr << "created sensor for topic[" << _sensor->topic()  << "]" << endl;
    _context->sensorMap().insert(make_pair(_sensor->topic(), _sensor));
  }
  Eigen::Isometry3d robotTransform;
  if (! _context->getOdomPose(robotTransform, scan->header.stamp.toSec()) ){
    return;
  } 
  boss_logger::ReferenceFrame* newReferenceFrame = new boss_logger::ReferenceFrame("robotPose", robotTransform);
  //_context->messageQueue().push_back(newReferenceFrame);
  // we get the image from ROS

  boss_logger::LaserData* laserData = new boss_logger::LaserData(_sensor);
  laserData->setTimestamp(scan->header.stamp.toSec());
  laserData->setTopic(_sensor->topic());
  laserData->ranges().resize(scan->ranges.size());
  laserData->setRobotReferenceFrame(newReferenceFrame);
  for (size_t i=0; i<laserData->ranges().size(); i++){
    laserData->ranges()[i] = scan->ranges[i];
  }
  
  laserData->remissions().resize(scan->intensities.size());
  _context->messageQueue().push_back(laserData);
  //delete imageData;
  //cerr << "l";
}
