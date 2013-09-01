#include "ros_laser_message_handler.h"

using namespace std;

RosLaserDataMessageHandler::RosLaserDataMessageHandler(RosMessageContext* context, std::string topicName_): RosMessageHandler(context){
    _topicName = topicName_;
    _sensor = 0;
  }

RosLaserDataMessageHandler::~RosLaserDataMessageHandler() {
}

bool RosLaserDataMessageHandler::configReady() const{
  return _sensor;
}

void RosLaserDataMessageHandler::subscribe(){
  _sub= _context->nodeHandle()->subscribe(_topicName, 1, &RosLaserDataMessageHandler::callback, this);
  cerr << "subscribed topics: [" <<  _topicName << "]" << endl;
}


void RosLaserDataMessageHandler::callback(const sensor_msgs::LaserScanConstPtr& scan){
  if  (! _sensor) {
    std::map<std::string, boss::Frame*>::iterator it = _context->frameMap().find(scan->header.frame_id);
    if (it == _context->frameMap().end()) {
      cerr << "missing transform for frame [" << scan->header.frame_id << "], skipping" << endl;
      return;
    }
    _sensor = new boss::LaserSensor;
    _sensor->setTopic(_topicName);
    _sensor->setFrame(it->second);
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
  boss::Frame* newFrame = new boss::Frame("robotPose", robotTransform);
  //_context->messageQueue().push_back(newFrame);
  // we get the image from ROS

  boss::LaserData* laserData = new boss::LaserData(_sensor);
  laserData->setTimestamp(scan->header.stamp.toSec());
  laserData->setTopic(_sensor->topic());
  laserData->ranges().resize(scan->ranges.size());
  laserData->setRobotFrame(newFrame);
  for (size_t i=0; i<laserData->ranges().size(); i++){
    laserData->ranges()[i] = scan->ranges[i];
  }
  
  laserData->remissions().resize(scan->intensities.size());
  _context->messageQueue().push_back(laserData);
  //delete imageData;
  //cerr << "l";
}
