#include "ros_sensor_publisher_handler.h"

RosSensorPublisherHandler::RosSensorPublisherHandler() {
  _nh = 0;
  _sensor = 0;
  _topic = "";
  _sequenceID = 0;
  _publishingQueueSize = 0;
}

RosSensorPublisherHandler::~RosSensorPublisherHandler() {}
