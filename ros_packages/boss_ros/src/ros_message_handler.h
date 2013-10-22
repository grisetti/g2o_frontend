#pragma once

#include "ros_message_context.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>


class RosMessageHandler {
 public:
  RosMessageHandler(RosMessageContext* context_) { 
    _sequenceID = 0;
    _context = context_; 
  }
  
  virtual ~RosMessageHandler();
  virtual void subscribe() = 0;
  virtual void publish(boss_logger::BaseSensorData* sdata) {}
  virtual void publish(double timestamp) {}
  virtual void advertise() {}
  virtual bool configReady() const;
  
 protected:
  unsigned int _sequenceID;
  RosMessageContext* _context;
};

