#pragma once

#include "ros_message_handler.h"
#include "tf/tfMessage.h"
#include "tf/transform_broadcaster.h"


class RosTransformMessageHandler: public RosMessageHandler{
public:
  RosTransformMessageHandler(RosMessageContext* context);
  virtual void subscribe();
  virtual void publish(double timestamp);
  virtual void advertise(){}
protected:
  void tfMessageCallback(const tf::tfMessage::ConstPtr& msg);
  ros::Subscriber _tfSub;
  tf::TransformBroadcaster _tfPub;
};
