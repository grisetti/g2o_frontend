#ifndef _ROS_TRANSFORM_MESSAGE_HANDLER_H_
#define _ROS_TRANSFORM_MESSAGE_HANDLER_H_
#include "ros_message_handler.h"
#include "tf/tfMessage.h"

class RosTransformMessageHandler: public RosMessageHandler{
public:
  RosTransformMessageHandler(RosMessageContext* context);
  virtual void subscribe();
protected:
  void tfMessageCallback(const tf::tfMessage::ConstPtr& msg);
  ros::Subscriber _tfSub;
};

#endif
