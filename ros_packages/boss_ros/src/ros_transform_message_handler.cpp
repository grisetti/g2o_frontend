#include "ros_transform_message_handler.h"
using namespace std;

RosTransformMessageHandler::RosTransformMessageHandler(RosMessageContext* context): RosMessageHandler(context){}

void RosTransformMessageHandler::subscribe(){
  _tfSub = _context->nodeHandle()->subscribe("tf", 1, &RosTransformMessageHandler::tfMessageCallback, this);
}

void RosTransformMessageHandler::tfMessageCallback(const tf::tfMessage::ConstPtr& msg){
  for (size_t i=0; i<msg->transforms.size(); i++){
    const geometry_msgs::TransformStamped& t=msg->transforms[i];
    boss::Frame* parentFrame = 0;
    boss::Frame* childFrame = 0;

    std::map<std::string, boss::Frame*>::iterator it = _context->frameMap().find(t.header.frame_id);    
    if (it == _context->frameMap().end()){
      parentFrame = new boss::Frame(t.header.frame_id,Eigen::Isometry3d::Identity(), 0);
      _context->frameMap().insert(std::make_pair(parentFrame->name(), parentFrame));
      cerr << "creating parent frame: " << parentFrame->name() << endl;
    } else {
      parentFrame = it->second;
    }
    it = _context->frameMap().find(t.child_frame_id);
    if (it == _context->frameMap().end()){
      childFrame = new boss::Frame(t.child_frame_id, Eigen::Isometry3d::Identity(), parentFrame);
      _context->frameMap().insert(std::make_pair(childFrame->name(), childFrame));
      cerr << "creating child frame: " << childFrame->name() << endl;
    } else {
      childFrame = it->second;
    }

    Eigen::Isometry3d transform;
    transform.translation().x()=t.transform.translation.x;
    transform.translation().y()=t.transform.translation.y;
    transform.translation().z()=t.transform.translation.z;
    Eigen::Quaterniond rot;
    rot.x()=t.transform.rotation.x;
    rot.y()=t.transform.rotation.y;
    rot.z()=t.transform.rotation.z;
    rot.w()=t.transform.rotation.w;
    transform.linear()=rot.toRotationMatrix();
    childFrame->setTransform(transform);
    if (childFrame->parent()==0) {
      cerr << "reparenting child frame: " << childFrame->name() << " to " << parentFrame->name() << endl;
      childFrame->setParent(parentFrame);
    }
  }
}
