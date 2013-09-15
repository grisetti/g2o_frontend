#include "ros_transform_message_handler.h"
using namespace std;

RosTransformMessageHandler::RosTransformMessageHandler(RosMessageContext* context): RosMessageHandler(context){}

void RosTransformMessageHandler::subscribe(){
  _tfSub = _context->nodeHandle()->subscribe("tf", 1, &RosTransformMessageHandler::tfMessageCallback, this);
}

void RosTransformMessageHandler::tfMessageCallback(const tf::tfMessage::ConstPtr& msg){
  for (size_t i=0; i<msg->transforms.size(); i++){
    const geometry_msgs::TransformStamped& t=msg->transforms[i];
    boss_logger::ReferenceFrame* parentReferenceFrame = 0;
    boss_logger::ReferenceFrame* childReferenceFrame = 0;

    boss_logger::StringReferenceFrameMap::iterator it = _context->frameMap().find(t.header.frame_id);    
    if (it == _context->frameMap().end()){
      parentReferenceFrame = new boss_logger::ReferenceFrame(t.header.frame_id,Eigen::Isometry3d::Identity(), 0);
      _context->frameMap().insert(std::make_pair(parentReferenceFrame->name(), parentReferenceFrame));
      cerr << "creating parent frame: " << parentReferenceFrame->name() << endl;
    } else {
      parentReferenceFrame = it->second;
    }
    it = _context->frameMap().find(t.child_frame_id);
    if (it == _context->frameMap().end()){
      childReferenceFrame = new boss_logger::ReferenceFrame(t.child_frame_id, Eigen::Isometry3d::Identity(), parentReferenceFrame);
      _context->frameMap().insert(std::make_pair(childReferenceFrame->name(), childReferenceFrame));
      cerr << "creating child frame: " << childReferenceFrame->name() << endl;
    } else {
      childReferenceFrame = it->second;
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
    childReferenceFrame->setTransform(transform);
    if (childReferenceFrame->parent()==0) {
      cerr << "reparenting child frame: " << childReferenceFrame->name() << " to " << parentReferenceFrame->name() << endl;
      childReferenceFrame->setParent(parentReferenceFrame);
    }
  }
}
