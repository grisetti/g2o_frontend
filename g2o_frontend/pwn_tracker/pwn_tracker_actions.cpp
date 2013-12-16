#include "pwn_tracker_actions.h"

namespace pwn_tracker {

  NewFrameWriteAction::NewFrameWriteAction(boss::Serializer* ser_, PwnTracker* tracker):
    PwnTracker::NewFrameAction(tracker){
    _ser = ser_;
  }

  void NewFrameWriteAction::compute (PwnTrackerFrame* frame) {
    cerr << "********************* NEW FRAME WRITE *********************" << endl;
    _ser->writeObject(*frame);
  }


  NewRelationWriteAction::NewRelationWriteAction(boss::Serializer* ser_, PwnTracker* tracker):
    PwnTracker::NewRelationAction(tracker) {
    _ser = ser_;
  }
  
  void NewRelationWriteAction::compute (PwnTrackerRelation* relation) {
    _ser->writeObject(*relation);
  }

  NewFrameEnqueueAction::NewFrameEnqueueAction(std::list<Serializable*>& objects_, PwnTracker* tracker):
    PwnTracker::NewFrameAction(tracker), _objects(objects_){}

  void NewFrameEnqueueAction::compute (PwnTrackerFrame* frame) {
    cerr << "********************* NEW FRAME *********************" << endl;
    _objects.push_back(frame);
  }


  NewRelationEnqueueAction::NewRelationEnqueueAction(std::list<Serializable*>& objects_, PwnTracker* tracker):
    PwnTracker::NewRelationAction(tracker), _objects(objects_){}
  
  void NewRelationEnqueueAction::compute (PwnTrackerRelation* relation) {
    _objects.push_back(relation);
  }

}
