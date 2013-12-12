#pragma once

#include "pwn_tracker.h"

namespace pwn_tracker {
  
  struct NewFrameWriteAction: public PwnTracker::NewFrameAction {
    NewFrameWriteAction(boss::Serializer* ser_, PwnTracker* tracker);
    void compute (PwnTrackerFrame* frame);
  protected:
    boss::Serializer* _ser;
  };


  struct NewRelationWriteAction: public PwnTracker::NewRelationAction {
    NewRelationWriteAction(boss::Serializer* ser_, PwnTracker* tracker);
    void compute (PwnTrackerRelation* relation);
  protected:
    boss::Serializer* _ser;
  };

  struct NewFrameEnqueueAction: public PwnTracker::NewFrameAction {
    NewFrameEnqueueAction(std::list<Serializable*>& objects_, PwnTracker* tracker);
    void compute (PwnTrackerFrame* frame);
  protected:
    std::list<Serializable*>& _objects;
  };


  struct NewRelationEnqueueAction: public PwnTracker::NewRelationAction {
    NewRelationEnqueueAction(std::list<Serializable*>& objects_, PwnTracker* tracker);
    void compute (PwnTrackerRelation* relation);
  protected:
    std::list<Serializable*>& _objects;
  };

	
};
