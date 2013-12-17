#pragma once
#include "pwn_tracker.h"
#include "pwn_closer.h"
#include <QGLViewer/qglviewer.h>
#include "GL/gl.h"

namespace pwn_tracker {

  using namespace pwn;
  using namespace boss_map;
  using namespace boss;
  using namespace std;

  struct VisCloud{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VisCloud(pwn::Cloud* cloud);
    void draw();
    GLuint drawList;
  };

  struct VisState{
    VisState(MapManager* manager);

    std::map<PwnTrackerFrame*,VisCloud*> cloudMap;
    std::vector<std::set <MapNode*> > partitions;
    int currentPartitionIndex;
    std::list<PwnCloserRelation*>  candidateRelations;
    bool final;
    MapManager* manager;
  
    Eigen::Isometry3f sensorPose(PwnTrackerFrame* f);

    void drawRelation(PwnTrackerRelation* r);
    void drawFrame(PwnTrackerFrame* f);
    void draw();  
  };






  // visualization actions
  struct NewFrameVisualizerCreator: public PwnTracker::NewFrameAction {
    NewFrameVisualizerCreator(VisState* visState_, PwnTracker* tracker);
    void compute (PwnTrackerFrame* frame) ;

    VisState* _visState;
  };



  struct CloserRelationVisualizer: public PwnTracker::NewRelationAction {
    CloserRelationVisualizer(PwnCloser* closer, 
			     VisState* visState_,  
			     PwnTracker* tracker);

    void compute (PwnTrackerRelation* /*relation*/);
    PwnCloser* _closer;
    VisState* _visState;
  };

class MyTrackerViewer: public QGLViewer{
public:
  MyTrackerViewer(VisState* s,QWidget *parent = 0, 
		  const QGLWidget *shareWidget = 0, 
		  Qt::WFlags flags = 0);
  virtual ~MyTrackerViewer();
  virtual void init();
  virtual void draw();
  VisState* visState;
};



}
