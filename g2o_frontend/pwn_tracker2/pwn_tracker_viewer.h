#pragma once
#include "pwn_tracker.h"
#include "pwn_closer.h"
#include <QGLViewer/qglviewer.h>
#include "GL/gl.h"
#include "g2o_frontend/boss_map/map_utils.h"
#include "g2o_frontend/boss_map/stream_processor.h"

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

    std::map<MapNode*,VisCloud*> cloudMap;
    std::vector<std::set <MapNode*> > partitions;
    int currentPartitionIndex;
    std::list<MapNodeBinaryRelation*>  candidateRelations;
    bool final;
    MapManager* manager;
    MapRelationSelector *relationSelector;
    Eigen::Isometry3f sensorPose(MapNode* f);

    void drawRelation(MapNodeBinaryRelation* r);
    void drawFrame(MapNode* f);
    void draw();  
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

  class PwnSLAMVisualizerProcessor: public StreamProcessor{
  public:
    PwnSLAMVisualizerProcessor(int id = -1, boss::IdContext* context = 0);
    void init();
    virtual void process(Serializable* s);
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    virtual void deserializeComplete();

    MapManager* _manager;
    VisState* _visState;
    PwnCloudCache* _cache;
    MapRelationSelector* _selector;
    volatile bool _needRedraw;
  };



}
