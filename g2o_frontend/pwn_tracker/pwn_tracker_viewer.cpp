#include "pwn_tracker_viewer.h"
#include "g2o/stuff/opengl_primitives.h"

namespace pwn_tracker  {

  using namespace pwn;
  using namespace boss_map;
  using namespace boss;
  using namespace std;

  class StandardCamera : public qglviewer::Camera {
  public:
    StandardCamera() : _standard(true) {}
  
    float zNear() const {
      if(_standard) 
	return 0.001f; 
      else 
	return Camera::zNear(); 
    }

    float zFar() const {  
      if(_standard) 
	return 10000.0f; 
      else 
	return Camera::zFar();
    }

    bool standard() const { return _standard; }
  
    void setStandard(bool s) { _standard = s; }

  protected:
    bool _standard;
  };

  VisCloud::VisCloud(pwn::Frame* cloud) {
    drawList = 0;
    if (cloud && (drawList = glGenLists(1)) ) {
      glNewList(drawList,GL_COMPILE);
      //g2o::opengl::drawPyramid(0.2, 0.2);
      int k=0;
      glBegin(GL_POINTS);
      for (size_t i=0; i<cloud->points().size(); i++){
	if (cloud->normals()[i].squaredNorm()>0.1f) {
	  glNormal3f(cloud->normals()[i].x(),cloud->normals()[i].y(),cloud->normals()[i].z());
	  glVertex3f(cloud->points()[i].x(),cloud->points()[i].y(),cloud->points()[i].z());
	  k++;
	}
      }
      glEnd();
      glEndList();
      cerr << "CLOUD: " << cloud << " POINTS: " << k << " DRAWLIST: " << drawList << endl;
    } else {
      cerr << "CLOUD: " << cloud << " DRAWLIST: " << drawList << endl;
    }
  }

  void VisCloud::draw() {
    if (drawList)
      glCallList(drawList);
  }

  VisState::VisState(MapManager* manager){
    this->manager = manager;
    final=false;
  }
  
  Eigen::Isometry3f VisState::sensorPose(PwnTrackerFrame* f){
    Eigen::Isometry3f t;
    convertScalar(t,f->transform());
    return t;/**f->sensorOffset;*/
  }

  void VisState::drawRelation(PwnTrackerRelation* r){
    PwnTrackerFrame* from = r->from();
    PwnTrackerFrame* to = r->to();
    Eigen::Isometry3f fromS = sensorPose(from);
    Eigen::Isometry3f toS = sensorPose(to);
    glBegin(GL_LINES);
    glNormal3f(0,0,1);
    glVertex3f(fromS.translation().x(), fromS.translation().y(), fromS.translation().z());
    glVertex3f(toS.translation().x(), toS.translation().y(), toS.translation().z());
    glEnd();
  }

  void VisState::drawFrame(PwnTrackerFrame* f){
    std::map<PwnTrackerFrame*,VisCloud*>::iterator it=cloudMap.find(f);
    if(it!=cloudMap.end()){
      Eigen::Isometry3f sPose=sensorPose(f);
      sPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      glPushMatrix();
      glMultMatrixf(sPose.data());
      //cerr << "DRAWING FRAME: " << f->seq << endl;
      it->second->draw();
      glPopMatrix();
    }
  }

  void VisState::draw(){
    // draw the trajectory
    for(std::set<MapNodeRelation*>::iterator it=manager->relations().begin(); it!=manager->relations().end(); it++){
      PwnCloserRelation* cRel=dynamic_cast<PwnCloserRelation*>(*it);
      PwnTrackerRelation* tRel=dynamic_cast<PwnTrackerRelation*>(*it);
      if (tRel || (cRel && cRel->accepted) ) {
	glColor3f(0,0,1);
	drawRelation(tRel); 
      } else if (cRel){
	glColor3f(0,1,0);
	drawRelation(cRel); 
      }
    }
    if (! final){
      for (int i=0; i<(int)partitions.size(); i++){
	std::set<MapNode*>& p=partitions[i];
	if (i==currentPartitionIndex){
	  glColor3f(1,0,0);
	} else {
	  glColor3f(0,0,1);
	}
	for (std::set<MapNode*>::iterator it=p.begin(); it!=p.end();it++){
	  PwnTrackerFrame* f = dynamic_cast<PwnTrackerFrame*>(*it);
	  if (f) {
	    drawFrame(f);
	  }
	}
      }
    } else {
      glColor3f(1,0,0);
      for (std::set<MapNode*>::iterator it=manager->nodes().begin(); it!=manager->nodes().end();it++){
	PwnTrackerFrame* f = dynamic_cast<PwnTrackerFrame*>(*it);
	if (f) {
	  drawFrame(f);
	}
      }
    }
  }





// visualization actions
  NewFrameVisualizerCreator::NewFrameVisualizerCreator(VisState* visState_, PwnTracker* tracker):
    PwnTracker::NewFrameAction(tracker){
    _visState=visState_;
  }

  void NewFrameVisualizerCreator::compute (PwnTrackerFrame* frame) {
    PwnCache::HandleType h=_tracker->cache()->get(frame);
    VisCloud* visCloud = new VisCloud(h.get());
    _visState->cloudMap.insert(make_pair(frame, visCloud));
  }



  CloserRelationVisualizer::CloserRelationVisualizer(PwnCloser* closer, 
		      VisState* visState_,  
		      PwnTracker* tracker):
    PwnTracker::NewRelationAction(tracker){
    _closer = closer;
    _visState = visState_;
  }

  void CloserRelationVisualizer::compute (PwnTrackerRelation* /*relation*/) {
    int nr=_closer->candidateRelations().size();
    _visState->candidateRelations=_closer->candidateRelations();
    _visState->partitions = _closer->partitions();
    for (size_t i =0; i<_closer->partitions().size(); i++){
      if (&_closer->partitions()[i]==_closer->currentPartition()){
	_visState->currentPartitionIndex = i;
      }
    }
    if (nr) {
      cerr << "CANDIDATE RELATIONS: " << nr << endl;
    }
  }


  MyTrackerViewer::MyTrackerViewer(VisState* s,QWidget *parent, 
				   const QGLWidget *shareWidget, 
				   Qt::WFlags flags): QGLViewer(parent,shareWidget,flags){visState=s;}
  MyTrackerViewer::~MyTrackerViewer() {}
  void MyTrackerViewer::init(){
    // Init QGLViewer.
    QGLViewer::init();
    // Set background color light yellow.
    setBackgroundColor(QColor::fromRgb(255, 255, 194));

    // Set some default settings.
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND); 
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_FLAT);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Don't save state.
    setStateFileName(QString::null);

    // Mouse bindings.
    setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
    setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);

    // Replace camera.
    qglviewer::Camera *oldcam = camera();
    qglviewer::Camera *cam = new StandardCamera();
    setCamera(cam);
    cam->setPosition(qglviewer::Vec(-1.0f, 0.0f, 0.0f));
    cam->setUpVector(qglviewer::Vec(0.0f, 0.0f, 1.0f));
    cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 0.0f));
    delete oldcam;
  }

  void MyTrackerViewer::draw() {
    if (visState)
      visState->draw();
  }

}
