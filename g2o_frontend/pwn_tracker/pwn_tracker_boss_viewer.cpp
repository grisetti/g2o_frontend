#include "set"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss_map/bframe.h"
#include "g2o_frontend/boss_map/bframerelation.h"
#include "g2o_frontend/boss_map/bimagesensor.h"
#include "g2o_frontend/boss_map/blasersensor.h"
#include "g2o_frontend/boss_map/bimusensor.h"
#include "g2o_frontend/boss_map/brobot_configuration.h"
#include "g2o_frontend/boss/bidirectional_serializer.h"
#include <QGLViewer/qglviewer.h>

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "GL/gl.h"
// just to make the linker happy
#include "pwn_tracker.h"
#include "g2o_frontend/boss_map/boss_map_utils.h"
#include "g2o/stuff/opengl_primitives.h"
#include "cache.h"
#include "pwn_tracker_g2o_wrapper.h"
#include "pwn_closer.h"
#include <QApplication>

using namespace pwn;
using namespace pwn_tracker;
using namespace boss_map;
using namespace boss;
using namespace std;

const char* banner[]={
  "pwn_tracker_boss: does SLAM on a raw boss log",
  "usage: pwn_tracker_boss <configfile> <inputFile>",
  "example: pwn_tracker_boss  integral_kinect_3m.conf myInput.log", 
  "the ouptur is written in "
  " - myInput_tracked.log (tracker output)",
  " - myInput_tracked_closed.log (tracker + closer output)",
  " - myInput_tracked.d (binary files)", 
};



void printBanner (){
  int c=0;
  while (banner[c]){
    cerr << banner [c] << endl;
    c++;
  }
}

std::string removeExtension(const std::string& filename) {
    size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos) return filename;
    return filename.substr(0, lastdot); 
}

struct VisCloud{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VisCloud(pwn::Frame* cloud) {
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
  inline void draw() {
    if (drawList)
      glCallList(drawList);
  }
  GLuint drawList;
};

struct VisState{
  VisState(MapManager* manager){
    this->manager = manager;
    final=false;
  }
  std::map<PwnTrackerFrame*,VisCloud*> cloudMap;
  std::vector<std::set <MapNode*> > partitions;
  int currentPartitionIndex;
  std::list<PwnCloserRelation*>  candidateRelations;
  bool final;
  MapManager* manager;
  
  Eigen::Isometry3f sensorPose(PwnTrackerFrame* f){
    Eigen::Isometry3f t;
    convertScalar(t,f->transform());
    return t;/**f->sensorOffset;*/
  }

  void drawRelation(PwnTrackerRelation* r){
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
  void drawFrame(PwnTrackerFrame* f){
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
  void draw(){
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
      for (size_t i=0; i<partitions.size(); i++){
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
  

};

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

class MyTrackerViewer: public QGLViewer{
public:
  MyTrackerViewer(VisState* s,QWidget *parent = 0, 
		  const QGLWidget *shareWidget = 0, 
		  Qt::WFlags flags = 0): QGLViewer(parent,shareWidget,flags){visState=s;}
  virtual ~MyTrackerViewer() {}
  virtual void init(){
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

  virtual void draw() {
    if (visState)
      visState->draw();
  }
  VisState* visState;
};

class MyTracker: public PwnTracker{
public:
  MyTracker(pwn::Aligner* aligner, 
	    pwn::DepthImageConverter* converter, 
	    boss_map::MapManager* manager,
	    PwnCache* cache,
	    PwnCloser* closer,
	    G2oWrapper* optimizer,
	    boss::Serializer* ser):
    PwnTracker(aligner, converter, manager, cache){
    objects.push_back(manager);
    this->ser = ser;
    this->closer = closer;
    this->optimizer = optimizer;
    lastFrameAdded = 0;
    visState = new VisState(manager);
  }  
  virtual void newFrameCallback(PwnTrackerFrame* frame) {
    cerr << "********************* NEW FRAME *********************" << endl;
    objects.push_back(frame);
    ser->writeObject(*frame);
    closer->addFrame(frame);
    lastFrameAdded=frame;
    VisCloud* visCloud = new VisCloud(_cache->get(frame));
    visState->cloudMap.insert(make_pair(frame, visCloud));
  }
  virtual void newRelationCallback(PwnTrackerRelation* relation) {
    objects.push_back(relation);
    ser->writeObject(*relation);
    closer->addRelation(relation);
    cerr << "CLOSER PARTITIONS: " << closer->partitions().size() << endl;
    int cr=0;
    int nr=closer->candidateRelations().size();
    visState->candidateRelations=closer->candidateRelations();
    visState->partitions = closer->partitions();
    for (size_t i =0; i<closer->partitions().size(); i++){
      if (&closer->partitions()[i]==closer->currentPartition()){
	visState->currentPartitionIndex = i;
      }
    }
    for(std::list<PwnCloserRelation*>::iterator it=closer->committedRelations().begin();
	it!=closer->committedRelations().end(); it++){
      objects.push_back(*it);
      cr++;
    }
    if (cr){
      optimizer->optimize();
      // char fname[100];
      // sprintf(fname, "out-%05d.g2o", lastFrameAdded->seq);
      // optimizer->save(fname);
    }
    if (nr) {
      cerr << "CANDIDATE RELATIONS: " << nr << endl;
    }
    if (cr) {
      cerr << "COMMITTED RELATIONS: " << nr << endl;
    }
    
  }
std::list<Serializable*> objects;
  VisState* visState;
protected:
  boss::Serializer* ser;
  PwnCloser* closer;
  G2oWrapper* optimizer;
  PwnTrackerFrame* lastFrameAdded;
};

int main(int argc, char** argv) {
  Deserializer des;
  
  if (argc<3){
    printBanner();
    return 0;
  }

  pwn::Aligner* aligner;  pwn::DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);
  cerr << "config loaded" << endl;
  cerr << " aligner:" << aligner << endl;
  cerr << " converter:" << converter << endl;
    
  des.setFilePath(argv[2]);
  std::vector<BaseSensorData*> sensorDatas;
  RobotConfiguration* conf = readLog(sensorDatas, des);
  cerr << "# frames: " << conf->frameMap().size() << endl;
  cerr << "# sensors: " << conf->sensorMap().size() << endl;
  cerr << "# sensorDatas: " << sensorDatas.size() << endl;

  TSCompare comp;
  std::sort(sensorDatas.begin(), sensorDatas.end(), comp);

  std::string odom_frame_id = conf->baseReferenceFrameId();
  cerr << "base reference frame id: " << odom_frame_id << endl;
  
  std::string depth_topic = "/camera/depth_registered/image_rect_raw";
  //std::string depth_topic = "/camera/depth_registered/image_raw";
  cerr << "depth topic: " << depth_topic << endl;
  

  string outBaseFile;
  if (argc==3){
    outBaseFile = removeExtension(argv[2]);
    cerr << "baseFile: " << outBaseFile << endl;
  } else {
    outBaseFile = removeExtension(argv[3]); 
    cerr << "baseFile: " << outBaseFile << endl;
 }

  string outTrackerFile = outBaseFile+"_tracked.log";
  string outCloserFile = outBaseFile+"_tracked_closed.log";
  string outBinaryDir=outTrackerFile+".d/<classname>.<id>.<ext>";

  cerr << "tracker output: [" << outTrackerFile << "]" << endl;
  cerr << "closer output : [" << outCloserFile << "]" << endl;
  cerr << "binary dir    : [" << outBinaryDir << "]" << endl;

  Serializer ser;
  ser.setFilePath(outTrackerFile);
  ser.setBinaryPath(outBinaryDir);
  MapManager* manager = new MapManager();
  ser.writeObject(*manager);

  // install the optimization wrapper
  G2oWrapper* wrapper = new G2oWrapper(manager);

  int scale = 4;
  // create a cache for the frames
  PwnCache* cache  = new PwnCache(converter, scale, 300);
  PwnCacheHandler* cacheHandler = new PwnCacheHandler(manager, cache);
  manager->actionHandlers().push_back(cacheHandler);
  cacheHandler->init();


  // create a closer
  PwnCloser* closer = new PwnCloser(aligner,converter,manager,cache);
  closer->setScale(scale);
  
  // install a closure criterion to select nodes in the clser
  DistancePoseAcceptanceCriterion criterion(manager);
  criterion.setRotationalDistance(M_PI/4);
  criterion.setTranslationalDistance(1);
  closer->setCriterion(&criterion);
  closer->_debug = false;
  
  MyTracker* tracker=new MyTracker(aligner, converter, manager, cache, closer, wrapper, &ser);
  tracker->_scale = scale;
  tracker->_newFrameInliersFraction = 0.4;
  tracker->init();

  QApplication app(argc,argv);
  MyTrackerViewer *viewer = new MyTrackerViewer(tracker->visState);
  viewer->show();
  
  PinholeImageData* previousImage = 0;
  Eigen::Isometry3d previousPose;
  for (size_t i=0; i<sensorDatas.size(); i++){
    PinholeImageData* imageData = dynamic_cast<PinholeImageData*>(sensorDatas[i]);
    if (imageData && imageData->topic() == depth_topic) {
      Eigen::Isometry3d pose=imageData->robotReferenceFrame()->transform();
      Eigen::Isometry3d sensorOffset_ = conf->sensorOffset(imageData->sensor());
      Eigen::Isometry3d initialGuess_ = previousPose.inverse()*pose;
      Eigen::Matrix3d cameraMatrix_ = imageData->cameraMatrix();

      Eigen::Isometry3f sensorOffset;
      convertScalar(sensorOffset, sensorOffset_);
      Eigen::Isometry3f initialGuess;
      convertScalar(initialGuess, initialGuess_);
      Eigen::Matrix3f cameraMatrix;
      convertScalar(cameraMatrix, cameraMatrix_);

      if (! previousImage){
	initialGuess.setIdentity();
      }	
      
      pwn::DepthImage depthImage;
      ImageBLOB* blob = imageData->imageBlob().get();
      depthImage.fromCvMat(blob->cvImage());
      tracker->processFrame(depthImage, sensorOffset, cameraMatrix, initialGuess);
      delete blob;
      previousPose = pose;
      // delete previousImage
      previousImage = imageData;
      viewer->updateGL();
    }
    app.processEvents();
  }
  tracker->visState->final=true;
  cerr << "done, writing out" << endl;
  ser.setFilePath(outCloserFile);
  for (std::list<Serializable*>::iterator it=tracker->objects.begin(); it!=tracker->objects.end(); it++){
    ser.writeObject(**it);
    cerr << ".";
  }

  viewer->updateGL();
  while(viewer->isVisible()){
    app.processEvents();
  }
  cerr << endl;
}
