#include "pwn_tracker.h"
#include "g2o_frontend/pwn_core/pwn_static.h"
#include "g2o/stuff/timeutil.h"

namespace pwn_tracker{

  using namespace std;
  using namespace pwn;
  using namespace boss;

  

  
  PwnTracker::PwnTrackerAction::PwnTrackerAction(PwnTracker* tracker_) {
    _tracker = tracker_;
  }
  
  PwnTracker::NewFrameAction::NewFrameAction(PwnTracker* tracker_): PwnTrackerAction(tracker_) {}

  PwnTracker::AlignmentAction::AlignmentAction(PwnTracker* tracker_): PwnTrackerAction(tracker_) {}

  PwnTracker::NewRelationAction::NewRelationAction(PwnTracker* tracker_): PwnTrackerAction(tracker_) {}

  PwnTracker::InitAction::InitAction(PwnTracker* tracker_): PwnTrackerAction(tracker_) {}



  PwnTracker::PwnTracker(pwn::Aligner* aligner, 
			 pwn::DepthImageConverter* converter, 
			 boss_map::MapManager* manager,
			 PwnCache* cache_): PwnMatcherBase(aligner, converter) {
    _previousCloud = 0;
    _globalT.setIdentity();
    _previousCloudTransform.setIdentity();
    _counter = 0;
    _numKeyframes = 0;
    _newFrameInliersFraction = 0.4;
    this->_manager = manager;
    _cache=cache_;
    cumTime = 0;
  }

  void PwnTracker::init(){
    if (_previousTrackerFrame)
      delete _previousTrackerFrame;
    if (_previousCloud)
      delete _previousCloud;
    _previousCloud = 0;
    _globalT.setIdentity();
    _previousCloudTransform.setIdentity();
    _counter = 0;
    _numKeyframes = 0;
    _seq = 0;
    initCallbacks();
  }

  /*
  PwnTrackerFrame* PwnTracker::makeFrame() {
    PwnTrackerFrame* currentTrackerFrame = new PwnTrackerFrame(_manager);
    //currentTrackerFrame->cloud.set(cloud);
    currentTrackerFrame->cloud=_currentCloud;
    
    boss_map::ImageBLOB* depthBLOB=new boss_map::ImageBLOB();
    DepthImage_convert_32FC1_to_16UC1(depthBLOB->cvImage(),depthImage);
    //depthImage.toCvMat(depthBLOB->cvImage());
    depthBLOB->adjustFormat();
    currentTrackerFrame->depthImage.set(depthBLOB);
    
    boss_map::ImageBLOB* normalThumbnailBLOB=new boss_map::ImageBLOB();
    boss_map::ImageBLOB* depthThumbnailBLOB=new boss_map::ImageBLOB();
    makeThumbnails(depthThumbnailBLOB->cvImage(), normalThumbnailBLOB->cvImage(), cloud, 
		   depthImage.rows, 
		   depthImage.cols,
		   sensorOffset,
		   cameraMatrix, 
		   0.1);
    normalThumbnailBLOB->adjustFormat();
    depthThumbnailBLOB->adjustFormat();
    
    currentTrackerFrame->depthThumbnail.set(depthThumbnailBLOB);
    currentTrackerFrame->normalThumbnail.set(normalThumbnailBLOB);
    currentTrackerFrame->cameraMatrix = cameraMatrix;
    currentTrackerFrame->imageRows = depthImage.rows;
    currentTrackerFrame->imageCols = depthImage.cols;
    Eigen::Isometry3d _iso;
    convertScalar(_iso, _globalT);
    currentTrackerFrame->setTransform(_iso);
    convertScalar(currentTrackerFrame->sensorOffset, sensorOffset);
    currentTrackerFrame->setSeq(_seq++);
    _manager->addNode(currentTrackerFrame);
    //cerr << "AAAA" << endl;
    
    if (_cache)
      _currentCloudHandle=_cache->get(currentTrackerFrame);
    //cerr << "BBBB" << endl;
    //_cache->lock(currentTrackerFrame);
    newFrameCallbacks(currentTrackerFrame);
    currentTrackerFrame->depthThumbnail.set(0);
    currentTrackerFrame->normalThumbnail.set(0);
    currentTrackerFrame->depthImage.set(0);
    return currentTrackerFrame;
  }
  */

  void PwnTracker::processFrame(const DepthImage& depthImage_, 
				const Eigen::Isometry3f& sensorOffset_, 
				const Eigen::Matrix3f& cameraMatrix_,
				const Eigen::Isometry3f& initialGuess){
    int r,c;
    Eigen::Matrix3f scaledCameraMatrix = cameraMatrix_;
    _currentCloudOffset = sensorOffset_;
    _currentCameraMatrix = cameraMatrix_;
    _currentDepthImage = depthImage_;
    _currentCloud = makeCloud(r,c,scaledCameraMatrix,_currentCloudOffset,_currentDepthImage);
    
    bool newFrame = false;
    bool newRelation = false;
    PwnTrackerFrame* currentTrackerFrame=0;
    Eigen::Isometry3d relationMean;
    if (_previousCloud){
      _aligner->setCurrentSensorOffset(_currentCloudOffset);
      _aligner->setCurrentFrame(_currentCloud);
      _aligner->setReferenceSensorOffset(_previousCloudOffset);
      _aligner->setReferenceFrame(_previousCloud);

      _aligner->correspondenceFinder()->setImageSize(r,c);
      PinholePointProjector* alprojector=dynamic_cast<PinholePointProjector*>(_aligner->projector());
      alprojector->setCameraMatrix(scaledCameraMatrix);
      alprojector->setImageSize(r,c);

      Eigen::Isometry3f guess=_previousCloudTransform.inverse()*_globalT*initialGuess;
      _aligner->setInitialGuess(guess);
      double t0, t1;
      t0 = g2o::get_time();
      _aligner->align();

      t1 = g2o::get_time();
      std::cout << "Time: " << t1 - t0 << " seconds " << std::endl;
 
      cerr << "inliers: " << _aligner->inliers() << endl;
      // cerr << "chi2: " << _aligner->error() << endl;
      // cerr << "chi2/inliers: " << _aligner->error()/_aligner->inliers() << endl;
      // cerr << "initialGuess: " << t2v(guess).transpose() << endl;
      // cerr << "transform   : " << t2v(_aligner->T()).transpose() << endl;
      if (_aligner->inliers()>0){
    	_globalT = _previousCloudTransform*_aligner->T();
    	//cerr << "TRANSFORM FOUND" <<  endl;
      } else {
    	//cerr << "FAILURE" <<  endl;
    	_globalT = _globalT*guess;
      }
      convertScalar(relationMean, _aligner->T());
      if (! (_counter%50) ) {
    	Eigen::Matrix3f R = _globalT.linear();
    	Eigen::Matrix3f E = R.transpose() * R;
    	E.diagonal().array() -= 1;
    	_globalT.linear() -= 0.5 * R * E;
      }
      _globalT.matrix().row(3) << 0,0,0,1;
 
      newAlignmentCallbacks(_globalT, _aligner->T(), _aligner->inliers(), _aligner->error());
			   
      int maxInliers = r*c;
      float inliersFraction = (float) _aligner->inliers()/(float) maxInliers;
      cerr << "inliers/maxinliers/fraction: " << _aligner->inliers() << "/" << maxInliers << "/" << inliersFraction << endl;
      if (inliersFraction<_newFrameInliersFraction){
	newFrame = true;
	newRelation = true;

	// char filename[1024];
	// sprintf (filename, "frame-%05d.pwn", _numKeyframes);
	// frame->save(filename,1,true,_globalT);

	_numKeyframes ++;
	if (!_cache) 
	  delete _previousCloud;
	else{
	  _previousCloudHandle.release();
	}
	//_cache->unlock(_previousTrackerFrame);
	// _aligner->setReferenceSensorOffset(_currentCloudOffset);
	// _aligner->setReferenceFrame(_currentCloud);
	_previousCloud = _currentCloud;
	_previousCloudTransform = _globalT;
	// cerr << "new frame added (" << _numKeyframes <<  ")" << endl;
	// cerr << "inliers: " << _aligner->inliers() << endl;
	// cerr << "maxInliers: " << maxInliers << endl;
	// cerr << "chi2: " << _aligner->error() << endl;
	// cerr << "chi2/inliers: " << _aligner->error()/_aligner->inliers() << endl;
	// cerr << "initialGuess: " << t2v(guess).transpose() << endl;
	// cerr << "transform   : " << t2v(_aligner->T()).transpose() << endl;
	// cerr << "globalTransform   : " << t2v(_globalT).transpose() << endl;
      } else { // previous frame but offset is small
	delete _currentCloud;
	_currentCloud = 0;
      }
    } else { // first frame
      //ser.writeObject(*manager);
      newFrame = true;
      // _aligner->setReferenceSensorOffset(_currentCloudOffset);
      // _aligner->setReferenceFrame(_currentCloud);
      _previousCloud = _currentCloud;
      _previousCloudTransform = _globalT;
      _previousCloudOffset = _currentCloudOffset;
      _numKeyframes ++;
      /*Eigen::Isometry3f t = _globalT;
	geometry_msgs::Point p;
	p.x = t.translation().x();
	p.y = t.translation().y();
	p.z = t.translation().z();
	m_odometry.points.push_back(p);
      */
    }
    _counter++;

    if (newFrame) {
      //cerr << "maing new frame, previous: " << _previousTrackerFrame << endl;
      currentTrackerFrame = new PwnTrackerFrame(_manager);
      //currentTrackerFrame->cloud.set(cloud);
      currentTrackerFrame->cloud=_currentCloud;
      currentTrackerFrame->sensorOffset = _currentCloudOffset;
      boss_map::ImageBLOB* depthBLOB=new boss_map::ImageBLOB();
      DepthImage_convert_32FC1_to_16UC1(depthBLOB->cvImage(),_currentDepthImage);
      //depthImage.toCvMat(depthBLOB->cvImage());
      depthBLOB->adjustFormat();
      currentTrackerFrame->depthImage.set(depthBLOB);
      
      boss_map::ImageBLOB* normalThumbnailBLOB=new boss_map::ImageBLOB();
      boss_map::ImageBLOB* depthThumbnailBLOB=new boss_map::ImageBLOB();
      makeThumbnails(depthThumbnailBLOB->cvImage(), normalThumbnailBLOB->cvImage(), _currentCloud, 
		    _currentDepthImage.rows, 
		    _currentDepthImage.cols,
		    _currentCloudOffset,
		    _currentCameraMatrix, 
		    0.1);
      normalThumbnailBLOB->adjustFormat();
      depthThumbnailBLOB->adjustFormat();
      
      currentTrackerFrame->depthThumbnail.set(depthThumbnailBLOB);
      currentTrackerFrame->normalThumbnail.set(normalThumbnailBLOB);
      currentTrackerFrame->cameraMatrix = _currentCameraMatrix;
      currentTrackerFrame->imageRows = _currentDepthImage.rows;
      currentTrackerFrame->imageCols = _currentDepthImage.cols;
      Eigen::Isometry3d _iso;
      convertScalar(_iso, _globalT);
      currentTrackerFrame->setTransform(_iso);
      convertScalar(currentTrackerFrame->sensorOffset, _currentCloudOffset);
      currentTrackerFrame->setSeq(_seq++);
      _manager->addNode(currentTrackerFrame);
      //cerr << "AAAA" << endl;

      if (_cache)
	_currentCloudHandle=_cache->get(currentTrackerFrame);
      //cerr << "BBBB" << endl;
      //_cache->lock(currentTrackerFrame);
      newFrameCallbacks(currentTrackerFrame);
      currentTrackerFrame->depthThumbnail.set(0);
      currentTrackerFrame->normalThumbnail.set(0);
      currentTrackerFrame->depthImage.set(0);
    }

    if (newRelation) {
      PwnTrackerRelation* rel = new PwnTrackerRelation(_manager);
      rel->setTransform(relationMean);
      Matrix6d omega;
      convertScalar(omega, _aligner->omega());
      omega.setIdentity();
      omega *= 100;
      rel->setInformationMatrix(omega);
      rel->setTo(currentTrackerFrame);
      rel->setFrom(_previousTrackerFrame);
      //cerr << "saved relation" << _previousTrackerFrame << " " << currentTrackerFrame << endl;
      _manager->addRelation(rel);
      newRelationCallbacks(rel);
      //ser.writeObject(*rel);
    }

    if (currentTrackerFrame) {
      _previousTrackerFrame = currentTrackerFrame;
    }
  }

  PwnTracker::~PwnTracker() {
    init();
  }  



  void PwnTracker::newFrameCallbacks(PwnTrackerFrame* frame) {
    for (std::list<NewFrameAction*>::iterator it = _newFrameActions.begin(); it!=_newFrameActions.end(); it++){
      NewFrameAction* a = *it;
      a->compute(frame);
    }
  }

  void PwnTracker::newAlignmentCallbacks(const Eigen::Isometry3f& globalT, 
					 const Eigen::Isometry3f& localT, 
					 int inliers, float error ) {
    for (std::list<AlignmentAction*>::iterator it = _alignmentActions.begin(); it!=_alignmentActions.end(); it++){
      AlignmentAction* a = *it;
      a->compute(globalT, localT, inliers, error);
    }

  }
  void PwnTracker::newRelationCallbacks(PwnTrackerRelation* relation) {
    for (std::list<NewRelationAction*>::iterator it = _newRelationActions.begin(); it!=_newRelationActions.end(); it++){
      NewRelationAction* a = *it;
      a->compute(relation);
    }

  }
  void PwnTracker::initCallbacks() {
    for (std::list<InitAction*>::iterator it = _initActions.begin(); it!=_initActions.end(); it++){
      InitAction* a = *it;
      a->compute();
    }
  }


  BOSS_REGISTER_CLASS(PwnTrackerFrame);
  BOSS_REGISTER_CLASS(PwnTrackerRelation);

}
