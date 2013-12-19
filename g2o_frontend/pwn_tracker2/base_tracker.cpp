#include "base_tracker.h"
#include "g2o_frontend/boss_map/sensing_frame_node.h"
#include <iostream>
#include "g2o_frontend/basemath/bm_se3.h"

namespace boss_map_building {
  using namespace std;
  using namespace boss_map;
  using namespace boss;
  
  BaseTracker::BaseTracker(MapManager* manager_, RobotConfiguration* configuration_){
    _manager = manager_;
    _robotConfiguration = configuration_;
    init();
  }

  void BaseTracker::init(){
    _previousNode = 0;
    _keyNode = 0;
    _currentNode = 0;
    _globalT.setIdentity();
    _previousNodeTransform.setIdentity();
    _currentNodeTransform.setIdentity();
  }

  Eigen::Isometry3d BaseTracker::computeInitialGuess(MapNode* n_){
    Eigen::Isometry3d t,dt;
    t.setIdentity();
    dt.setIdentity();
    SensingFrameNode *n = dynamic_cast<SensingFrameNode*>(n_);
    
    if (_keyNode) {
      dt = _previousNodeTransform*_currentNodeTransform;
      if (n && n->odometry()) {
	dt = n->odometry()->transform();
      }
    }
    t = _globalT * dt;
    //cerr << "dt: " << t2v(dt).transpose() << endl;
    
    if (n && n->imu())
      t.linear() = n->imu()->transform().linear();
    return t; 
  }
	 
  ofstream os("ptrack.txt");
  
  void BaseTracker::process(Serializable* s){
    _outputQueue.push_back(s);
    MapNode* n = dynamic_cast<MapNode*>(s);
    if (! n) {
      return;
    }
    
    //cerr << "*************** NEW NODE " << cnt++ <<  " *************** " << endl;
   
    _currentNode = n;
    _currentNodeTransform = n->transform();
    lock(_currentNode);
    Eigen::Isometry3d guess = computeInitialGuess(n);
    _globalT = guess;
    //cerr << "globalT: " << t2v(_globalT).transpose() << endl;
    _currentNode->setTransform(guess);
    //cerr << "guess" << t2v(guess).transpose() << endl;
    if (_keyNode){
      MapNodeBinaryRelation* r = registerNodes(_keyNode, _currentNode);
      if (! r){ // matching failed
	//cerr << "TRACK_INIT" << endl;
	cerr << "X";
    	flushQueue();
    	unlock(_keyNode);
    	_keyNode = _currentNode;
	_keyNode->setTransform(guess);
	os << t2v(_keyNode->transform()).transpose() << endl;
    	_globalT = guess;
      } else { // matching ok
	//cerr << "rel: "  << t2v(r->transform()).transpose() << endl;
	//cerr << "knt: " << t2v(_keyNode->transform()).transpose() << endl;
    	_globalT = (_keyNode->transform()*r->transform());

	Eigen::Matrix3d R = _globalT.linear();
	Eigen::Matrix3d E = R.transpose() * R;
	E.diagonal().array() -= 1;
	_globalT.linear() -= 0.5 * R * E;
	_currentNode->setTransform(_globalT);
    	if (shouldChangeKeyframe(r)){
	  cerr << "K";
    	  //cerr << "KF_CHANGE" << endl;
	  flushQueue();
    	  _keyNode = _currentNode;
    	  _outputQueue.push_back(r);
    	  _manager->addRelation(r);
    	  //cerr << "knt: " << t2v(_keyNode->transform()).transpose() << endl;
	  os << t2v(_keyNode->transform()).transpose() << endl;
	  unlock(_keyNode);
	  flushQueue();
    	} else {
	  cerr << "A";
    	  delete r;
    	  unlock(_currentNode);
    	}
      }
    } else {
      //cerr << "KF_INIT" << endl;
      _keyNode = _currentNode;
      //cerr << "knt: " << endl << _keyNode->transform().matrix() << endl;
      os << t2v(_keyNode->transform()).transpose() << endl;
    }
    _previousNode = _currentNode;
    _previousNodeTransform = _currentNodeTransform;
  }

  void BaseTracker::flushQueue(){
    while (! _outputQueue.empty()){
      if (_outputQueue.front() == _keyNode)
	return;
      put(_outputQueue.front());
      _outputQueue.pop_front();
    }
  }

  void BaseTracker::lock(MapNode*) {}

  void BaseTracker::unlock(MapNode*) {}

  bool BaseTracker::shouldChangeKeyframe(MapNodeBinaryRelation* ) { 
    return true;
  }

  BaseTracker::~BaseTracker(){}

  MapNodeBinaryRelation* BaseTracker::registerNodes(MapNode* keyNode, MapNode* otherNode) {
    MapNodeBinaryRelation* rel = new MapNodeBinaryRelation(_manager);
    //cerr << "rel" << keyNode->seq() << " " << otherNode->seq() << endl;
    rel->nodes()[0]=keyNode;
    rel->nodes()[1]=otherNode;
    rel->setTransform(keyNode->transform().inverse()*otherNode->transform());
    //cerr << "k: " << endl << keyNode->transform().matrix() << endl;
    //cerr << "o: " << endl << otherNode->transform().matrix() << endl;
    //cerr << "t: " << endl << rel->transform().matrix() << endl;
    rel->setOwner(keyNode);
    Eigen::Matrix<double, 6,6> info;
    info.setIdentity();
    rel->setInformationMatrix(info);
    return rel;
  }

};
