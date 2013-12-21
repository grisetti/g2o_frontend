#include "base_tracker.h"
#include "g2o_frontend/boss_map/sensing_frame_node.h"
#include <iostream>
#include "g2o_frontend/basemath/bm_se3.h"
#include "g2o_frontend/boss/serializable.h"

namespace boss_map_building {
  using namespace std;
  using namespace boss_map;
  using namespace boss;

  NewKeyNodeMessage::NewKeyNodeMessage(MapNode* kn){
    keyNode = kn;
  }
  NewKeyNodeMessage::~NewKeyNodeMessage() {}

  void NewKeyNodeMessage::serialize(ObjectData& data, IdContext&) {
    data.setPointer("keyNode", keyNode);
  }
  void NewKeyNodeMessage::deserialize(ObjectData& data, IdContext&) {
    data.getReference("keyNode").bind(keyNode);
  }
  
  BaseTracker::BaseTracker(MapManager* manager_, RobotConfiguration* configuration_){
    _manager = manager_;
    _robotConfiguration = configuration_;
    init();
  }

  void BaseTracker::init(){
    _pendingNode = 0;
    _previousNode = 0;
    _currentNode = 0;
    _keyNode = 0;
    _globalT.setIdentity();
  }

  Eigen::Isometry3d BaseTracker::computeInitialGuess(MapNode* n_){
    Eigen::Isometry3d t,dt;
    t.setIdentity();
    dt.setIdentity();
    SensingFrameNode *n = dynamic_cast<SensingFrameNode*>(n_);
    
    if (_keyNode) {
      if (n && n->odometry()) {
	dt = n->odometry()->transform();
      }
    }
    if (_previousNode)
      t = _previousNode->transform() * dt;
    //cerr << "dt: " << t2v(dt).transpose() << endl;
    
    if (n && n->imu())
      t.linear() = n->imu()->transform().linear();
    return t; 
  }
	 
  
  void BaseTracker::process(Serializable* s){
    MapNode* n = dynamic_cast<MapNode*>(s);
    if (n) {
      _currentNode = _pendingNode;
      _pendingNode = n;
      doStuff();
    }
    _outputQueue.push_back(s);
  }

  bool BaseTracker::shouldChangeKeyNode(MapNodeBinaryRelation* rel){
    return true;
  }

  void BaseTracker::doStuff(){
    if (! _currentNode)
      return;
    
    //cerr << "*************** NEW NODE " << cnt++ <<  " *************** " << endl;
   
    Eigen::Isometry3d guess = computeInitialGuess(_currentNode);
    _globalT = guess;
    //cerr << "globalT: " << t2v(_globalT).transpose() << endl;
    _currentNode->setTransform(guess);
    //cerr << "guess" << t2v(guess).transpose() << endl;
    if (_keyNode){
      MapNodeBinaryRelation* r = registerNodes(_keyNode, _currentNode, _keyNode->transform().inverse()*guess);
      if (! r){ // matching failed
	//cerr << "TRACK_INIT" << endl;
	cerr << "X";
    	flushQueue();
    	_keyNode = _currentNode;
    	_globalT = guess;
	_outputQueue.push_back(new NewKeyNodeMessage(_keyNode));
      } else { // matching ok
	//cerr << "rel: "  << t2v(r->transform()).transpose() << endl;
	//cerr << "knt: " << t2v(_keyNode->transform()).transpose() << endl;
    	_globalT = (_keyNode->transform()*r->transform());
	Eigen::Matrix3d R = _globalT.linear();
	Eigen::Matrix3d E = R.transpose() * R;
	E.diagonal().array() -= 1;
	_globalT.linear() -= 0.5 * R * E;
	_currentNode->setTransform(_globalT);
    	if (shouldChangeKeyNode(r)){
	  cerr << "K";
    	  //cerr << "KF_CHANGE" << endl;
    	  _outputQueue.push_back(r);
	  flushQueue();
    	  _keyNode = _currentNode;
	  _outputQueue.push_back(new NewKeyNodeMessage(_keyNode));
   	  _manager->addRelation(r);
    	  //cerr << "knt: " << t2v(_keyNode->transform()).transpose() << endl;
	  flushQueue();
    	} else {
	  cerr << "A";
    	  delete r;
    	}
      }
    } else {
      //cerr << "KF_INIT" << endl;
      _keyNode = _currentNode;
      _outputQueue.push_back(new NewKeyNodeMessage(_keyNode));
      //cerr << "knt: " << endl << _keyNode->transform().matrix() << endl;
    }
    _previousNode = _currentNode;
  }

  void BaseTracker::flushQueue(){
    while (! _outputQueue.empty()){
      put(_outputQueue.front());
      _outputQueue.pop_front();
    }
  }

  BaseTracker::~BaseTracker(){}

  MapNodeBinaryRelation* BaseTracker::registerNodes(MapNode* keyNode, MapNode* otherNode, const Eigen::Isometry3d& guess) {
    MapNodeBinaryRelation* rel = new MapNodeBinaryRelation(_manager);
    //cerr << "rel" << keyNode->seq() << " " << otherNode->seq() << endl;
    rel->nodes()[0]=keyNode;
    rel->nodes()[1]=otherNode;
    rel->setTransform(guess);
    //cerr << "k: " << endl << keyNode->transform().matrix() << endl;
    //cerr << "o: " << endl << otherNode->transform().matrix() << endl;
    //cerr << "t: " << endl << rel->transform().matrix() << endl;
    rel->setOwner(keyNode);
    Eigen::Matrix<double, 6,6> info;
    info.setIdentity();
    rel->setInformationMatrix(info);
    return rel;
  }

  BOSS_REGISTER_CLASS(NewKeyNodeMessage);
};
