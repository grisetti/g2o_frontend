#pragma once

#include <list>
#include <set>
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss_map/reference_frame.h"
#include "g2o_frontend/boss_map/reference_frame_relation.h"
#include "g2o_frontend/boss_map/image_sensor.h"
#include "g2o_frontend/boss_map/laser_sensor.h"
#include "g2o_frontend/boss_map/imu_sensor.h"
#include "g2o_frontend/boss_map/sensor_data_synchronizer.h"
#include "g2o_frontend/boss_map/robot_configuration.h"
#include "g2o_frontend/boss_map/map_manager.h"


namespace boss_map_building {
  using namespace boss_map;
  using namespace boss;
  class BaseTracker: public StreamProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BaseTracker(MapManager* manager_=0, RobotConfiguration* configuration_=0);
    //! initializes the tracker
    virtual void init();
    //! returns the robot configuration
    inline RobotConfiguration* robotConfiguration() {return _robotConfiguration;}
    //! returns the map manager
    inline MapManager* manager() {return _manager;}
    //! computes the initial guess of a node, by taking into account the
    //! - global transform
    //! - the relative displacement from the initial positions of the previous and the current nodes
    //! the odometry (if available) and the imu (if available)
    //! @param n_: the node;
    //! @returns: the global position of the node
    virtual Eigen::Isometry3d computeInitialGuess(MapNode* n_);
	 				
    //! processes a new incoming data
    //! if the data is not a node, it is just propagated as output
    virtual void process(Serializable* s);

    //! flushes the queue, by doing repeated outputs
    void flushQueue();
    //! declares a node is used and its temporary information cannot be removed from the cache
    virtual void lock(MapNode* n);
    //! declares a node is not used and it its temporaries may be removed if needed
    virtual void unlock(MapNode* n);
    //! true if the frames linked by the relation are different enough but they are still aligned
    //! this involves the creation of a new keyframe and the reconstruction of the bookkeping
    virtual bool shouldChangeKeyframe(MapNodeBinaryRelation* r);
    //! alignment function you have to implement
    //! @ paeam
    virtual MapNodeBinaryRelation* registerNodes(MapNode* keyNode, MapNode* otherNode);
  
    virtual ~BaseTracker();
  protected:
    //! the manager
    MapManager* _manager; 
    //! the robot configuration
    RobotConfiguration* _robotConfiguration;
    //! previousNode: the last node processed
    //! keyNode: the current keyframe
    //! currentNode: the current node in the pool
    MapNode* _previousNode, *_keyNode, *_currentNode ;
    //! transforms of the nodes, when they are read from the input stream
    Eigen::Isometry3d _previousNodeTransform, _currentNodeTransform;
    //! global position of the tracker
    Eigen::Isometry3d _globalT;
    //! queue where to put the output before closing a frame
    std::list<Serializable*> _outputQueue;
  };
}
