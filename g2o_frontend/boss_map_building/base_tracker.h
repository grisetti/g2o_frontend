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

    class NewKeyNodeMessage: public Serializable{
    public:
      NewKeyNodeMessage(MapNode* kn=0);
      virtual ~NewKeyNodeMessage();
      virtual void serialize(ObjectData& data, IdContext& context);
      virtual void deserialize(ObjectData& data, IdContext& context);
      MapNode* keyNode;
    };

  class BaseTracker: public StreamProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BaseTracker(MapManager* manager_=0, RobotConfiguration* configuration_=0, int id=-1, boss::IdContext* context=0);
    //! initializes the tracker
    virtual void init();
    //! returns the map manager
    inline MapManager* manager() {return _manager;}
    //! returns the map manager
    inline void setManager(MapManager* m) {_manager = m;}

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


    virtual bool shouldChangeKeyNode(MapNodeBinaryRelation* rel);

    //! alignment function you have to implement
    //! @ paeam
    virtual MapNodeBinaryRelation* registerNodes(MapNode* keyNode, 
						 MapNode* otherNode, 
						 const Eigen::Isometry3d& guess = Eigen::Isometry3d::Identity());

    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
  
    virtual ~BaseTracker();
  protected:
    void doStuff();
    //! the manager
    MapManager* _manager; 
    //! previousNode: the last node processed
    //! keyNode: the current keyframe
    //! currentNode: the current node in the pool
    MapNode* _pendingNode, *_keyNode, *_currentNode ;
    //! global position of the tracker
    Eigen::Isometry3d _localT;
    //! queue where to put the output before closing a frame
    std::list<Serializable*> _outputQueue;
  };

}
