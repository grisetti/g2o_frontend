#ifndef _BOSS_MAP_UTILS_H_
#define _BOSS_MAP_UTILS_H_

#include "map_manager.h"
#include "g2o_frontend/boss/identifiable.h"
#include "g2o_frontend/basemath/bm_se3.h"

namespace boss_map {

  class NodeAcceptanceCriterion: public boss::Identifiable{
  public:
    NodeAcceptanceCriterion(MapManager* manager_=0, int id = -1, boss::IdContext* context = 0);
    virtual bool accept(MapNode* n) = 0;
    virtual ~NodeAcceptanceCriterion();
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    inline MapManager* manager() {return _manager;}
    inline void setManager(MapManager* manager_) {_manager = manager_;}
  protected:
    MapManager* _manager;
  };

  class PoseAcceptanceCriterion: public NodeAcceptanceCriterion {
  public:
    PoseAcceptanceCriterion(MapManager* manager_=0, int id = -1, boss::IdContext* context = 0);
    virtual void setReferencePose(const Eigen::Isometry3d& pose_) {_pose = pose_; _invPose=_pose.inverse();}
    inline const Eigen::Isometry3d referencePose() const { return _pose;}
  protected:
    Eigen::Isometry3d _pose;
    Eigen::Isometry3d _invPose;
  };

  class DistancePoseAcceptanceCriterion: public PoseAcceptanceCriterion{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    DistancePoseAcceptanceCriterion(MapManager* manager_=0, int id = -1, boss::IdContext* context = 0);
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    virtual bool accept(MapNode* n);
    inline double translationalDistance() const { return _translationalDistance;}
    inline void setTranslationalDistance(double td)  { _translationalDistance = td; _td2=td*td;}
    inline double rotationalDistance() const { return _rotationalDistance;}
    inline void setRotationalDistance(double rd)  { _rotationalDistance = rd; _td2=rd*rd;}
  protected:
    double _translationalDistance, _rotationalDistance;
    double _td2; // squared distances
  };

  class MahalanobisPoseAcceptanceCriterion: public PoseAcceptanceCriterion{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    MahalanobisPoseAcceptanceCriterion(MapManager* manager_=0, int id = -1, boss::IdContext* context = 0);
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    virtual bool accept(MapNode* n);
    inline void setDistance(double d) {_distance = d;}
    inline double distance() const {return _distance;}
    inline const Matrix6d& informationMatrix() const {return _info;}
    inline void  setInformationMatrix(const Matrix6d& info_)  {_info = info_;}
  protected:
    Matrix6d _info;
    double _distance;
  };


  class MapRelationSelector: public boss::Identifiable{
  public:
    MapRelationSelector(MapManager* manager_=0, int id = -1, boss::IdContext* context = 0);
    virtual bool accept(MapNodeRelation* r) = 0;
    virtual ~MapRelationSelector();
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    inline MapManager* manager() {return _manager;}
    inline void setManager(MapManager* manager_) {_manager = manager_;}
  protected:
    MapManager* _manager;
  };


  void selectNodes(std::set<MapNode*>& nodes, NodeAcceptanceCriterion* criterion);
  void extractInternalRelations(std::set<MapNodeRelation*>& internalRelations, 
				std::set<MapNode*>& nodes, 
				MapManager* manager);
  void makePartitions(std::vector<std::set<MapNode*> >& partitions,
		      std::set<MapNode*>& nodes, MapRelationSelector* relationSelector=0);
}

#endif
