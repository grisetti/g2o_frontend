#ifndef _BOSS_MAP_H_
#define _BOSS_MAP_H_

#include "reference_frame.h"
#include "reference_frame_relation.h"
#include "sensor.h"

namespace boss_map {
  using namespace boss;
  class MapManager;
  class MapNodeRelation;

  /***************************************** MapItem *****************************************/  
  /**An item of the map (can be either a relation or a node)*/
  class MapItem: public Identifiable {
  public:
    MapItem (MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! returns the manager object
    inline MapManager* manager() const {return _manager;}
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);
  protected:
    MapManager * _manager;
  };

  /***************************************** MapNode *****************************************/  
  /**
     This is a generic map node. Can be a sensor measurement, a sensing frame or a collection of sensing frames.
     It is associated with a map_manager that is in charge of the bookkeeping between elements.
   */
  class MapNode : public MapItem {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    friend class MapManager;
    MapNode (MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);
    //! called when all links are resolved, adjusts the bookkeeping of the parents
    virtual void deserializeComplete();
    //! isometry to which the map node is referred to
    virtual const Eigen::Isometry3d& transform() const {return _transform;}
    //! sets the isometry
    virtual void setTransform(const Eigen::Isometry3d& transform_) {_transform = transform_;}

    //! a useful sequence number
    inline int seq() const { return _seq; };
    inline void setSeq( int seq_) { _seq = seq_; };

    //! gets level of hierarchy of this node
    inline int level() const { return _level;}
    //! sets the level of this node
    inline void setLevel(int level_) {_level=level_;}
    
    //! compute the parent nodes, scanning for all nodes that belong to a relation owned by another node
    std::vector<MapNode*> parents();

    //! compute the parent relations, scanning for all nodes that belong to a relation  owned by this
    std::vector<MapNodeRelation*> parentRelations();

    //! compute the children nodes, scanning for all relations whose owner is this node
    //! and whose level is the current level minus one
    std::vector<MapNode*> children();

    //! computes the children relations, that are all relations whose owner is this one
    //! and whose level is the current one minus one
    std::vector<MapNodeRelation*> childrenRelations();

  protected:
    MapManager* _manager;
    int _level;
    int _seq;
    Eigen::Isometry3d _transform;
  };

  class MapNodeAlias: public MapNode{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    MapNodeAlias(MapNode* original=0, MapManager* manager=0, int id=-1, IdContext* context = 0);
    inline MapNode* originalNode() {return _original;}
    inline void setOriginalNode(MapNode* original_){
      _original = original_; 
      if(_original) 
	_level=_original->level()+1;
    }
    //! isometry to which the map node is referred to
    virtual const Eigen::Isometry3d& transform() const;
    //! sets the isometry
    virtual void setTransform(const Eigen::Isometry3d& transform_);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);
  protected:
    MapNode* _original;
  };

  /***************************************** MapNodeRelation *****************************************/
  
  /**
     This class models a relation between one or more map nodes.
     A relation is a spatial constraint, that originates either by a sensor measurement
     or by a matching algorithm.
     In either case, this class should be specialized to retain the information
     about the specific result (e.g. number of inliers and so on).
   */
  struct MapNodeRelation: public MapItem{
    //! ctor
    MapNodeRelation (MapManager* manager=0, int id=-1, IdContext* context = 0);
    inline std::vector<MapNode*>& nodes() {return _nodes;}
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    virtual void deserializeComplete();
    Identifiable* generator() {return _generator;}
    void setGenerator(Identifiable* generator_) {_generator=generator_;}
    //! gets level of hierarchy of this relation
    MapNode* owner() { return _owner; }
    void setOwner(MapNode* owner_) { _owner = owner_; }
  protected:
    MapNode* _owner;
    std::vector<MapNode*> _nodes;
    Identifiable* _generator;
  };
  

  struct MapNodeAliasRelation: public MapNodeRelation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    MapNodeAliasRelation(MapNode* owner = 0, MapManager* manager=0, int id=-1, IdContext* context = 0);
  };

  /***************************************** MapNodeBinaryRelation *****************************************/

  struct MapNodeBinaryRelation: public MapNodeRelation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    MapNodeBinaryRelation(MapManager* manager=0, int id=-1, IdContext* context = 0);
    //virtual ~MapNodeBinaryRelation();
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);
    //! called when all links are resolved, adjusts the bookkeeping of the parents
    //virtual void deserializeComplete();
    //! isometry to which the map node is referred to
    inline const Eigen::Isometry3d& transform() const {return _transform;}
    //! sets the isometry
    inline void setTransform(const Eigen::Isometry3d& transform_) {_transform = transform_;}
    inline const Eigen::Matrix<double, 6,6>& informationMatrix() {return _informationMatrix;}
    inline void setInformationMatrix(const Eigen::Matrix<double, 6,6>& informationMatrix_) {_informationMatrix=informationMatrix_;}
  protected:
    Eigen::Matrix<double, 6,6> _informationMatrix;
    Eigen::Isometry3d _transform;
  };
  
  /***************************************** MapNodeUnaryRelation *****************************************/
  struct MapNodeUnaryRelation: public MapNodeRelation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 
    MapNodeUnaryRelation(MapManager* manager=0, int id=-1, IdContext* context = 0);
    //virtual ~MapNodeUnaryRelation();
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);
    //! called when all links are resolved, adjusts the bookkeeping of the parents
    //virtual void deserializeComplete();
    //! isometry to which the map node is referred to
    inline const Eigen::Isometry3d& transform() const {return _transform;}
    //! sets the isometry
    inline void setTransform(const Eigen::Isometry3d& transform_) {_transform = transform_;}
    inline const Eigen::Matrix<double, 6,6>& informationMatrix() {return _informationMatrix;}
    inline void setInformationMatrix(const Eigen::Matrix<double, 6,6>& informationMatrix_) {_informationMatrix=informationMatrix_;}
  protected:
    Eigen::Matrix<double, 6,6> _informationMatrix;
    Eigen::Isometry3d _transform;
  };
 
}

#endif
