#ifndef _BOSS_MAP_H_
#define _BOSS_MAP_H_

#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/boss_logger/bframerelation.h"
#include "g2o_frontend/boss_logger/bsensor.h"


namespace boss_map {
  using namespace boss;
  class MapManager;
  class MapNodeCollection;

  /***************************************** MapNode *****************************************/
  
  /**
     This is a generic map node. Can be a sensor measurement, a sensing frame or a collection of sensing frames.
     It is associated with a map_manager that is in charge of the bookkeeping between elements.
   */
  struct MapNode : public Identifiable {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    friend class MapManager;
    MapNode (MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);
    //! called when all links are resolved, adjusts the bookkeeping of the parents
    virtual void deserializeComplete();
    //! climbs up in the hierarchy of parents by counting the level. The hierarchical map, seen vertically is a DAG
    int level();
    //! returns the manager object
    inline MapManager* manager() const {return _manager;}
    //! isometry to which the map node is referred to
    inline const Eigen::Isometry3d& transform() const {return _transform;}
    //! sets the isometry
    inline void setTransform(const Eigen::Isometry3d& transform_) {_transform = transform_;}
    //! returns the vector of parents
    inline std::vector<MapNodeCollection*>& parents() { return _parents;}
  protected:
    MapManager* _manager;
    std::vector<MapNodeCollection*> _parents;
    Eigen::Isometry3d _transform;
  };

  /***************************************** MapNodeRelation *****************************************/
  
  /**
     This class models a relation between one or more map nodes.
     A relation is a spatial constraint, that originates either by a sensor measurement
     or by a matching algorithm.
     In either case, this class should be specialized to retain the information
     about the specific result (e.g. number of inliers and so on).
   */
  struct MapNodeRelation: public Identifiable{
    //! ctor
    MapNodeRelation (MapManager* manager=0, int id=-1, IdContext* context = 0);
    inline MapManager* manager() {return _manager;}
    inline std::vector<MapNode*>& nodes() {return _nodes;}
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    virtual void deserializeComplete();
    Identifiable* generator() {return _generator;}
    void setGenerator(Identifiable* generator_) {_generator=generator_;}
  protected:
    MapManager* _manager;
    std::vector<MapNode*> _nodes;
    Identifiable* _generator;
  };

  /**
     This class models a generic local map.
     A local map is aset of map nodes and of relations between these nodes.
     these are the internalRelations.
     These relations concur in determining the relative layout of the nodes w.r.t
     one of them (the gauge). 
     This relative layout is expressed as a set of relations between the gauge and the other nodes,
     and stored int he star_relations;
   */
  struct MapNodeCollection: public MapNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //! ctor
    MapNodeCollection (MapManager* manager=0, int id=-1, IdContext* context = 0);
    //! boss serialize
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialize
    virtual void deserialize(ObjectData& data, IdContext& context);
    //! adjusts the bookkeeping when the elements have been loaded
    virtual void deserializeComplete();
    //! returns the guage of the local map. Must belong to one of the children
    inline MapNode* gauge() {return _gauge;}
    //! returns the elements in the local map. A single element can be shared among more than one local map
    inline std::vector<MapNode*>& children() {return _children;}
    //! returns the relations connecting the elements of the local map that have been used to determine a global alignment
    inline std::vector<MapNodeRelation*>& internalRelations() {return _internalRelations;}
    //! returns the summary relations connecting the gauge and all other elements of the local map
    inline std::vector<MapNodeRelation*>& starRelations()  {return _starRelations;}
  protected:
    MapNode* _gauge;
    std::vector<MapNode*> _children;
    std::vector<MapNodeRelation*> _internalRelations;
    std::vector<MapNodeRelation*> _starRelations;
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
  
  
  /***************************************** MapManager********************************/
  class MapManager: public Identifiable {
  public:
    MapManager(int id=-1, IdContext* context = 0);
    //! boss serialization
    virtual void serialize(ObjectData& data, IdContext& context);
    //! boss deserialization
    virtual void deserialize(ObjectData& data, IdContext& context);

    bool addNode(MapNode* n);
    bool addParentToNode(MapNode* child, MapNodeCollection* parent);
    bool removeParentFromNode(MapNode* child, MapNodeCollection* parent);
    bool addRelation(MapNodeRelation* relation);
    bool removeRelation(MapNodeRelation* relation);
    inline std::set<MapNode*>& nodes() {return _nodes;}
    inline std::set<MapNodeRelation*>& relations() {return _relations;}

    std::set<MapNodeRelation*>& nodeRelations(MapNode* n) {
      return _nodesRelationMap.find(n)->second;
    }
  protected:
    std::set<MapNode*> _nodes;
    std::set<MapNodeRelation*> _relations;
    std::map<MapNode*, std::set<MapNodeRelation*> > _nodesRelationMap;
  };

}

#endif
