#ifndef G2O_FRONTEND_DATA_ASSOCIATION
#define G2O_FRONTEND_DATA_ASSOCIATION

#include "g2o/core/optimizable_graph.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"

namespace g2o {

struct BaseAssociationItem{
  virtual ~BaseAssociationItem();
};

template <typename VertexType_>
struct TypedAssociationItem : public BaseAssociationItem {
  typedef VertexType_ VertexType;
  typedef typename VertexType_::EstimateType_ EstimateType;
  virtual const EstimateType estimate() = 0;
};

template <typename VertexType_, typename EdgeType_, typename FixedVertexType_>
struct UnassignedAssociationItem: public TypedAssociationItem<VertexType_> {
  typedef typename TypedAssociationItem<VertexType_>::VertexType VertexType;
  typedef typename TypedAssociationItem<VertexType_>::VertexType::EstimateType EstimateType;
  typedef FixedVertexType_ FixedVertexType;
  typedef EdgeType_ EdgeType;
  UnassignedAssociationItem(EdgeType* edge_, int assignedVertexIndex_ = 0, int unassignedVertexIndex_=1) {
    _edge = edge_;
    _assignedVertexIndex = assignedVertexIndex_;
    _unassignedVertexIndex = unassignedVertexIndex_;
  }

  EdgeType* edge() {return _edge;}

  const EdgeType* edge() const {return _edge;}

  int unassignedVertexIndex() const {return _unassignedVertexIndex;}

  int assignedVertexIndex() const {return _assignedVertexIndex;}

  virtual const EstimateType estimate() {
    FixedVertexType* assignedVertex = static_cast<FixedVertexType*>(_edge->vertex(_assignedVertexIndex));
    return assignedVertex->estimate() * _edge->measurement();
  }

 protected:
  EdgeType* _edge;
  int _assignedVertexIndex;
  int _unassignedVertexIndex;
};

template <typename VertexType_>
struct AssignedAssociationItem: public TypedAssociationItem<VertexType_> {
  typedef typename TypedAssociationItem<VertexType_>::VertexType VertexType;
  typedef typename TypedAssociationItem<VertexType_>::VertexType::EstimateType EstimateType;

  AssignedAssociationItem(VertexType* vertex_){
    _vertex = vertex_;
  }

  virtual const EstimateType estimate() {
    return _vertex->estimate();
  }
  
 protected:
  VertexType* _vertex;
};

 typedef UnassignedAssociationItem<VertexPointXY, EdgeSE2PointXY, VertexSE2> UnassignedSE2PointXYAssociation;
 typedef AssignedAssociationItem<VertexPointXY> UnassignedPointXYAssociation;

 typedef UnassignedAssociationItem<VertexSE2, EdgeSE2, SE2> UnassignedSE2Association;
 typedef AssignedAssociationItem<VertexSE2> AssignedSE2Association;

 
}
#endif
