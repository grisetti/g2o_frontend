#ifndef _G2O_FRONTEND_DISTANCE_VALIDATOR_H_
#define _G2O_FRONTEND_DISTANCE_VALIDATOR_H_
#include "ransac.h"
namespace g2o_frontend{

  template <typename PointVertexType>
  class DistanceCorrespondenceValidator: public CorrespondenceValidator{
  public:
    DistanceCorrespondenceValidator(int mss) :CorrespondenceValidator(mss){
      _intraFrameDistanceDifference=.1;
      _intraFrameMinimalDistance=1.;
    }
    void setIntraFrameDistanceDifference(double idd){ _intraFrameDistanceDifference = idd;}
    void setIntraFrameMinimalDistance(double md){ _intraFrameMinimalDistance = md;}
    double intraFrameDistanceDifference() const {return _intraFrameDistanceDifference;}
    double intraFrameMinimalDistance() const {return _intraFrameMinimalDistance;}

    virtual bool operator()(const CorrespondenceVector& correspondences, const IndexVector& indices, int k){
      if (k>minimalSetSize())
    return true;
      assert(indices.size()>=k && "VALIDATION_INDEX_OUT_OF_BOUND");
      assert(correspondences.size()>=indices[k] && "VALIDATION_CORRESPONDENCE_INDEX_OUT_OF_BOUND");
      const g2o::OptimizableGraph::Edge* edgek = correspondences[indices[k]].edge();
      const PointVertexType* vk1=static_cast<const PointVertexType*>(edgek->vertex(0));
      const PointVertexType* vk2=static_cast<const PointVertexType*>(edgek->vertex(1));
      typename PointVertexType::EstimateType ek1=vk1->estimate();
      typename PointVertexType::EstimateType ek2=vk2->estimate();
      for (int i=0; i<k-1; i++){
    const g2o::OptimizableGraph::Edge* edge = correspondences[indices[i]].edge();
    const PointVertexType* v1=static_cast<const PointVertexType*>(edge->vertex(0));
    const PointVertexType* v2=static_cast<const PointVertexType*>(edge->vertex(1));
    typename PointVertexType::EstimateType e1=v1->estimate();
    typename PointVertexType::EstimateType e2=v2->estimate();

    double d1 = (ek1-e1).norm();
    double d2 = (ek2-e2).norm();
    if (d1<_intraFrameMinimalDistance){
      return false;
    }
    if (d2<_intraFrameMinimalDistance){
      return false;
    }
    if (fabs(d1-d2)>_intraFrameDistanceDifference) {
      return false;
    }
      }
      return true;
    }
  protected:
    double _intraFrameDistanceDifference;
    double _intraFrameMinimalDistance;

  };

}

#endif
