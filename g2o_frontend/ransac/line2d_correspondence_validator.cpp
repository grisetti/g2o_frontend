#include "line2d_correspondence_validator.h"

namespace g2o_frontend{
  using namespace g2o;
  
  bool Line2DCorrespondenceValidator::operator()(const CorrespondenceVector& correspondences, const IndexVector& indices, int k){
    if (k>minimalSetSize())
      return true;
    assert(indices.size()>=k && "VALIDATION_INDEX_OUT_OF_BOUND");
    assert(correspondences.size()>=indices[k] && "VALIDATION_CORRESPONDENCE_INDEX_OUT_OF_BOUND");
    const g2o::OptimizableGraph::Edge* edgek = correspondences[indices[k]].edge();
    const VertexLine2D* vk1=static_cast<const VertexLine2D*>(edgek->vertex(0));
    const VertexLine2D* vk2=static_cast<const VertexLine2D*>(edgek->vertex(1));
    Line2D ek1=vk1->estimate();
    Line2D ek2=vk2->estimate();
    for (int i=0; i<k-1; i++){
      const g2o::OptimizableGraph::Edge* edge = correspondences[indices[i]].edge();
      const VertexLine2D* v1=static_cast<const VertexLine2D*>(edge->vertex(0));
      const VertexLine2D* v2=static_cast<const VertexLine2D*>(edge->vertex(1));
      Line2D e1=v1->estimate();
      Line2D e2=v2->estimate();
	
      double d1 = ek1(0)-e1(0);
      d1=atan2(sin(d1),cos(d1));
      double d2 = ek2(0)-e2(0);
      d2=atan2(sin(d2),cos(d2));
	
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

}


