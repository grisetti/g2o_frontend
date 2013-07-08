#include "id_correspondence_validator.h"
namespace g2o_frontend{

  bool IdCorrespondenceValidator::operator()(const CorrespondenceVector& correspondences, const IndexVector& indices, int k){
    if (k>minimalSetSize())
      return true;
    
    assert(indices.size()>=k && "VALIDATION_INDEX_OUT_OF_BOUND");
    assert(correspondences.size()>=indices[k] && "VALIDATION_CORRESPONDENCE_INDEX_OUT_OF_BOUND");
    const g2o::OptimizableGraph::Edge* edgek = correspondences[indices[k]].edge();
    int idk1=edgek->vertex(0)->id();
    int idk2=edgek->vertex(1)->id();
    
    for (int i=0; i<k-1; i++){
      const g2o::OptimizableGraph::Edge* edge = correspondences[indices[i]].edge();
      int id1=edge->vertex(0)->id();
      int id2=edge->vertex(1)->id();
      if (idk1==id1)
	return false;
      if (idk2==id2)
	return false;
    }
    return true;
  }

}
