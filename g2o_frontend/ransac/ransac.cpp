#include "ransac.h"

namespace g2o_frontend {
  using namespace g2o;
  using namespace Eigen;

  CorrespondenceValidator::~CorrespondenceValidator(){}
  
  BaseGeneralizedRansac::BaseGeneralizedRansac(int minimalSetSize_):_minimalSetSize(minimalSetSize_),_indices(minimalSetSize_, 0){ }
  BaseGeneralizedRansac::~BaseGeneralizedRansac(){};

  void BaseGeneralizedRansac::setCorrespondences(const CorrespondenceVector& correspondences_){
    _correspondences = correspondences_; 
    _errors.resize(_correspondences.size());
    _vertices1.resize(_correspondences.size());
    _vertices2.resize(_correspondences.size());
    for (size_t i = 0; i<_correspondences.size(); i++){
      OptimizableGraph::Edge* edge=_correspondences[i].edge();
      assert(edge->vertices.size()==2 && "THE_NUMBER_OF_VERTICES_IN_THE_EDGE_FOR_RANSAC_SHOULD BE_2");
      _vertices1[i]=static_cast<OptimizableGraph::Vertex*>(edge->vertex(0));
      assert(_vertices1[i] && "NULL_EDGE_FOR_RANSAC");
      _vertices2[i]=static_cast<OptimizableGraph::Vertex*>(edge->vertex(1));
      assert(_vertices2[i] && "NULL_EDGE_FOR_RANSAC");
    }
    _verticesPushed = false;
  }
  
  bool BaseGeneralizedRansac::validateCorrespondences(int k){
    for (CorrespondenceValidatorPtrVector::iterator it=_correspondenceValidators.begin(); it!=_correspondenceValidators.end(); it++){
      CorrespondenceValidator* validator =*it;
      if (! validator || k>=validator->minimalSetSize())
	continue;
      if (!(*validator)(_correspondences, _indices, k))
	return false;
    }
    return true;
  }

  bool BaseGeneralizedRansac::_init() {
    assert (!_verticesPushed && "ATTEMPT_TO_INITIALIZE_THINGS_TWICE");
    if (!_verticesPushed && (int)_correspondences.size()>=minimalSetSize()) {
      std::fill(_indices.begin(), _indices.end(), 0);
      std::sort(_correspondences.begin(), _correspondences.end());
      // push all the vertices in the stack
      for (size_t i =0; i<_vertices2.size(); i++){
	OptimizableGraph::Vertex* v=_vertices2[i];
	assert(v && "VERTEX_CANNOT_BE_NULL");
	v->push();
      }
      _verticesPushed = true;
      return true;
    } else {
      _indices.clear();
      return false;
    }
  }

  bool BaseGeneralizedRansac::_cleanup() {
    assert (_verticesPushed && "ATTEMPT_CLEANUP_TWICE");
    if (_verticesPushed && (int)_correspondences.size()>=minimalSetSize()) {
      for (size_t i =0; i<_vertices2.size(); i++){
	OptimizableGraph::Vertex* v=_vertices2[i];
	assert(v && "VERTEX_CANNOT_BE_NULL");
	v->pop();
      }
      _verticesPushed = false;
      return true;
    } else {
      _indices.clear();
      return false;
    }
  }





}

