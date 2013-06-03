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
      for (size_t i=0; i<_indices.size(); i++)
	_indices[i] = i;
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

void BaseGeneralizedRansac::keepBestFriend(std::vector<int>& bestFriendInliers_, const std::vector<double> _errors, std::vector<int>& inliers)
{
    viCorrMap ViCorrMap;
    viCorrMap::iterator it;
    std::pair<viCorrMap::iterator, bool> ret_insert;

    cout << endl << ">>> start creating the map(v1.id(),vector<ciErr>) - original size of inliers: " << inliers.size() << endl;
    for (size_t i = 0; i<inliers.size(); i++){

        int idx = inliers[i];
        Correspondence& c =_correspondences[idx];
        double err = _errors[idx];
        g2o::OptimizableGraph::Edge* e = c.edge();
        g2o::OptimizableGraph::Vertex* v1=static_cast<g2o::OptimizableGraph::Vertex*>(e->vertex(0));
        g2o::OptimizableGraph::Vertex* v2=static_cast<g2o::OptimizableGraph::Vertex*>(e->vertex(1));
//         cerr << "Current inlier is: " << "(" << idx << ","<< e << ","<< v1->id() << "," << v2->id() << "," << err << "), ";

        ciError CiErr;
        CiErr.idx = idx;
        CiErr.err = err;

        it = ViCorrMap.find(v1->id());
        if(it == ViCorrMap.end()){ //the vertex doesn't exists

            ciErrVector CiErrVector;
            CiErrVector.push_back(CiErr);
            ret_insert = ViCorrMap.insert(std::pair<int,ciErrVector>(v1->id(),CiErrVector));
//             cerr << "size of ViCorrMap: " << ViCorrMap.size() << ", inserito?? " << ret_insert.second  << endl;

        } else { //the vertex already exists'

            it->second.push_back(CiErr);
//             cerr << "size of ciErrVector: " << it->second.size() << endl;
        }
    }
    cerr << endl;

    cerr << " >>> start finding the best friends." << endl;
    for(viCorrMap::iterator it = ViCorrMap.begin(); it != ViCorrMap.end(); it++) {

        int idv1 = it->first;
        ciErrVector errVector = it->second;

        if(errVector.size() == 1){
            ciError ci = errVector[0];
            bestFriendInliers_.push_back(ci.idx);
            cerr << "For vertex id: " << idv1 << "-->inlier correspondance with min error: " << ci.err << " and inlier index: " << ci.idx << endl;
        } else {
            double err_min = 1e9;
            int index_inliers = -1;
            for(int i = 0; i < (int)errVector.size(); i++) {

                ciError ci = errVector[i];
                if(ci.err < err_min) {
                    err_min = ci.err;
                    index_inliers = ci.idx;
                }
            }
            cerr << "For vertex id: " << idv1 << "-->inlier correspondance with min error: " << err_min << " and inlier index: " << index_inliers << endl;
            bestFriendInliers_.push_back(index_inliers);
        }
    }
    inliers = bestFriendInliers_;
    //debug
//    cerr << "best friend inliers size: " << bestFriendInliers_.size() << ", must be equal to " << inliers.size() << endl;
    cerr << "bestFriendInliers_ are: ";
    for (size_t i = 0; i<bestFriendInliers_.size(); i++){
      int idx = bestFriendInliers_[i];
      Correspondence& c=_correspondences[idx];
      double err = _errors[idx];
      g2o::OptimizableGraph::Edge* e=c.edge();
      g2o::OptimizableGraph::Vertex* v1=static_cast<g2o::OptimizableGraph::Vertex*>(e->vertex(0));
      g2o::OptimizableGraph::Vertex* v2=static_cast<g2o::OptimizableGraph::Vertex*>(e->vertex(1));
      cerr << "(" << idx << ","<< e << ","<< v1->id() << "," << v2->id() << "," << err << "), ";
    }
    cerr << endl;

}






}

