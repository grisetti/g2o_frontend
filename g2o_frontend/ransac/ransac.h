#ifndef _G2O_FRONTEND_RANSAC_H_
#define _G2O_FRONTEND_RANSAC_H_
#include <vector>
#include <algorithm>
#include <assert.h>
#include <Eigen/Geometry>
#include "g2o/core/optimizable_graph.h"


namespace g2o_frontend {

struct Correspondence {
  Correspondence(g2o::OptimizableGraph::Edge* edge_, double score_=0):  _edge(edge_), _score(score_){}
  inline g2o::OptimizableGraph::Edge* edge() {return _edge;}
  inline const g2o::OptimizableGraph::Edge* edge() const {return _edge;}
  inline double score() const {return _score;}
  inline bool operator<(const Correspondence& c2) const { return score()>c2.score(); }  // sorts the corresopondences is ascending order
protected:
  g2o::OptimizableGraph::Edge* _edge;
  double _score;
};

typedef std::vector<Correspondence> CorrespondenceVector;
typedef std::vector<int> IndexVector;

class CorrespondenceValidator{
 public:
  CorrespondenceValidator(int minimalSetSize_):_minimalSetSize(minimalSetSize_){}
  virtual bool operator()(const CorrespondenceVector& correspondences, const IndexVector& indices, int k) = 0;
  virtual ~CorrespondenceValidator();
  inline int minimalSetSize() const {return _minimalSetSize;}
 protected:
  int _minimalSetSize;
};

typedef std::vector<CorrespondenceValidator*> CorrespondenceValidatorPtrVector;


  template <typename TransformType_, typename PointVertexType_>
  class AlignmentAlgorithm{
  public:
    typedef TransformType_ TransformType;
    typedef PointVertexType_ PointVertexType;
    typedef typename PointVertexType_::EstimateType PointEstimateType;
    
    AlignmentAlgorithm(int minimalSetSize_) {_minimalSetSize = minimalSetSize_;}
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices) = 0; 
    inline int minimalSetSize() const  {return _minimalSetSize;}
  protected:
    int _minimalSetSize;
  };
  
  //typedef AlignmentAlgorithm<g2o::SE2,g2o::VertexLine2D>  AlignmentAlgorithmSE2Line2D;
  //typedef AlignmentAlgorithm<Eigen::Isometry3d,Slam3dAddons::VertexLine3D>   AlignmentAlgorithmSE3Line3D;

  
class BaseGeneralizedRansac{
public:
  BaseGeneralizedRansac(int _minimalSetSize);
  ~BaseGeneralizedRansac();
  
  void setCorrespondences(const CorrespondenceVector& correspondences_);
  inline const CorrespondenceVector& correspondences() const {return _correspondences;}
  bool validateCorrespondences(int k); 
  inline CorrespondenceValidatorPtrVector& correspondenceValidators() {return _correspondenceValidators;}
  inline const CorrespondenceValidatorPtrVector& correspondenceValidators() const {return _correspondenceValidators;}
  inline void setInlierErrorThreshold(double inlierErrorThreshold_) {_inlierErrorTheshold = inlierErrorThreshold_;}
  inline double inlierErrorThreshold() const {return _inlierErrorTheshold;}
  inline void setInlierStopFraction(double inlierStopFraction_) {_inlierStopFraction = inlierStopFraction_;}
  inline double inlierStopFraction() const {return _inlierStopFraction;}

  inline int minimalSetSize() const {return _minimalSetSize;}
protected:
  bool _init();
  bool _cleanup();

  int _minimalSetSize;
  int _maxIterations;
  IndexVector _indices;
  CorrespondenceVector _correspondences;
  CorrespondenceValidatorPtrVector _correspondenceValidators;
  double _inlierErrorTheshold;
  double _inlierStopFraction;
  std::vector<double> _errors;
  std::vector<g2o::OptimizableGraph::Vertex*> _vertices1;
  std::vector<g2o::OptimizableGraph::Vertex*> _vertices2;
  bool _verticesPushed;
};

template <typename AlignmentAlgorithmType>
class GeneralizedRansac : public BaseGeneralizedRansac{
public:
  typedef typename AlignmentAlgorithmType::TransformType TransformType;
  typedef typename AlignmentAlgorithmType::PointVertexType PointVertexType;
  typedef typename AlignmentAlgorithmType::PointVertexType::EstimateType PointEstimateType;
 
  GeneralizedRansac(int minimalSetSize_) :BaseGeneralizedRansac(minimalSetSize_){
    _alignmentAlgorithm = 0;
  }

  inline void setAlignmentAlgorithm(AlignmentAlgorithmType* alignmentAlgorithm_) {_alignmentAlgorithm = alignmentAlgorithm_;}

  inline AlignmentAlgorithmType* alignmentAlgorithm() {return _alignmentAlgorithm;}

  inline static int minimalSetSize() {return AlignmentAlgorithmType::minimalSetSize();}


  bool computeMinimalSet(TransformType& t, int k){
    assert(_alignmentAlgorithm && "YOU_MUST_SET_AN_ALIGNMENT_ALGORITHM");

    for (; _indices[k]<(int)_correspondences.size()-k; _indices[k]++){
      if (k==minimalSetSize()-1){
	if (validateCorrespondences(k))
	  if ((*_alignmentAlgorithm)(t,_correspondences,_indices))
	    return true;
      } else {
	_indices[k+1] = _indices[k] + 1;
	if (validateCorrespondences(k) && computeMinimalSet(t,k+1))
	  return true;
      }
    }
    return false;
  }
  
  bool operator()(TransformType& treturn){
    if (_correspondences.size()<minimalSetSize())
      return false;
    if (!_init())
      assert(0 && "_ERROR_IN_INITIALIZATION_");
    
    std::vector<int> bestInliers;
    bestInliers.reserve(_correspondences.size());
    double bestError=std::numeric_limits<double>::max();
    TransformType bestTransform = treturn;

    bool transformFound = false;
    for (int i=0; i<_maxIterations; i++){
      TransformType t;
      if (! computeMinimalSet(t,0))
	continue;
      
      std::vector<int> inliers;
      inliers.reserve(_correspondences.size());
      std::vector<int> currentErrors(_correspondences.size());
      int error = 0;
      
      // a transform has been found, now we need to apply it to all points to determine if they are inliers
      for(size_t k=0; k>_correspondences.size(); k++){
	Correspondence& c=_correspondences[k];
	g2o::OptimizableGraph::Edge* e=c.edge();
	PointVertexType* v2=static_cast<PointVertexType*>(e->vertex(1));
	PointEstimateType ebackup = v2->estimate();
	v2->setEstimate(t*v2->estimate());
	e->computeError();
	currentErrors[k] = e->chi2();
	if (e->chi2()<_inlierErrorTheshold){
	  inliers.push_back(k);
	  error+=e->chi2();
	}
	v2->setEstimate(ebackup);
      }
      if (inliers.size()<minimalSetSize())
	continue;
      if (inliers.size()>bestInliers.size()){
	double currentError = error/inliers.size();
	if (currentError<bestError){
	  bestError= currentError;
	  bestInliers = inliers;
	  _errors = currentErrors;
	  bestTransform = t;
	}
	if ((double)inliers.size()/(double)correspondences.size() > inlierStopFraction){
	  break;
	  transformFound = true;
	}
      }
    }
    if (transformFound){
      (*alignmentAlgorithm)(treturn,correspondences,bestInliers);
    }
    treturn = bestTransform;
    if (!_cleanup)
      assert(0 && "_ERROR_IN_CLEANUP_");
  }

protected:
  AlignmentAlgorithmType* _alignmentAlgorithm;
};

}

#endif
