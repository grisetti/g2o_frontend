#ifndef _G2O_FRONTEND_RANSAC_H_
#define _G2O_FRONTEND_RANSAC_H_
#include <vector>
#include <algorithm>
#include <assert.h>
#include <Eigen/Geometry>
#include "g2o/core/optimizable_graph.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"


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
  
  typedef AlignmentAlgorithm<g2o::SE2,g2o::VertexPointXY> AlignmentAlgorithmSE2PointXY;
  typedef AlignmentAlgorithm<g2o::SE2,g2o::VertexLine2D>  AlignmentAlgorithmSE2Line2D;
  typedef AlignmentAlgorithm<g2o::SE2,g2o::VertexSE2>     AlignmentAlgorithmSE2SE2;

  typedef AlignmentAlgorithm<Eigen::Isometry3d,g2o::VertexPointXYZ> AlignmentAlgorithmSE3PointXYZ;
  typedef AlignmentAlgorithm<Eigen::Isometry3d,g2o::VertexSE3>      AlignmentAlgorithmSE3SE3;
  //typedef AlignmentAlgorithm<Eigen::Isometry3d,g2o::VertexLine3D>   AlignmentAlgorithmSE3Line3D;

  
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
  std::vector<double> _errors;
  std::vector<g2o::OptimizableGraph::Vertex*> _vertices1;
  std::vector<g2o::OptimizableGraph::Vertex*> _vertices2;
  bool _verticesPushed;
};

template <typename AlignmentAlgorithmType, int _minimalSetSize>
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
    if (k==minimalSetSize()){
      if (validateCorrespondences(k))
	if ((*_alignmentAlgorithm)(t,_correspondences,_indices))
	  return true;
    }
    for (; _indices[k]<(int)_correspondences.size()-k; _indices[k]++){
      _indices[k+1] = _indices[k] + 1;
      if (validateCorrespondences(k) && computeMinimalSet(t,k+1))
	return true;
    }
    return false;
  }
  
protected:
  AlignmentAlgorithmType* _alignmentAlgorithm;
};

}

#endif
