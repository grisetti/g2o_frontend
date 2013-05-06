#ifndef _ALIGNER_H_
#define _ALIGNER_H_

#include "linearizer.h"
#include "pointprojector.h"
#include "homogeneouspoint3fstats.h"
#include "homogeneouspoint3fomega.h"
#include "correspondencegenerator.h"

class Aligner {
 public:
  Aligner() {
    _projector = 0;
    _linearizer = new Linearizer();
    _linearizer->setAligner(this);
    _referencePoints = 0;
    _currentPoints = 0;
    _referenceNormals = 0;
    _currentNormals = 0;
    _referenceStats = 0;
    _currentStats = 0;
    _currentPointOmegas = 0;
    _currentNormalOmegas = 0;
    _numCorrespondences = 0;
    _outerIterations = 0;
    _innerIterations = 0;
    _rows = 0;
    _cols = 0;
    _T = Eigen::Isometry3f::Identity();
    _initialGuess = Eigen::Isometry3f::Identity();
    _sensorOffset = Eigen::Isometry3f::Identity();
  };

  inline PointProjector* projector() const { return _projector; }
  inline void setProjector(PointProjector* projector_) { _projector = projector_; }
  inline Linearizer* linearizer() const { return _linearizer; }
  inline void setLinearizer(Linearizer* linearizer_) { _linearizer = linearizer_; }
  inline HomogeneousPoint3fVector* referencePoints() const { return _referencePoints; }
  inline HomogeneousPoint3fVector* currentPoints() const { return _currentPoints; }

  inline void setPoints(HomogeneousPoint3fVector* referencePoints_, HomogeneousPoint3fVector* currentPoints_) {
    _referencePoints = referencePoints_;
    _currentPoints = currentPoints_;
  }

  inline HomogeneousNormal3fVector* referenceNormals() const { return _referenceNormals; }
  inline HomogeneousNormal3fVector* currentNormals() const { return _currentNormals; }

  inline void setNormals(HomogeneousNormal3fVector* referenceNormals_, HomogeneousNormal3fVector* currentNormals_) {
    _referenceNormals = referenceNormals_;
    _currentNormals = currentNormals_;
  }
  
  inline HomogeneousPoint3fStatsVector* getReferenceStats() { return _referenceStats; }
  inline HomogeneousPoint3fStatsVector* getCurrentStats() { return _currentStats; }
  inline void setStats(HomogeneousPoint3fStatsVector* referenceStats_, HomogeneousPoint3fStatsVector* currentStats_) {
    _referenceStats = referenceStats_;
    _currentStats = currentStats_;
  }
  
  inline HomogeneousPoint3fOmegaVector* currentPointOmegas() { return _currentPointOmegas; }
  inline HomogeneousPoint3fOmegaVector* currentNormalOmegas() { return _currentNormalOmegas; }
  inline void setCurrentOmegas(HomogeneousPoint3fOmegaVector* currentPointOmegas_, HomogeneousPoint3fOmegaVector* currentNormalOmegas_) {
    _currentPointOmegas = currentPointOmegas_;
    _currentNormalOmegas = currentNormalOmegas_;
  }

  inline CorrespondenceVector& correspondences() { return _correspondences; }
  inline void setCorrespondences(CorrespondenceVector& correspondences_) { _correspondences = correspondences_; }
  
  inline int numCorrespondences() { return _numCorrespondences; }
  inline void setNumCorrespondences(int numCorrespondences_) { _numCorrespondences = numCorrespondences_; }
  inline int outerIterations() { return _outerIterations; }
  inline void setOuterIterations(int outerIterations_) { _outerIterations = outerIterations_; }
  inline int innerIterations() { return _innerIterations; }
  inline void setInnerIterations(int innerIterations_) { _innerIterations = innerIterations_; }
  
  inline int rows() { return _rows; }
  inline int cols() { return _cols; }
  inline void setImageSize(int rows_, int cols_) { 
    _rows = rows_; 
    _cols = cols_;
    _referenceIndexImage.resize(_rows, _cols);
    _currentIndexImage.resize(_rows, _cols); 
  }



  inline Eigen::Isometry3f& T() { return _T; }
  inline void setT(Eigen::Isometry3f T_) { _T = T_; }
  inline Eigen::Isometry3f& initialGuess() { return _initialGuess; }
  inline void setInitialGuess(Eigen::Isometry3f initialGuess_) { _initialGuess = initialGuess_; }
  inline Eigen::Isometry3f& sensorOffset() { return _sensorOffset; }
  inline void setSensorOffset(Eigen::Isometry3f sensorOffset_) { _sensorOffset = sensorOffset_; }

  void align();

 protected:
  PointProjector *_projector;
  Linearizer *_linearizer;

  CorrespondenceGenerator _correspondenceGenerator;
  
  HomogeneousPoint3fVector *_referencePoints, *_currentPoints;
  HomogeneousNormal3fVector *_referenceNormals,* _currentNormals;
  HomogeneousPoint3fStatsVector *_referenceStats, *_currentStats;
  HomogeneousPoint3fOmegaVector *_currentPointOmegas;
  HomogeneousPoint3fOmegaVector *_currentNormalOmegas;
  
  int _numCorrespondences;
  int _outerIterations, _innerIterations;
  int _rows, _cols;

  CorrespondenceVector _correspondences;
  MatrixXi _referenceIndexImage, _currentIndexImage;
  DepthImage _referenceDepthImage, _currentDepthImage;

  Eigen::Isometry3f _T;
  Eigen::Isometry3f _initialGuess;
  Eigen::Isometry3f _sensorOffset;
};

#endif
