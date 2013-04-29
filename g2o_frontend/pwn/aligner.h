#include "homogeneouspoint3fscene.h"
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
    _referenceScene = 0;
    _currentScene = 0;
    _numCorrespondences = 0;
    _outerIterations = 10;
    _innerIterations = 1;
    _T = Eigen::Isometry3f::Identity();
    _initialGuess = Eigen::Isometry3f::Identity();
    _sensorOffset = Eigen::Isometry3f::Identity();
  };

  inline PointProjector* projector() const { return _projector; }
  inline Linearizer* linearizer() const { return _linearizer; }
  inline HomogeneousPoint3fScene* referenceScene() { return _referenceScene; }
  inline HomogeneousPoint3fScene* currentScene() { return _currentScene; }
  inline CorrespondenceVector& correspondences() { return _correspondences; }
  inline int numCorrespondences() { return _numCorrespondences; }
  inline int outerIterations() { return _outerIterations; }
  inline int innerIterations() { return _innerIterations; }
  inline Eigen::Isometry3f& T() { return _T; }
  inline Eigen::Isometry3f& initialGuess() { return _initialGuess; }
  inline Eigen::Isometry3f& sensorOffset() { return _sensorOffset; }

  inline void setProjector(PointProjector* projector_) { _projector = projector_; }  
  inline void setLinearizer(Linearizer* linearizer_) { _linearizer = linearizer_; }
  inline void setReferenceScene(HomogeneousPoint3fScene* referenceScene_) { _referenceScene = referenceScene_; }
  inline void setCurrentScene(HomogeneousPoint3fScene* currentScene_) { _currentScene = currentScene_; }
  inline void setCorrespondences(CorrespondenceVector& correspondences_) { _correspondences = correspondences_; }  
  inline void setNumCorrespondences(int numCorrespondences_) { _numCorrespondences = numCorrespondences_; }
  inline void setOuterIterations(int outerIterations_) { _outerIterations = outerIterations_; }
  inline void setInnerIterations(int innerIterations_) { _innerIterations = innerIterations_; }
  inline void setT(Eigen::Isometry3f T_) { _T = T_; }
  inline void setInitialGuess(Eigen::Isometry3f initialGuess_) { _initialGuess = initialGuess_; }
  inline void setSensorOffset(Eigen::Isometry3f sensorOffset_) { _sensorOffset = sensorOffset_; }

  void align();

 protected:
  PointProjector *_projector;
  Linearizer *_linearizer;

  HomogeneousPoint3fScene *_referenceScene;
  HomogeneousPoint3fScene *_currentScene;

  CorrespondenceGenerator _correspondenceGenerator;
  CorrespondenceVector _correspondences;
  int _numCorrespondences;
  
  int _outerIterations, _innerIterations;
 
  Eigen::Isometry3f _T;
  Eigen::Isometry3f _initialGuess;
  Eigen::Isometry3f _sensorOffset;
};
