#include "linearizer.h"
#include "pointprojector.h"
#include "homogeneouspoint3fscene.h"
#include "correspondencegenerator.h"

class Aligner {
 public:
  Aligner() {
    _linearizer.setAligner(this);    
    _referenceScene = 0;
    _currentScene = 0;
    _outerIterations = 0;
    _innerIterations = 0;
    _T = Eigen::Isometry3f::Identity();
    _initialGuess = Eigen::Isometry3f::Identity();
    _sensorOffset = Eigen::Isometry3f::Identity();
  };

  inline void setProjector(PointProjector *projector_) { _projector = projector_; }
  inline void setReferenceScene(HomogeneousPoint3fScene *referenceScene_) { _referenceScene = referenceScene_; }
  inline void setCurrentScene(HomogeneousPoint3fScene *currentScene_) { _currentScene = currentScene_; }
  inline void setOuterIterations(const int outerIterations_) { _outerIterations = outerIterations_; }
  inline void setInnerIterations(const int innerIterations_) { _innerIterations = innerIterations_; }
  inline void setT(const Eigen::Isometry3f T_) { _T = T_; }
  inline void setInitialGuess(const Eigen::Isometry3f initialGuess_) { _initialGuess = initialGuess_; }
  inline void setSensorOffset(const Eigen::Isometry3f sensorOffset_) { _sensorOffset = sensorOffset_; }

  inline const PointProjector* projector() const { return _projector; }
  inline Linearizer& linearizer() { return _linearizer; }
  inline CorrespondenceGenerator& correspondenceGenerator() { return _correspondenceGenerator; } 
  inline const HomogeneousPoint3fScene* referenceScene() const { return _referenceScene; }
  inline const HomogeneousPoint3fScene* currentScene() const { return _currentScene; }  
  inline int outerIterations() const { return _outerIterations; }
  inline int innerIterations() const { return _innerIterations; }
  inline Eigen::Isometry3f T() const { return _T; }
  inline Eigen::Isometry3f initialGuess() const { return _initialGuess; }
  inline Eigen::Isometry3f sensorOffset() const { return _sensorOffset; }

  void align();

 protected:
  PointProjector *_projector;
  Linearizer _linearizer;

  CorrespondenceGenerator _correspondenceGenerator;

  HomogeneousPoint3fScene *_referenceScene;
  HomogeneousPoint3fScene *_currentScene;
  
  int _outerIterations, _innerIterations;

  Eigen::Isometry3f _T;
  Eigen::Isometry3f _initialGuess;
  Eigen::Isometry3f _sensorOffset;
};
