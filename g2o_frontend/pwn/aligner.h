#include "linearizer.h"
#include "pointprojector.h"
#include "homogeneouspoint3fscene.h"
#include "correspondencegenerator.h"

class Aligner {
 public:
  Aligner() {
    _linearizer.setAligner(this);
    _outerIterations = 0;
    _innerIterations = 0;
    _T = Eigen::Isometry3f::Identity();
    _initialGuess = Eigen::Isometry3f::Identity();
    _sensorOffset = Eigen::Isometry3f::Identity();
  };

  inline void setProjector(PointProjector *projector_) { _projector = projector_; }
  inline void setLinearizer(Linearizer &linearizer_) { _linearizer = linearizer_; }
  inline void setCorrespondenceGenerator(CorrespondenceGenerator &correspondenceGenerator_) { _correspondenceGenerator = correspondenceGenerator_; } 
  inline void setReferenceScene(const HomogeneousPoint3fScene &referenceScene_) { _referenceScene = referenceScene_; }
  inline void setCurrentScene(const HomogeneousPoint3fScene &currentScene_) { _currentScene = currentScene_; }
  inline void setOuterIterations(int outerIterations_) { _outerIterations = outerIterations_; }
  inline void setInnerIterations(int innerIterations_) { _innerIterations = innerIterations_; }
  inline void setT(Eigen::Isometry3f T_) { _T = T_; }
  inline void setInitialGuess(Eigen::Isometry3f initialGuess_) { _initialGuess = initialGuess_; }
  inline void setSensorOffset(Eigen::Isometry3f sensorOffset_) { _sensorOffset = sensorOffset_; }

  inline PointProjector* projector() { return _projector; }
  inline Linearizer& linearizer() { return _linearizer; }
  inline CorrespondenceGenerator& correspondenceGenerator() { return _correspondenceGenerator; } 
  inline HomogeneousPoint3fScene& referenceScene() { return _referenceScene; }
  inline HomogeneousPoint3fScene& currentScene() { return _currentScene; }  
  inline int outerIterations() { return _outerIterations; }
  inline int innerIterations() { return _innerIterations; }
  inline Eigen::Isometry3f T() { return _T; }
  inline Eigen::Isometry3f initialGuess() { return _initialGuess; }
  inline Eigen::Isometry3f sensorOffset() { return _sensorOffset; }

  void align();

 protected:
  PointProjector *_projector;
  Linearizer _linearizer;

  CorrespondenceGenerator _correspondenceGenerator;

  HomogeneousPoint3fScene _referenceScene;
  HomogeneousPoint3fScene _currentScene;
  
  int _outerIterations, _innerIterations;

  Eigen::Isometry3f _T;
  Eigen::Isometry3f _initialGuess;
  Eigen::Isometry3f _sensorOffset;
};
