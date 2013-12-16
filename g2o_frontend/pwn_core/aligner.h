#pragma once

#include "linearizer.h"
#include "pointprojector.h"
#include "cloud.h"
#include "correspondencefinder.h"
#include "se3_prior.h"

namespace pwn {

  class Aligner {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Aligner();
    virtual ~Aligner() {}

    inline PointProjector* projector() { return _projector; }
    inline void setProjector(PointProjector *projector_) { _projector = projector_; }
    
    inline const Cloud* referenceCloud() const { return _referenceCloud; }
    inline void setReferenceCloud(Cloud *referenceCloud_) { 
      _referenceCloud = referenceCloud_; 
      clearPriors();
    }
    
    inline const Cloud* currentCloud() const { return _currentCloud; }
    inline void setCurrentCloud(Cloud *currentCloud_) { 
      _currentCloud = currentCloud_; 
      clearPriors();
    }

    inline int outerIterations() const { return _outerIterations; }
    inline void setOuterIterations(const int outerIterations_) { _outerIterations = outerIterations_; }

    inline int innerIterations() const { return _innerIterations; }
    inline void setInnerIterations(const int innerIterations_) { _innerIterations = innerIterations_; }

    inline const Eigen::Isometry3f& T() const { return _T; }
    inline void setT(const Eigen::Isometry3f T_) { 
      _T = T_; 
      _T.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f; 
    }

    inline const Eigen::Isometry3f& initialGuess() const { return _initialGuess; }	
    inline void setInitialGuess(const Eigen::Isometry3f initialGuess_) { 
      _initialGuess = initialGuess_; 
      _initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f; 
    }

    inline const Eigen::Isometry3f& sensorOffset() const { return _referenceSensorOffset; }
    inline void setSensorOffset(const Eigen::Isometry3f sensorOffset_) { 
      setReferenceSensorOffset(sensorOffset_);
      setCurrentSensorOffset(sensorOffset_);
    }

    inline const Eigen::Isometry3f& referenceSensorOffset() const { return _referenceSensorOffset; }
    inline void setReferenceSensorOffset(const Eigen::Isometry3f referenceSensorOffset_) { 
      _referenceSensorOffset = referenceSensorOffset_; 
      _referenceSensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    }

    inline const Eigen::Isometry3f& currentSensorOffset() const { return _currentSensorOffset; }
    inline void setCurrentSensorOffset(const Eigen::Isometry3f currentSensorOffset_) { 
      _currentSensorOffset = currentSensorOffset_; 
      _currentSensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    }

    inline Linearizer* linearizer() { return _linearizer; }
    inline void setLinearizer(Linearizer* linearizer_) { 
      _linearizer = linearizer_; 
      if( _linearizer) 
	_linearizer->setAligner(this); 
    }

    inline bool debug() const { return _debug; }
    inline void setDebug(const bool debug_) { _debug = debug_; }

    inline int minInliers() const { return _minInliers; }
    inline void setMinInliers(const int minInliers_) { _minInliers = minInliers_; }

    inline float translationalMinEigenRatio() { return _translationalMinEigenRatio; }
    inline void setTranslationalMinEigenRatio(const float translationalMinEigenRatio_) { _translationalMinEigenRatio = translationalMinEigenRatio_; }
    
    inline float rotationalMinEigenRatio() { return _rotationalMinEigenRatio; }
    inline void setRotationalMinEigenRatio(const float rotationalMinEigenRatio_) { _rotationalMinEigenRatio = rotationalMinEigenRatio_; }
    
    inline CorrespondenceFinder* correspondenceFinder() { return _correspondenceFinder; }
    inline void setCorrespondenceFinder(CorrespondenceFinder* correspondenceFinder_) { _correspondenceFinder = correspondenceFinder_; }

    virtual void align();
    const Matrix6f& omega() const { return _omega; }
    inline float error() const { return _error; }
    inline int inliers() const { return _inliers; }
    inline double totalTime() const { return _totalTime; }
  
    void addRelativePrior(const Eigen::Isometry3f &mean, const Matrix6f &informationMatrix);
    void addAbsolutePrior(const Eigen::Isometry3f &referenceTransform, const Eigen::Isometry3f &mean, const Matrix6f &informationMatrix);
    void clearPriors();

  protected:
    void _computeStatistics(Vector6f &mean, Matrix6f &Omega, 
			    float &translationalRatio, float &rotationalRatio) const;

    PointProjector *_projector;
    Linearizer *_linearizer;
    CorrespondenceFinder *_correspondenceFinder;

    Cloud *_referenceCloud;
    Cloud *_currentCloud;
  
    bool _debug;
    int _outerIterations, _innerIterations, _minInliers;

    Eigen::Isometry3f _T;
    Eigen::Isometry3f _initialGuess;
    Eigen::Isometry3f _referenceSensorOffset;
    Eigen::Isometry3f _currentSensorOffset;
    
    Matrix6f _omega;
    Vector6f _mean;

    int _inliers;
    double _totalTime;
    float _error;
    float  _translationalEigenRatio, _rotationalEigenRatio, _translationalMinEigenRatio, _rotationalMinEigenRatio;

    std::string _debugPrefix;
    std::vector<SE3Prior*> _priors;
  };

}
