#ifndef _PWN_ALINGER_H_
#define _PWN_ALINGER_H_
#include "g2o_frontend/boss_logger/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "linearizer.h"
#include "pointprojector.h"
#include "frame.h"
#include "correspondencefinder.h"
#include "se3_prior.h"

namespace pwn {

  class Aligner : public boss::Identifiable{
  public:
    Aligner(int id=-1, boss::IdContext* context=0);

    inline void setProjector(PointProjector *projector_) { _projector = projector_; }
    inline void setReferenceFrame(Frame *referenceFrame_) { _referenceFrame = referenceFrame_; clearPriors();}
    inline void setCurrentFrame(Frame*currentFrame_) { _currentFrame = currentFrame_; clearPriors();}
    inline void setOuterIterations(const int outerIterations_) { _outerIterations = outerIterations_; }
    inline void setInnerIterations(const int innerIterations_) { _innerIterations = innerIterations_; }
    inline void setT(const Eigen::Isometry3f T_) { _T = T_; _T.matrix().row(3) << 0,0,0,1; }
    inline void setInitialGuess(const Eigen::Isometry3f initialGuess_) { _initialGuess = initialGuess_; _initialGuess.matrix().row(3) << 0,0,0,1;}
    inline void setSensorOffset(const Eigen::Isometry3f sensorOffset_) { 
      setReferenceSensorOffset(sensorOffset_);
      setCurrentSensorOffset(sensorOffset_);
    }
    inline void setReferenceSensorOffset(const Eigen::Isometry3f referenceSensorOffset_) { _referenceSensorOffset = referenceSensorOffset_; _referenceSensorOffset.matrix().row(3) << 0,0,0,1;}
    inline void setCurrentSensorOffset(const Eigen::Isometry3f currentSensorOffset_) { _currentSensorOffset = currentSensorOffset_; _currentSensorOffset.matrix().row(3) << 0,0,0,1;}

    inline const PointProjector* projector() const { return _projector; }
    inline PointProjector* projector() { return _projector; }
    inline Linearizer* linearizer() { return _linearizer; }
    inline void setLinearizer(Linearizer* linearizer_) { _linearizer = linearizer_; 
      if( _linearizer) 
	_linearizer->setAligner(this); 
    }
    inline void setDebug(const bool debug_) { _debug = debug_; }
    inline void setMinInliers(const int minInliers_) { _minInliers = minInliers_; }
    inline void setTranslationalMinEigenRatio(const float translationalMinEigenRatio_) { _translationalMinEigenRatio = translationalMinEigenRatio_; }
    inline void setRotationalMinEigenRatio(const float rotationalMinEigenRatio_) { _rotationalMinEigenRatio = rotationalMinEigenRatio_; }
    

    inline CorrespondenceFinder* correspondenceFinder() { return _correspondenceFinder; }
    inline void setCorrespondenceFinder(CorrespondenceFinder* correspondenceFinder_) { _correspondenceFinder = correspondenceFinder_; }
    inline const Frame* referenceFrame() const { return _referenceFrame; }
    inline const Frame* currentFrame() const { return _currentFrame; }
    inline int outerIterations() const { return _outerIterations; }
    inline int innerIterations() const { return _innerIterations; }
    inline const Eigen::Isometry3f& T() const { return _T; }
    inline const Eigen::Isometry3f& initialGuess() const { return _initialGuess; }
    inline const Eigen::Isometry3f& sensorOffset() const { return _referenceSensorOffset; }
    inline const Eigen::Isometry3f& referenceSensorOffset() const { return _referenceSensorOffset; }
    inline const Eigen::Isometry3f& currentSensorOffset() const { return _currentSensorOffset; }
    inline bool debug() const { return _debug; }
    inline int minInliers() const { return _minInliers; }

    virtual void align();
    const Matrix6f& omega() const {return _omega;}
    inline float error() const {return _error;}
    inline int inliers() const {return _inliers; }
    inline double totalTime() const {return _totalTime; }
  
    void addRelativePrior(const Eigen::Isometry3f& mean, const Matrix6f& informationMatrix);
    void addAbsolutePrior(const Eigen::Isometry3f& referenceTransform, const Eigen::Isometry3f& mean, const Matrix6f& informationMatrix);
    void clearPriors();

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserializeComplete();
    
  protected:
    void _computeStatistics(Vector6f& mean, Matrix6f& Omega, 
			   float& translationalRatio, float& rotationalRatio) const;

    PointProjector *_projector;
    Linearizer *_linearizer;
    CorrespondenceFinder *_correspondenceFinder;

    Frame *_referenceFrame;
    Frame *_currentFrame;
  
    bool _debug;
    int _outerIterations, _innerIterations, _minInliers;

    Eigen::Isometry3f _T;
    Eigen::Isometry3f _initialGuess;
    // Eigen::Isometry3f _sensorOffset;
    Eigen::Isometry3f _referenceSensorOffset;
    Eigen::Isometry3f _currentSensorOffset;
    
    Matrix6f _omega;
    Vector6f _mean;

    int _inliers;
    double _totalTime;
    float _error;
    float  _translationalEigenRatio, _rotationalEigenRatio, _translationalMinEigenRatio, _rotationalMinEigenRatio;

    std::vector<SE3Prior*> _priors;
  };

}

#endif
