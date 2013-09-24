#ifndef _PWN_PERFORMANCES_ANALYZER_H_
#define _PWN_PERFORMANCES_ANALYZER_H_

#include "performances_analyzer.h"

#include "g2o_frontend/pwn2/merger.h"

using namespace std;
using namespace Eigen;
using namespace g2o;

namespace pwn {
  class PWNPerformancesAnalyzer : public PerformancesAnalyzer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    PWNPerformancesAnalyzer();
    virtual ~PWNPerformancesAnalyzer();

    bool loadNextPair();
    void computeAlignment();

    virtual void setSensorType(string sensorType_) {
      _sensorType = sensorType_;
      updateStructures();
    }
    virtual void setCameraMatrix(Matrix3f cameraMatrix_) {
      _cameraMatrix = cameraMatrix_;
      updateStructures();
    }
    virtual void setDepthImageSize(int imageRows_, int imageCols_) {
      _imageRows = imageCols_;
      _imageCols = imageRows_;
      updateStructures();
      _correspondenceFinder->setImageSize(_scaledImageRows, _scaledImageCols);
      _projector->setImageSize(_scaledImageRows, _scaledImageCols);
      _merger->setImageSize(_scaledImageRows, _scaledImageCols);
    }
    virtual void setScale(float scale_) {
      _scale = scale_;
      updateStructures();
      _correspondenceFinder->setImageSize(_scaledImageRows, _scaledImageCols);
      _projector->setImageSize(_scaledImageRows, _scaledImageCols);
      _merger->setImageSize(_scaledImageRows, _scaledImageCols);
    }

    virtual inline void setMaxDistance(float maxDistance) { 
      _projector->setMaxDistance(maxDistance); 
      _merger->setMaxPointDepth(maxDistance);
    }
    inline void setScaleFactor(float scaleFactor_) { _scaleFactor = scaleFactor_; }
    inline void setWorldRadius(float worldRadius) { _statsCalculator->setWorldRadius(worldRadius); }
    inline void setMinImageRadius(int minImageRadius) { _statsCalculator->setMinImageRadius(minImageRadius); }
    inline void setMaxImageRadius(int maxImageRadius) { _statsCalculator->setMaxImageRadius(maxImageRadius); }
    inline void setMinPoints(int minPoints) { _statsCalculator->setMinPoints(minPoints); }
    inline void setCurvatureThreshold(float curvatureThreshold) { 
      _statsCalculator->setCurvatureThreshold(curvatureThreshold);
      _pointInformationMatrixCalculator->setCurvatureThreshold(curvatureThreshold);
      _normalInformationMatrixCalculator->setCurvatureThreshold(curvatureThreshold);
    }
    inline void setInlierDistanceThreshold(float inlierDistanceThreshold) { 
      _correspondenceFinder->setInlierDistanceThreshold(inlierDistanceThreshold);
      _merger->setDistanceThreshold(inlierDistanceThreshold);
    }
    inline void setFlatCurvatureThreshold(float flatCurvatureThreshold) { _correspondenceFinder->setFlatCurvatureThreshold(flatCurvatureThreshold); }
    inline void setInlierCurvatureRatioThreshold(float inlierCurvatureRatioThreshold) { _correspondenceFinder->setInlierCurvatureRatioThreshold(inlierCurvatureRatioThreshold); }
    inline void setInlierNormalAngularThreshold(float inlierNormalAngularThreshold) { 
      _correspondenceFinder->setInlierNormalAngularThreshold(cosf(inlierNormalAngularThreshold)); 
      _merger->setNormalThreshold(cosf(inlierNormalAngularThreshold));
    }
    inline void setInlierMaxChi2(float inlierMaxChi2) { _linearizer->setInlierMaxChi2(inlierMaxChi2); }
    inline void setInnerIterations(int innerIterations) { _aligner->setInnerIterations(innerIterations); }    
    inline void setOuterIterations(int outerIterations) { _aligner->setOuterIterations(outerIterations); }
    inline void setChunkStep(int chunkStep_) { _chunkStep = chunkStep_; }

    inline float scaleFactor() { return _scaleFactor; }
    inline float worldRadius() { return _statsCalculator->worldRadius(); }
    inline int minImageRadius() { return _statsCalculator->minImageRadius(); }
    inline int maxImageRadius() { return _statsCalculator->maxImageRadius(); }
    inline int minPoints() { return _statsCalculator->minPoints(); }
    inline float curvatureThreshold() { return _statsCalculator->curvatureThreshold(); } 
    inline float inlierDistanceThreshold() { return _correspondenceFinder->inlierDistanceThreshold(); }
    inline float flatCurvatureThreshold() { return _correspondenceFinder->flatCurvatureThreshold(); }
    inline float inlierCurvatureRatioThreshold() { return _correspondenceFinder->inlierCurvatureRatioThreshold(); }
    inline float inlierNormalAngularThreshold() { return acosf(_correspondenceFinder->inlierNormalAngularThreshold()); }
    inline float inlierMaxChi2() { return _linearizer->inlierMaxChi2(); }
    inline int innerIterations() { return _aligner->innerIterations(); }    
    inline int outerIterations() { return _aligner->outerIterations(); }
    inline int chunkStep() { return _chunkStep; }
    inline const DepthImage &referenceDepth() { return _referenceDepth; }
    inline const DepthImage &currentDepth() { return _currentDepth; }
    inline const DepthImage &referenceScaledDepth() { return _referenceScaledDepth; }
    inline const DepthImage &currentScaledDepth() { return _currentScaledDepth; }
    inline const MatrixXi &indexImage() { return _indexImage; }
    inline const MatrixXi &scaledIndexImage() { return _scaledIndexImage; }
    inline const Frame &referenceFrame() { return _referenceFrame; }
    inline const Frame &currentFrame() { return _currentFrame; }

  protected:
    // Alignment structures
    int _chunkStep, _counter;
    float _scaleFactor;
    string _referenceDepthFilename, _currentDepthFilename;
    DepthImage _referenceDepth, _currentDepth, _referenceScaledDepth, _currentScaledDepth;
    MatrixXi _indexImage, _scaledIndexImage;
    Isometry3f _localPose, _chunkInitialPose;
    Frame _referenceFrame, _currentFrame, _scene, _subScene; 
    StatsCalculator *_statsCalculator;
    PointInformationMatrixCalculator *_pointInformationMatrixCalculator;
    NormalInformationMatrixCalculator *_normalInformationMatrixCalculator;
    DepthImageConverter *_converter;    
    CorrespondenceFinder *_correspondenceFinder;
    Linearizer *_linearizer;
#ifdef _PWN_USE_CUDA_
    CuAligner *_aligner;
#else
    Aligner *_aligner;
#endif
    Merger *_merger;
  };
}

#endif
