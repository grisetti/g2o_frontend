#ifndef _PERFORMANCES_ANALYZER_H_
#define _PERFORMANCES_ANALYZER_H_

#include <fstream>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"

#include "g2o_frontend/basemath/bm_se3.h"

using namespace std;
using namespace Eigen;
using namespace g2o;

namespace pwn {
  
  class PerformancesAnalyzer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PerformancesAnalyzer();
    virtual ~PerformancesAnalyzer();

    bool openEvaluationFiles(string groundtruthFilename, string associationsFilename,
			     string absoluteTrajectoryFilename, string relativeTrajectoryFilename,
			     string benchmarkStatsFilename);
    void writePose();
    void writeBenchmarkStats();
    void parseGroundTruth(Isometry3f &initialTransformation, 
			  ifstream &isGroundtruth, double targetTimestamp);

    virtual void updateStructures();

    virtual void setSensorType(string sensorType_) {
      _sensorType = sensorType_;
      updateStructures();
    }
    virtual void setCameraMatrix(Matrix3f cameraMatrix_) {
      _cameraMatrix = cameraMatrix_;
      updateStructures();
    }
    virtual void setDepthImageSize(int imageRows_, int imageCols_) {
      _imageRows = imageRows_;
      _imageCols = imageCols_;
      updateStructures();
    }
    virtual void setScale(float scale_) {
      _scale = scale_;
      updateStructures();
    }
    inline void setMaxDistance(float maxDistance) { _projector->setMaxDistance(maxDistance); }
    inline void setSensorOffset(Isometry3f sensorOffset_) { _sensorOffset = sensorOffset_; }
    inline void setVerbose(int verbose_) { _verbose = verbose_; }

    inline int imageRows() { return _imageRows; }
    inline int imageCols() { return _imageCols; }
    inline int scaledImageRows() { return _scaledImageRows; }
    inline int scaledImageCols() { return _scaledImageCols; }
    inline float scale() { return _scale; }
    inline float maxDistance() { return _projector->maxDistance(); }
    inline double time() { return _oend - _ostart; }
    inline string sensorType() { return _sensorType; }
    inline string timestamp() { return _currentTimestamp; }
    inline Matrix3f cameraMatrix() { return _cameraMatrix; }
    inline Matrix3f scaledCameraMatrix() { return _scaledCameraMatrix; }
    inline Isometry3f sensorOffset() { return _sensorOffset; }
    inline Isometry3f absolutePose() { return _absolutePose; }
    inline Isometry3f relativePose() { return _relativePose; }
    inline int verbose() { return _verbose; }

  protected:
    // Dummy alignment structures for only chi2/#inliers evaluation purpose
    CorrespondenceFinder *_dummyCorrespondenceFinder;
    Linearizer *_dummyLinearizer;
#ifdef _PWN_USE_CUDA_
    CuAligner *_dummyAligner;
#else
    Aligner *_dummyAligner;
#endif

    // File streams
    ifstream _isGroundtruth, _isAssociations;
    ofstream _osAbsoluteTrajectory, _osRelativeTrajectory, _osBenchmark;

    // Alignment structures & objects
    int _imageRows, _imageCols, _scaledImageRows, _scaledImageCols;
    float _scale;
    double _ostart, _oend;
    string _sensorType, _currentTimestamp;
    Matrix3f _cameraMatrix, _scaledCameraMatrix;
    Isometry3f _absolutePose, _relativePose, _localPose, _sensorOffset;

    PinholePointProjector *_projector;

    int _verbose;
  };
}
#endif
