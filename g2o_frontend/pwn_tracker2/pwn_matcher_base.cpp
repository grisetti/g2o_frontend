#include "pwn_matcher_base.h"
#include "g2o_frontend/pwn_core/pwn_static.h"
#include "g2o/stuff/timeutil.h"

namespace pwn_tracker {
  using namespace std;


  PwnMatcherBase::PwnMatcherBase(pwn::Aligner* aligner_, pwn::DepthImageConverter* converter_,
				 int id, boss::IdContext* context):
    Identifiable(id, context){
    _aligner = aligner_;
    _converter = converter_;
    _scale = 2;
    _frameInlierDepthThreshold = 50;
  }

  void PwnMatcherBase::serialize(boss::ObjectData& data, boss::IdContext& context){
    boss::Identifiable::serialize(data,context);
    _baligner=dynamic_cast<pwn_boss::Aligner*>(_aligner);
    _bconverter=dynamic_cast<pwn_boss::DepthImageConverter*>(_converter);
    data.setPointer("aligner", _baligner);
    data.setPointer("converter", _bconverter);
    data.setInt("scale", _scale);
    data.setFloat("frameInlierDepthThreshold", _frameInlierDepthThreshold);
  }
    
  
  void PwnMatcherBase::deserialize(boss::ObjectData& data, boss::IdContext& context){
    boss::Identifiable::deserialize(data,context);
    data.getReference("aligner").bind(_baligner);
    data.getReference("converter").bind(_bconverter);
    int s = data.getInt("scale");
    _frameInlierDepthThreshold = data.getFloat("frameInlierDepthThreshold");
    setScale(s);
  }


  void PwnMatcherBase::deserializeComplete(){
    boss::Identifiable::deserializeComplete();
    _aligner = _baligner;
    _converter = _bconverter;
  }


  pwn::Cloud* PwnMatcherBase::makeCloud(int& r, int& c, Eigen::Matrix3f& cameraMatrix,
				    const Eigen::Isometry3f& sensorOffset,  const DepthImage& depthImage) {


    PinholePointProjector* projector=dynamic_cast<PinholePointProjector*>(_converter->projector());
    float invScale = 1.0f / _scale;
    Matrix3f scaledCameraMatrix = cameraMatrix * invScale;
    scaledCameraMatrix(2, 2) = 1.0f;

    projector->setCameraMatrix(scaledCameraMatrix);
    int scaledImageRows = depthImage.rows / _scale;
    int scaledImageCols = depthImage.cols / _scale;
    projector->setImageSize(scaledImageRows, scaledImageCols);

    DepthImage scaledImage;
    DepthImage_scale(scaledImage,depthImage,_scale);

    cameraMatrix = projector->cameraMatrix();
    r = projector->imageRows();
    c = projector->imageCols();
    pwn::Cloud* cloud = new pwn::Cloud;
    double t0 = g2o::get_time();
    _converter->compute(*cloud, scaledImage, sensorOffset);
    double t1 = g2o::get_time();
    numCalls ++;
    cumTime += (t1-t0);
  // cloud->save("temp.pwn", 1, true);
    // scale image and camera matrix
    return cloud;
  }

  void PwnMatcherBase::matchClouds(PwnMatcherBase::MatcherResult& result, 
				   pwn::Cloud* fromCloud, pwn::Cloud* toCloud,
				   const Eigen::Isometry3f& fromOffset, const Eigen::Isometry3f& toOffset, 
				   const Eigen::Matrix3f& toCameraMatrix,
				   int toRows, int toCols,
				   const Eigen::Isometry3d& initialGuess){
    
    
    /*
    Eigen::Isometry3f fromOffset, toOffset;
    Eigen::Matrix3f fromCameraMatrix, toCameraMatrix;

    convertScalar(fromOffset, fromOffset_);
    convertScalar(fromCameraMatrix, fromCameraMatrix_);
    convertScalar(toOffset, toOffset_);
    convertScalar(toCameraMatrix, toCameraMatrix_);
    */
    

    PinholePointProjector* projector = dynamic_cast<PinholePointProjector*>(_aligner->projector());
    int r, c;
    
    _aligner->setReferenceSensorOffset(fromOffset);
    _aligner->setCurrentSensorOffset(toOffset);
    Eigen::Isometry3f ig;
    convertScalar(ig, initialGuess);
    ig.translation().z() = 0;
    ig.matrix().row(3) << 0,0,0,1;
    _aligner->setInitialGuess(ig);
  //cerr << "initialGuess: " << t2v(ig).transpose() << endl;
    projector->setCameraMatrix(toCameraMatrix);
    projector->setImageSize(toRows,toCols);
    projector->scale(1./_scale);
    
  //cerr << "cameraMatrix: " << endl;
  //cerr << projector->cameraMatrix() << endl;
  //cerr << "fromOffset: " << endl;
  //cerr << fromOffset.matrix()<< endl;
  //cerr << "toOffset: " << endl;
  //cerr << toOffset.matrix()<< endl;
  
    r = projector->imageRows();
    c = projector->imageCols();
    // char dbgName[1024];
    // sprintf(dbgName, "match-%06d-%06d",from->seq, to->seq);
    // _aligner->debugPrefix()=dbgName;
    _aligner->correspondenceFinder()->setImageSize(r,c);
    _aligner->setReferenceCloud(fromCloud);
    _aligner->setCurrentCloud(toCloud);
    _aligner->align();
    //_aligner->debugPrefix()=""; FICSMI

    //cerr << "_fromCloud.points():" << fromCloud->points().size() << endl;
    //cerr << "_toCloud.points():" << toCloud->points().size() << endl;
    //cerr << "AlInliers: " << _aligner->inliers() << endl;
    Eigen::Isometry3d relationMean;
    convertScalar(result.transform, _aligner->T());

    Matrix6d omega;
    convertScalar(omega, _aligner->omega());
    omega.setIdentity(); //HACK
    omega*=100;
    result.informationMatrix = omega;
    
    result.cloud_inliers = _aligner->inliers();

    DepthImage 
      currentDepthThumb = _aligner->correspondenceFinder()->currentDepthImage(),
      referenceDepthThumb = _aligner->correspondenceFinder()->referenceDepthImage();
    cv::Mat currentRect, referenceRect;
    DepthImage_convert_32FC1_to_16UC1(currentRect,currentDepthThumb); 
    //currentDepthThumb.toCvMat(currentRect);
    
    DepthImage_convert_32FC1_to_16UC1(referenceRect,referenceDepthThumb); 
    //referenceDepthThumb.toCvMat(referenceRect);

    cv::Mat mask = (currentRect>0) & (referenceRect>0);
    currentRect.convertTo(currentRect, CV_32FC1);
    referenceRect.convertTo(referenceRect, CV_32FC1);
    mask.convertTo(mask, currentRect.type());
    cv::Mat diff = abs(currentRect-referenceRect)&mask;
    int nonZeros = countNonZero(mask);
    float sum=0;
    int inliers = 0;
    for (int i = 0; i<diff.rows; i++)
      for (int j = 0; j<diff.cols; j++){
	float d = diff.at<float>(i,j);
	if (mask.at<float>(i,j) && d < _frameInlierDepthThreshold)
	  inliers ++;
	sum +=d;
      }
    //int imSize = diff.rows*diff.cols;
    result.image_reprojectionDistance = sum/nonZeros;
    result.image_nonZeros = nonZeros;
    result.image_outliers = nonZeros-inliers;
    result.image_inliers = inliers;
  }


  BOSS_REGISTER_CLASS(PwnMatcherBase);

}
