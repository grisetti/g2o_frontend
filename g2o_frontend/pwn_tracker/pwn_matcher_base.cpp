#include "pwn_matcher_base.h"
#include "g2o_frontend/pwn_core/pwn_static.h"
#include "g2o/stuff/timeutil.h"

namespace pwn_tracker {
  using namespace std;


  PwnMatcherBase::PwnMatcherBase(pwn::Aligner* aligner_, pwn::DepthImageConverter* converter_){
    _aligner = aligner_;
    _converter = converter_;
    _scale = 2;
    _frameInlierDepthThreshold = 50;
  }


  void PwnMatcherBase::makeThumbnails(cv::Mat& depthThumbnail, cv::Mat& normalThumbnail, 
				      pwn::Cloud* f, int r, int c, 
				  const Eigen::Isometry3f& offset, 
				  const Eigen::Matrix3f& cameraMatrix,
				  float scale){



    PinholePointProjector proj;
    proj.setImageSize(r,c);
    proj.setCameraMatrix(cameraMatrix);
    proj.scale(scale);
    pwn::IntImage indices(proj.imageRows(), proj.imageCols());
    pwn::DepthImage depthBuffer(proj.imageRows(), proj.imageCols());
    proj.setTransform(offset);
    proj.project(indices, depthBuffer, f->points());
    normalThumbnail = cv::Mat(proj.imageCols(), proj.imageRows(), CV_8UC3);
    depthThumbnail = cv::Mat(proj.imageCols(), proj.imageRows(), CV_16UC1);
    
    DepthImage_convert_32FC1_to_16UC1(depthThumbnail, depthBuffer); 
    //depthBuffer.toCvMat(depthThumbnail);
    for (int j = 0; j<indices.rows; j++) {
      for (int i = 0; i<indices.cols; i++){
      	cv::Vec3b& pixel = normalThumbnail.at<cv::Vec3b>(i,j);
    	int idx = indices(j,i);
    	pixel[0] = 0;
    	pixel[1] = 0;
    	pixel[2] = 0;
    	if (idx==0)
    	  continue;
    	const pwn::Normal& n = f->normals()[idx];
    	if (n.squaredNorm()<0.1)
    	  continue;
    	pixel[0] = 127*(n.x()-0.5);
    	pixel[1] = 127*(n.y()-0.5);
    	pixel[2] = 127*(n.z()-0.5);
      }
    }
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



}
