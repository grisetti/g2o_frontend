#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <sstream>

#include "g2o/stuff/command_args.h"
#include "depthimage.h"
#include "pointwithnormal.h"
#include "pointwithnormalstatsgenerator.h"
#include "pointwithnormalaligner.h"
#include "g2o/stuff/timeutil.h"
#include "scene.h"
#include "pixelmapper.h"

using namespace Eigen;
using namespace g2o;
using namespace std;

set<string> readDir(std::string dir){
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  std::set<std::string> filenames;
  dp = opendir( dir.c_str() );
  if (dp == NULL){
    return filenames;
  }
  
  while ((dirp = readdir( dp ))) {
    string filepath = dir + "/" + dirp->d_name;

    // If the file is a directory (or is in some way invalid) we'll skip it 
    if (stat( filepath.c_str(), &filestat )) continue;
    if (S_ISDIR( filestat.st_mode ))         continue;

    filenames.insert(filepath);
  }

  closedir( dp );
  return filenames;
}

#if 1

struct PointsWithNormalMerger{
  std::vector<int> _collapsedIndices;
  Eigen::Matrix3f _cameraMatrix;
  Eigen::MatrixXf _zBuffer;
  Eigen::MatrixXi _indexImage;
  float _distanceThreshold;
  float _normalThreshold;
  float _maxPointDepth;
  PointWithNormalStatistcsGenerator* _normalGenerator;

  PointsWithNormalMerger(){
    _distanceThreshold = 0.1;
    _normalThreshold = cos(20*M_PI/180.0f);
    _normalGenerator = 0;
    _indexImage.resize(480,640);
    _zBuffer.resize(480,540);
    _maxPointDepth = 10;
    _cameraMatrix << 
      525.0f, 0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f, 0.0f, 1.0f;

  }
  void setImageSize(int r, int c) {
    _indexImage.resize(r,c);
    _zBuffer.resize(r,c);
  }
  
  Eigen::Vector2i imageSize() const {
    return Eigen::Vector2i(_indexImage.rows(), _indexImage.cols());
  }

  inline void setDistanceThreshold(float distanceThreshold_) {
    _distanceThreshold = distanceThreshold_;
  }

  inline float distanceThreshold() const {
    return _distanceThreshold;
  }

  inline void setNormalThreshold(float normalThreshold_) {
    _normalThreshold = normalThreshold_;
  }

  inline float normalThreshold() const {
    return _normalThreshold;
  }
  
  inline void setNormalGenerator(PointWithNormalStatistcsGenerator* normalGenerator_) { _normalGenerator = normalGenerator_;} 

  inline PointWithNormalStatistcsGenerator* normalGenerator() {return _normalGenerator;} 
  
  void setCameraMatrix(const Eigen::Matrix3f cameraMatrix_) {_cameraMatrix = cameraMatrix_;}

  void merge(Scene* scene, Eigen::Isometry3f& transform){
    assert (_normalGenerator && "you need to set the normal generator");
    scene->_points.toIndexImage(_indexImage, _zBuffer, _cameraMatrix, transform, _maxPointDepth);

    // scan all the points, 
    // if they fall in a cell not with -1, 
    //   skip
    // if they fall in a cell with n>1, 
    //   if distance is incompatible,
    //      skip
    // if notrmals are incompatible
    //      skip
    // accumulate the point in the cell i
    // set the target accumulator  to i;
    PixelMapper pixelMapper;
    pixelMapper.setCameraMatrix(_cameraMatrix);
    pixelMapper.setTransform(transform);

    _collapsedIndices.resize(scene->points().size());
    std::fill(_collapsedIndices.begin(), _collapsedIndices.end(), -1);
    
    int killed=0;
    int currentIndex=0;
    for (PointWithNormalVector::const_iterator it = scene->_points.begin(); it!=scene->_points.end(); currentIndex++ ,it++){
      const PointWithNormal& currentPoint = *it;
      Vector3f ip=pixelMapper.projectPoint(currentPoint.head<3>());
      Vector2i coords=pixelMapper.imageCoords(ip);
      if (ip.z()<0 || ip.z() > _maxPointDepth || 
	  coords.x()<0 || coords.x()>=_zBuffer.cols() || 
	  coords.y()<0 || coords.y()>=_zBuffer.rows())
	continue;
      float& targetZ = _zBuffer(coords.y(), coords.x());
      int targetIndex =_indexImage(coords.y(), coords.x());
      const PointWithNormal& targetPoint = scene->points()[targetIndex];
      float currentZ=ip.z();
      if (targetIndex <0){
	continue;
      }
      if (targetIndex == currentIndex){
	_collapsedIndices[currentIndex] = currentIndex;
      } else if (fabs(currentZ-targetZ)<_distanceThreshold && currentPoint.normal().dot(targetPoint.normal()) > _normalThreshold){
	Gaussian3f& targetGaussian = scene->_gaussians[targetIndex];
	Gaussian3f& currentGaussian = scene->_gaussians[currentIndex];
	targetGaussian.addInformation(currentGaussian);
	_collapsedIndices[currentIndex]=targetIndex;
	killed ++;
      }
    }
    cerr << "killed: " << killed  << endl;
    // scan the vector of covariances.
    // if the index is -1
    //    copy into k
    //    increment k 
    // if the index is the same,
    //    update the point with normal
    //    copy into k
    //    increment k

    int murdered = 0;
    int k=0;
    for (size_t i=0; i<_collapsedIndices.size(); i++){
      int collapsedIndex = _collapsedIndices[i];
      if (collapsedIndex == (int)i){
	scene->_points[i].setPoint(scene->_gaussians[i].mean());
      }
      if (collapsedIndex <0 || collapsedIndex == (int)i){
	scene->_points[k] = scene->_points[i];
	scene->_svds[k] = scene->_svds[i];
	scene->_gaussians[k] = scene->_gaussians[i];
	k++;
      } else {
	murdered ++;
      }
    }
    
    int originalSize = scene->size();
    // kill the leftover points
    scene->_points.resize(k);
    scene->_gaussians.resize(k);
    scene->_svds.resize(k);
    cerr << "murdered: " << murdered  << endl;
    cerr << "resized: " << originalSize << "->" << k << endl;
    
    // recompute the normals
    scene->_points.toIndexImage(_indexImage, _zBuffer, _cameraMatrix, transform, 10);
    _normalGenerator->computeNormalsAndSVD(scene->_points, scene->_svds, _indexImage, _cameraMatrix, transform);
  }

};

#endif

void writeParams(ostream& os, int paramIndex, const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose = Eigen::Isometry3f::Identity()){
  Quaternionf q(cameraPose.rotation());
  os << "PARAMS_CAMERACALIB " 
     << paramIndex << " "
     << cameraPose.translation().transpose() << " "
     << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
     << cameraMatrix(0,0) << " " << cameraMatrix(1,1) << " " << cameraMatrix(0,2) << " " << cameraMatrix(1,2)<< endl;
}

void writeVertex(ostream& os, int currentIndex, const Vector6f& estimate, const std::string& filename){
    // write the vertex frame
      Vector3f t = estimate.head<3>();
      Vector3f mq = estimate.tail<3>();
      float w = mq.squaredNorm();
      if (w>1){
	mq.setZero();
	w = 1.0f;
      } else {
	w = sqrt(1-w);
      }
      
      os << "VERTEX_SE3:QUAT " << currentIndex << " " << t.transpose() << " " << mq.transpose() << " " << w << endl;
      os << "DEPTH_IMAGE_DATA 0 " << filename << " 0 0 " << endl;
}

void writeEdge(ostream& os, int previousIndex, int currentIndex, const Vector6f& mean, const Matrix6f& omega){
  Vector6f x = mean;
  Vector3f t = x.head<3>();
  Vector3f mq = x.tail<3>();
  float w = mq.squaredNorm();
  if (w>1){
    mq.setZero();
    w = 1.0f;
  } else {
    w = sqrt(1-w);
  }
  
  os << "EDGE_SE3:QUAT " << previousIndex << " " << currentIndex << " ";
  os << t.transpose() << " " << mq.transpose() << " " << w <<  " ";
  for (int r=0; r<6; r++){
    for (int c=r; c<6; c++){
      os << omega(r,c) << " ";
    }
  }
  os << endl;
}

int
 main(int argc, char** argv){
  string dirname;
  string graphFilename;
  float numThreads;

  int ng_step;
  int ng_minPoints;
  int ng_imageRadius;
  float ng_worldRadius;
  float ng_maxCurvature;
  
  float al_inlierDistance;
  float al_inlierCurvatureRatio;
  float al_inlierNormalAngle;
  float al_inlierMaxChi2;
  float al_scale;
  float al_flatCurvatureThreshold;
  float al_outerIterations;
  float al_nonLinearIterations;
  float al_linearIterations;
  float al_minInliers;
  float al_lambda;
  float al_debug;
  float me_distanceThreshold;
  float me_normalThreshold;


  float chunkAngle;
  float chunkDistance;
  int processEachN;
  int chunkEachN;
  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;

  PointWithNormalStatistcsGenerator normalGenerator;
  PointWithNormalAligner aligner;
  PointsWithNormalMerger merger;
  merger.setNormalGenerator(&normalGenerator);
  g2o::CommandArgs arg;
  arg.param("ng_step",ng_step,normalGenerator.step(),"compute a normal each x pixels") ;
  arg.param("ng_minPoints",ng_minPoints,normalGenerator.minPoints(),"minimum number of points in a region to compute the normal");
  arg.param("ng_imageRadius",ng_imageRadius,normalGenerator.imageRadius(), "radius of a ball in the works where to compute the normal for a pixel");
  arg.param("ng_worldRadius",ng_worldRadius,  normalGenerator.worldRadius(), "radius of a ball in the works where to compute the normal for a pixel");
  arg.param("ng_maxCurvature",ng_maxCurvature, normalGenerator.maxCurvature(), "above this threshold the normal is not computed");
  
  arg.param("al_inlierDistance", al_inlierDistance, aligner.inlierDistanceThreshold(),  "max metric distance between two points to regard them as iniliers");
  arg.param("al_inlierCurvatureRatio", al_inlierCurvatureRatio, aligner.inlierCurvatureRatioThreshold(), "max metric distance between two points to regard them as iniliers");
  arg.param("al_inlierNormalAngle", al_inlierNormalAngle, aligner.inlierNormalAngularThreshold(), "max metric distance between two points to regard them as iniliers");
  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, aligner.inlierMaxChi2(), "max metric distance between two points to regard them as iniliers");
  arg.param("al_minInliers", al_minInliers, aligner.minInliers(), "minimum numver of inliers to do the matching");
  arg.param("al_scale", al_scale, aligner.scale(), "scale of the range image for the alignment");
  arg.param("al_flatCurvatureThreshold", al_flatCurvatureThreshold, aligner.flatCurvatureThreshold(), "curvature above which the patches are not considered planar");
  arg.param("al_outerIterations", al_outerIterations, aligner.outerIterations(), "outer interations (incl. data association)");
  arg.param("al_linearIterations", al_linearIterations, aligner.linearIterations(), "linear iterations for each outer one (uses R,t)");
  arg.param("al_nonLinearIterations", al_nonLinearIterations, aligner.nonLinearIterations(), "nonlinear iterations for each outer one (uses q,t)");
  arg.param("al_lambda", al_lambda, aligner.lambda(), "damping factor for the transformation update, the higher the smaller the step");
  arg.param("al_debug", al_debug, aligner.debug(), "prints lots of stuff");
  arg.param("me_distanceThreshold", me_distanceThreshold, merger.distanceThreshold(), "distance along the z when to merge points");
  arg.param("me_normalThreshold", me_normalThreshold, acos(merger.normalThreshold()), "distance along the z when to merge points");


  arg.param("numThreads", numThreads, 1, "numver of threads for openmp");
  arg.param("processEachN",processEachN,1,"compute an image every X") ;
  arg.param("chunkEachN",chunkEachN,1000000,"reset the process every X images") ;
  arg.param("chunkAngle",chunkAngle,M_PI/4,"reset the process each time the camera has rotated of X radians from the first frame") ;
  arg.param("chunkDistance",chunkDistance,0.5,"reset the process each time the camera has moved of X meters from the first frame") ;
  


  if (numThreads<1)
    numThreads = 1;
  
  std::vector<string> fileNames;

  arg.paramLeftOver("dirname", dirname, "", "", true);
  arg.paramLeftOver("graph-file", graphFilename, "out.g2o", "graph-file", true);
  arg.parseArgs(argc, argv);
  cerr << "dirname " << dirname << endl;

  std::vector<string> filenames;
  std::set<string> filenamesset = readDir(dirname);
  for(set<string>::const_iterator it =filenamesset.begin(); it!=filenamesset.end(); it++) {
    filenames.push_back(*it);
  }
  
  normalGenerator.setStep(ng_step);
  normalGenerator.setMinPoints(ng_minPoints);
  normalGenerator.setImageRadius(ng_imageRadius);
  normalGenerator.setWorldRadius(ng_worldRadius);
  normalGenerator.setMaxCurvature(ng_maxCurvature);
#ifdef _PWN_USE_OPENMP_
  normalGenerator.setNumThreads(numThreads);
#endif //_PWN_USE_OPENMP_

  aligner.setInlierDistanceThreshold(al_inlierDistance);
  aligner.setInlierCurvatureRatioThreshold(al_inlierCurvatureRatio);
  aligner.setInlierNormalAngularThreshold(al_inlierNormalAngle);
  aligner.setInlierMaxChi2(al_inlierMaxChi2);
  aligner.setMinInliers(al_minInliers);
  aligner.setScale(al_scale);
  aligner.setFlatCurvatureThreshold(al_flatCurvatureThreshold);
  aligner.setOuterIterations(al_outerIterations);
  aligner.setLinearIterations(al_linearIterations);
  aligner.setNonLinearIterations(al_nonLinearIterations);
  aligner.setLambda(al_lambda);
  aligner.setDebug(al_debug);
#ifdef _PWN_USE_OPENMP_
  aligner.setNumThreads(numThreads);
#endif //_PWN_USE_OPENMP_


  float mergerScale =al_scale;
  Matrix3f mergerCameraMatrix = cameraMatrix;
  mergerCameraMatrix.block<2, 3>(0, 0) *= mergerScale;
  merger.setImageSize(480*mergerScale, 640*mergerScale);
  merger.setCameraMatrix(mergerCameraMatrix);
  merger.setDistanceThreshold(me_distanceThreshold);
  merger.setNormalThreshold(cos(me_normalThreshold));

  DepthFrame* referenceFrame= 0;


  ostringstream os(graphFilename.c_str());
  

  cerr << "there are " << filenames.size() << " files  in the pool" << endl; 
  Isometry3f trajectory;
  trajectory.setIdentity();
  Isometry3f initialFrame = trajectory;
  int previousIndex=-1;
  int nFrames = 0;
  Scene* globalScene = new Scene;
  Scene* partialScene = new Scene;
  int cumPoints=0;
  string baseFilename = graphFilename.substr( 0, graphFilename.find_last_of( '.' ) );
  
  for (size_t i=0; i<filenames.size(); i+=processEachN){
    cerr << endl << endl << endl;
    cerr << ">>>>>>>>>>>>>>>>>>>>>>>> PROCESSING " << filenames[i] << " <<<<<<<<<<<<<<<<<<<<" <<  endl;
    DepthFrame* currentFrame= new DepthFrame();
    DepthImage img;
    if(!img.load(filenames[i].c_str())) {
      cerr << "failure in loading image: " << filenames[i] << ", skipping" << endl;
      delete currentFrame;
      break;
    }
    if (! referenceFrame ){
      writeParams(os, 0, cameraMatrix);
      writeVertex(os, i, t2v(trajectory), filenames[i]);
      initialFrame = trajectory;
    }
    nFrames ++;
    
    currentFrame->_cameraMatrix = cameraMatrix;
    currentFrame->_baseline = 0.075;
    currentFrame->_maxDistance = 10;
    currentFrame->setImage(img);
    currentFrame->_updateSVDsFromPoints(normalGenerator, cameraMatrix);
    currentFrame->_suppressNoNormals();
    if (referenceFrame) {
      globalScene->subScene(*partialScene,cameraMatrix,trajectory);
      //aligner.setReferenceScene(referenceFrame);
      aligner.setReferenceScene(partialScene);
      aligner.setCurrentScene(currentFrame);

      Matrix6f omega;
      Vector6f mean;
      float tratio;
      float rratio;
      aligner.setImageSize(currentFrame->image().rows(), currentFrame->image().cols());
      Eigen::Isometry3f X=trajectory;
      //X.setIdentity();
      double ostart = get_time();
      float error;
      int result = aligner.align(error, X, mean, omega, tratio, rratio);
      cerr << "inliers: " << result << " error/inliers: " << error/result << endl;
      cerr << "localTransform : " << endl;
      cerr << (trajectory.inverse()*X).matrix() << endl;
      trajectory=X;
      cerr << "globaltransform: " << endl;
      cerr << trajectory.matrix() << endl;
      double oend = get_time();
      cerr << "alignment took: " << oend-ostart << " sec." << endl;
      cerr << "aligner scaled image size: " << aligner.scaledImageRows() << " " << aligner.scaledImageCols() << endl;
      
      Eigen::Isometry3f motionFromFirstFrame = initialFrame.inverse()*trajectory;
      Eigen::AngleAxisf rotationFromFirstFrame(motionFromFirstFrame.linear());
      cerr << "motion from first frame: " << motionFromFirstFrame.translation().transpose() << " d:" <<
	motionFromFirstFrame.translation().norm() << endl;
      cerr << "rotation from first frame: " << rotationFromFirstFrame.axis().transpose() << " a:" <<
	rotationFromFirstFrame.angle() << endl;
      
      if(rratio < 100 && tratio < 100 && (i%chunkEachN) && fabs(rotationFromFirstFrame.angle())<chunkAngle && motionFromFirstFrame.translation().norm() < chunkDistance) {
	writeVertex(os, i, t2v(trajectory), filenames[i]);  
	writeEdge(os, previousIndex, i, mean, omega);
	cumPoints += currentFrame->size();
	cerr << "total points that would be added: " << cumPoints << endl;
      } else {
	char buf[1024];
	sprintf(buf, "%s-%05d.g2o", baseFilename.c_str(), (int)i);	
	ofstream gs(buf);
	gs << os.str();
	gs.close();
	sprintf(buf, "%s-%05d.pwn", baseFilename.c_str(), (int)i);	
	globalScene->points().save(buf,1,true);
	os.str("");
	os.clear();
	if (referenceFrame)
	  delete referenceFrame;
	referenceFrame = 0;
	nFrames = 0;
	globalScene->clear();
	initialFrame = trajectory;
      }

    }
    previousIndex = i;
    if (referenceFrame)
      delete referenceFrame;
    referenceFrame = currentFrame;
    globalScene->add(*currentFrame,trajectory);
    merger.merge(globalScene, trajectory);
  }

  char buf[1024];
  sprintf(buf, "%s-%05d.g2o", baseFilename.c_str(), (int)filenames.size());	
  cerr << "saving final frames, n: " << nFrames << " in file [" << buf << "]" << endl;
  cerr << "filesize:" << os.str().length() << endl; 
  ofstream gs(buf);
  gs << os.str();
  //cout << os.str();
  gs.close();
  sprintf(buf, "%s-%05d.pwn", baseFilename.c_str(), (int)filenames.size());	
  cerr << "saving final points, n: " << globalScene->size() << " in file [" << buf << "]" << endl;
  globalScene->points().save(buf,1,true);
  //globalScene->_suppressNoNormals();
    
  
}
