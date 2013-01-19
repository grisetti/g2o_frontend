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

struct Frame{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  DepthImage depthImage;
  MatrixXi indexImage;
  PointWithNormalVector points;
  PointWithNormalSVDVector svds;
  MatrixXf zBuffer;
  

  bool load(std::string filename) {
    return depthImage.load(filename.c_str());
  }
  void computeStats(PointWithNormalStatistcsGenerator & generator, const Matrix3f& cameraMatrix){
    zBuffer.resize(depthImage.rows(), depthImage.cols());
    points.fromDepthImage(depthImage,cameraMatrix,Isometry3f::Identity());
    indexImage.resize(depthImage.rows(), depthImage.cols());
    points.toIndexImage(indexImage, zBuffer, cameraMatrix, Eigen::Isometry3f::Identity(), 10);
    cerr << "points: " << points.size() << endl; 
    svds.resize(points.size());
    double tNormalStart = get_time();
    generator.computeNormalsAndSVD(points, svds, indexImage, cameraMatrix);
    double tNormalEnd = get_time();
    cerr << "Normal Extraction took " << tNormalEnd - tNormalStart << " sec." << endl;
  }
  void setAligner(PointWithNormalAligner& aligner, bool isRef){
    if (isRef) {
      aligner.setReferenceCloud(&points, &svds);
    } else {
      aligner.setCurrentCloud(&points, &svds);
    }
  }
};

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

  PointWithNormalStatistcsGenerator normalGenerator;
  PointWithNormalAligner aligner;

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
  arg.param("numThreads", numThreads, 1, "numver of threads for openmp");
  

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


  Frame* referenceFrame= 0;

  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;

  ostringstream os(graphFilename.c_str());
  

  cerr << "there are " << filenames.size() << " files  in the pool" << endl; 
  Isometry3f trajectory;
  int previousIndex=-1;
  int graphNum=0;
  int nFrames = 0;
  string baseFilename = graphFilename.substr( 0, graphFilename.find_last_of( '.' ) +1 );
  for (size_t i=0; i<filenames.size(); i++){
    cerr << endl << endl << endl;
    cerr << ">>>>>>>>>>>>>>>>>>>>>>>> PROCESSING " << filenames[i] << " <<<<<<<<<<<<<<<<<<<<" <<  endl;
    Frame* currentFrame= new Frame();
    if(!currentFrame->load(filenames[i])) {
      cerr << "failure in loading image: " << filenames[i] << ", skipping" << endl;
      delete currentFrame;
      continue;
    }
    nFrames ++;
    if (! referenceFrame ){
      os << "PARAMS_CAMERACALIB 0 0 0 0 0 0 0 1 525 525 319.5 239.5"<< endl;
      Vector6f t0;
      t0 << 0,0,0,0.5, 0.5, 0.5;
      trajectory= v2t(t0);
    }
    {
      // write the vertex frame
      Vector6f x = t2v(trajectory);
      Vector3f t = x.head<3>();
      Vector3f mq = x.tail<3>();
      float w = mq.squaredNorm();
      if (w>1){
	mq.setZero();
	w = 1.0f;
      } else {
	w = sqrt(1-w);
      }
      
      os << "VERTEX_SE3:QUAT " << i << " " << t.transpose() << " " << mq.transpose() << " " << w << endl;
      os << "DEPTH_IMAGE_DATA 0 " << filenames[i] << " 0 0 " << endl;
    }
    
    currentFrame->computeStats(normalGenerator,cameraMatrix);
    if (referenceFrame) {
      referenceFrame->setAligner(aligner, true);
      currentFrame->setAligner(aligner, false);

      Matrix6f omega;
      Vector6f mean;
      float tratio;
      float rratio;
      aligner.setImageSize(currentFrame->depthImage.rows(), currentFrame->depthImage.cols());
      Eigen::Isometry3f Xold = trajectory;
      Eigen::Isometry3f X=Xold;
      //X.setIdentity();
      double ostart = get_time();
      float error;
      int result = aligner.align(error, X, mean, omega, tratio, rratio);
      Eigen::Isometry3f dX = Xold.inverse()*X;;
      cerr << "inliers=" << result << " error/inliers: " << error/result << endl;
      cerr << "localTransform : " << endl;
      cerr << dX.matrix() << endl;
      trajectory=X;
      cerr << "globaltransform: " << endl;
      cerr << trajectory.matrix() << endl;
      double oend = get_time();
      cerr << "alignment took: " << oend-ostart << " sec." << endl;
      cerr << "aligner scaled image size: " << aligner.scaledImageRows() << " " << aligner.scaledImageCols() << endl;
      
      if(rratio < 100 && tratio < 100) {
	// write the edge frame
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
	
	os << "EDGE_SE3:QUAT " << previousIndex << " " << i << " ";
	os << t.transpose() << " " << mq.transpose() << " " << w <<  " ";
	for (int r=0; r<6; r++){
	  for (int c=r; c<6; c++){
	    os << omega(r,c) << " ";
	  }
	} 
	os << endl;
      } else {
	if (nFrames >10) {
	  char buf[1024];
	  sprintf(buf, "%s-%03d.g2o", baseFilename.c_str(), graphNum);	
	  ofstream gs(buf);
	  gs << os.str();
	  gs.close();
	}
	os.str("");
	os.clear();
	if (referenceFrame)
	  delete referenceFrame;
	referenceFrame = 0;
	graphNum ++;
	nFrames = 0;
      }

    }
    previousIndex = i;
    if (referenceFrame)
      delete referenceFrame;
    referenceFrame = currentFrame;
    for (size_t k=0; k<currentFrame->points.size(); k++){
      currentFrame->points[k] = trajectory * currentFrame->points[k];
      currentFrame->svds[k] = trajectory * currentFrame->svds[k];
    }
  }

  char buf[1024];
  sprintf(buf, "%s-%03d.g2o", baseFilename.c_str(), graphNum);	
  cerr << "saving final frames, n: " << nFrames << " in file [" << buf << "]" << endl;
  cerr << "filesize:" << os.str().length() << endl; 
  ofstream gs(buf);
  gs << os.str();
  cout << os.str();
  gs.close();
  
}
