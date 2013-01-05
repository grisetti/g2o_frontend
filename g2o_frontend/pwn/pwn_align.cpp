#include "g2o/stuff/command_args.h"
#include "depthimage.h"
#include "pointwithnormal.h"
#include "pointwithnormalstatsgenerator.h"
#include "pointwithnormalaligner.h"
#include <iostream>
#include <fstream>
#include <string>
#include "g2o/stuff/timeutil.h"

using namespace Eigen;
using namespace g2o;
using namespace std;

void savegnuplot(ostream& os, const PointWithNormalVector& points, float scale= 0.1, int step=1) {
  for (size_t i =0; i<points.size(); i+=step) {
    const PointWithNormal & pwn=points[i];
    Vector3f v = pwn.point();
    if (pwn.normal().squaredNorm() > 0.1) {
      os << v.x() <<  " " <<  v.y() << " " << v.z() << endl;
      v+=pwn.normal()*scale;
      os << v.x() <<  " " <<  v.y() << " " << v.z() << endl;
      os << endl;
      os << endl;
    }

  }
}

int
 main(int argc, char** argv){
  string imageName0;
  string imageName1;
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

  arg.paramLeftOver("image0", imageName0, "", "image0", true);
  arg.paramLeftOver("image1", imageName1, "", "image1", true);
  arg.parseArgs(argc, argv);


  normalGenerator.setStep(ng_step);
  normalGenerator.setMinPoints(ng_minPoints);
  normalGenerator.setImageRadius(ng_imageRadius);
  normalGenerator.setWorldRadius(ng_worldRadius);
  normalGenerator.setMaxCurvature(ng_maxCurvature);
  
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

  if (! imageName0.length()){
    cerr << "no image provided" << endl;
    return 0;
  }

  if (! imageName0.length()){
    cerr << "no second image provided" << endl;
    return  0;
  }
    
  DepthImage image0, image1;
  bool loadIm0 = image0.load(imageName0.c_str());
  if (! loadIm0) {
    cerr <<  "failure in loading image: " << imageName0 << endl;
    return 0;
  }

  bool loadIm1 = image1.load(imageName1.c_str());
  if (! loadIm1) {
    cerr <<  "failure in loading image: " << imageName1 << endl;
    return 0;
  }
  
  if (! image0.cols()==image1.cols() || image0.rows()!=image1.rows()){
    cerr << "images must have the same size, this is not the case" << endl;
    return 0;
  }
  
  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;

  // compute the points
  PointWithNormalVector points0;
  points0.fromDepthImage(image0,cameraMatrix,Isometry3f::Identity());
  PointWithNormalVector points1;
  points1.fromDepthImage(image1,cameraMatrix,Isometry3f::Identity());
  
  cerr << "points0: " << points0.size();
  cerr << "points1: " << points1.size();


  // compute the index images
  MatrixXi indexImage0(image0.rows(), image0.cols());
  Eigen::MatrixXf zBuffer0(image0.rows(), image0.cols());
  points0.toIndexImage(indexImage0, zBuffer0, cameraMatrix, Eigen::Isometry3f::Identity(), 10);

  MatrixXi indexImage1(image1.rows(), image1.cols());
  Eigen::MatrixXf zBuffer1(image1.rows(), image1.cols());
  points1.toIndexImage(indexImage1, zBuffer1, cameraMatrix, Eigen::Isometry3f::Identity(), 10);

  // compute the normals for the two images
  PointWithNormalSVDVector svds0(points0.size());
  PointWithNormalSVDVector svds1(points1.size());
  normalGenerator.computeNormalsAndSVD(points0, svds0, indexImage0, cameraMatrix);
  normalGenerator.computeNormalsAndSVD(points1, svds1, indexImage1, cameraMatrix);

  aligner.setImageSize(image1.rows(), image1.cols());
  aligner.setReferenceCloud(&points0, &svds0);
  aligner.setCurrentCloud(&points1, &svds1);
  Eigen::Isometry3f X;
  X.setIdentity();
  double ostart = get_time();
  float error;
  int result = aligner.align(error, X);
  cerr << "result=" << result << endl;
  cerr << "transform: " << endl;
  cerr << X.inverse().matrix() << endl;
  double oend = get_time();
  cerr << "alignment took: " << oend-ostart << " sec." << endl;

  cerr << "aligner scaled image size: " << aligner.scaledImageRows() << " " << aligner.scaledImageCols() << endl;
  
}
