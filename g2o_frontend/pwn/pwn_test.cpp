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
  g2o::CommandArgs arg;
  string imageName0;
  arg.paramLeftOver("image0", imageName0, "", "image0", true);
  arg.parseArgs(argc, argv);

  /***********************************************************************************/
  cout << "Test 1 Depth Image load" << endl;
  DepthImage depthImage0;
  int result = depthImage0.load(imageName0.c_str());
  if (! result){
    cout << "load failed" << endl;
    return 0;
  }

  /***********************************************************************************/

  cout << "Test 2 Depth Image save" << endl;
  result = depthImage0.save("test1.pgm");
  if (! result){
    cout << "save failed" << endl;
    return 0;
  }
  cout << "check that the two files are binary the same" << endl;

  /***********************************************************************************/
  
  cout << "Test 3 generating a cloud from an image" << endl;
  PointWithNormalVector points0;
  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;

  points0.fromDepthImage((const MatrixXf&)depthImage0, cameraMatrix, Eigen::Isometry3f::Identity(), 10);
  cerr << " generated " << points0.size() << endl;
  points0.save("test3.pwn");

  /***********************************************************************************/
  cout << "Test 4 generating an image from a cloud" << endl;
  DepthImage depthImage1(depthImage0.rows(), depthImage0.cols());
  depthImage1.fill(0);
  points0.toDepthImage(depthImage1, cameraMatrix, Eigen::Isometry3f::Identity(), 10);

  result = depthImage1.save("test4.pgm");
  if (! result){
    cout << "save failed" << endl;
    return 0;
  }

  /***********************************************************************************/

  cout << "Test 5 loading a cloud and generating an image";
  points0.load("test3.pwn");
  depthImage1.fill(0);
  points0.toDepthImage(depthImage1, cameraMatrix, Eigen::Isometry3f::Identity(), 10);

  result = depthImage1.save("test5.pgm");
  if (! result){
    cout << "save failed" << endl;
    return 0;
  }

  /***********************************************************************************/

  cout << "Test 6 integral image validation";
  MatrixXi indexImage(depthImage0.rows(), depthImage0.cols());
  Eigen::MatrixXf zBuffer(depthImage0.rows(), depthImage0.cols());
  
  points0.toIndexImage(indexImage, zBuffer, cameraMatrix, Eigen::Isometry3f::Identity(), 10);
  IntegralPointImage integralImage;
  integralImage.compute(indexImage,points0);

  int xmin = .25 *indexImage.cols();
  int ymin = .25 *indexImage.rows();
  int xmax = .5 *indexImage.cols();
  int ymax  = .5 *indexImage.rows();
  PointAccumulator accTest = integralImage.getRegion(xmin, xmax, ymin, ymax);
  PointAccumulator accTest2;
  int _n = 0;
  Vector3f _mean;
  _mean.setZero();
  for(int c=xmin; c<xmax; c++){
    for(int r=ymin; r<ymax;  r++){
      int index = indexImage(r,c);
      if (index>0){
	accTest2 += points0[index].head<3>();
	_mean+=points0[index].head<3>();
	_n++;
      }
    }
  }
  _mean *= 1./(float) _n;

  // covariance
  Eigen::Matrix3f _cov;
  _cov.setZero();
  for(int c=xmin; c<xmax; c++){
    for(int r=ymin; r<ymax;  r++){
      int index = indexImage(r,c);
      if (index>0){
	Vector3f dp = points0[index].head<3>() - _mean;
	_cov += dp*dp.transpose();
      }
    }
  }
  _cov *= 1./(float) _n;

  cerr << "ideal " << endl;
  cerr << "\t nPoints: " << _n << endl;
  cerr << "\t mean: " << _mean.transpose() << endl;
  cerr << "\t cov: " << endl <<  _cov << endl;

  cerr << "iterative " << endl;
  cerr << "\t nPoints: " << accTest2.n() << endl;
  cerr << "\t mean: " << accTest2.mean().transpose() << endl;
  cerr << "\t cov: " << endl << accTest2.covariance() << endl;

  cerr << "integralimage " << endl;
  cerr << "\t nPoints: " << accTest.n() << endl;
  cerr << "\t mean: " << accTest.mean().transpose() << endl;
  cerr << "\t cov: " << endl << accTest.covariance() << endl;


  /***********************************************************************************/

  cout << "Test 7 normals computation";

  double nStart = g2o::get_time();
  PointWithNormalStatistcsGenerator normalGenerator;
  int cycles = 10;
  PointWithNormalSVDVector svds0(points0.size());
  for (int i=0; i<cycles; i++){
    normalGenerator.computeNormalsAndSVD(points0, svds0, indexImage, cameraMatrix);
    cerr << ".";
  }
  double nEnd = g2o::get_time() - nStart;
  cerr << "took "  << nEnd/cycles  << " secs/iteration" << endl;

  points0.save("test7.pwn");
  ofstream os;
  os.open("test7.dat");
  savegnuplot(os, points0, 0.1, 10);
  os.close();
  
  /***********************************************************************************/
  cout << "Test 8 aligment";
  PointWithNormalAligner aligner;
  aligner.setImageSize(indexImage.rows(), indexImage.cols());
  aligner.setScale(.25);
  aligner.setReferenceCloud(&points0, &svds0);
  aligner.setCurrentCloud(&points0, &svds0);
  aligner.setOuterIterations(3);
  aligner.setLinearIterations(1);
  aligner.setNonLinearIterations(1);
  aligner.setLambda(1e3);
  aligner.setDebug(false);
  Eigen::Isometry3f X;
  X.setIdentity();
  X.translation() = Vector3f(.1, .1, -.1);
  float error = 0;
  double ostart = get_time();
  result = aligner.align(error, X);
  cerr << "result=" << result << endl;
  cerr << "transform: " << endl;
  cerr << X.matrix() << endl;
  double oend = get_time();
  cerr << "alignment took: " << oend-ostart << " sec." << endl;
}
