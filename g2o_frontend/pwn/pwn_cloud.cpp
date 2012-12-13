#include "pwn_cloud.h"
#include "pwn_math.h"
#include <iostream>
using namespace std;
using namespace Eigen;

void cloud2mat(Vector6fPtrMatrix& pointsMat,
	       Matrix6fPtrMatrix& informationMat,
	       CovarianceSVDPtrMatrix& svdMat,
	       Vector6fVector& points,
	       Matrix6fVector& omegas,
	       CovarianceSVDVector& svdVec,
	       const Eigen::Isometry3f& X,
	       const Eigen::Matrix3f& cameraMatrix,
	       Eigen::MatrixXf& zBuffer /* temp zBuffer for working. Allocate it outside */,
	       bool acceptUndefined) {
  zBuffer.resize(pointsMat.rows(), pointsMat.cols());
  if(omegas.size())
    informationMat.fill(0);
  if(svdVec.size())
    svdMat.fill(0);

  pointsMat.fill(0);
  zBuffer.fill(std::numeric_limits<float>::max());

  Isometry3f iX = X.inverse();
  AffineCompact3f T;
  T.setIdentity();
  T.linear() = cameraMatrix*iX.linear();
  T.affine() = cameraMatrix*iX.affine();

  for (size_t i = 0; i < points.size(); i++) {
    Vector6f& p = points[i];
    Vector3f pt = T * p.head<3>();
    Vector2f ip = pt.head<2>() * (1.f/pt(2));
    int x = (int)ip(0);
    int y = (int)ip(1);
    if(y < 0 || y >= pointsMat.rows() ||
       x < 0 || x >= pointsMat.cols())
      continue;
    if(zBuffer(y, x) > pt.z()) {
      bool hasNormal = p.tail<3>().squaredNorm() > 1e-3;
      if (hasNormal || acceptUndefined) {
	zBuffer(y, x) = pt.z();
	pointsMat(y, x) = &p;
	if(omegas.size())
	  informationMat(y, x) = &omegas[i];
	if(svdVec.size())
	  svdMat(y, x) = &svdVec[i];
      }
    }
  }
}

void cloud2mat(Vector6fPtrMatrix& pointsMat,
	       Matrix6fPtrMatrix& informationMat,
	       Vector6fVector& points,
	       Matrix6fVector& omegas,
	       const Isometry3f& X,
	       const Matrix3f& cameraMatrix,
	       MatrixXf& zBuffer,
	       bool acceptUndefined) {
  static CovarianceSVDVector dummy_svdVec;
  static CovarianceSVDPtrMatrix dummy_svdMat;
  cloud2mat(pointsMat, informationMat, dummy_svdMat,
	    points, omegas, dummy_svdVec, X, cameraMatrix, zBuffer, acceptUndefined);
}


void cloud2mat(Vector6fPtrMatrix& pointsMat,
               Vector6fVector& points,
               const Isometry3f& X,
               const Matrix3f& cameraMatrix,
               MatrixXf& zBuffer /* temp zBuffer for working. Allocate it outside*/,
	       bool acceptUndefined) {
  static Matrix6fVector dummy_omegas;
  static Matrix6fPtrMatrix dummy_informationMat(0,0);
  cloud2mat(pointsMat, dummy_informationMat, points, dummy_omegas, X, cameraMatrix, zBuffer, acceptUndefined);
}

int mat2cloud(Vector6fVector& destPoints,
	      Matrix6fVector& destOmegas,
	      CovarianceSVDVector& destSVD,
	      Vector6fPtrMatrix& srcPointsMat,
	      Matrix6fPtrMatrix& srcOmegaMat,
	      CovarianceSVDPtrMatrix& srcSvdMat){

  bool addOmegas = srcOmegaMat.cols();
  bool addSvd = srcSvdMat.cols();
  int s = destPoints.size();
  int howMany = srcPointsMat.cols() * srcPointsMat.rows();
  destPoints.reserve(s + howMany);
  if (addOmegas){
    destOmegas.reserve(s+howMany);
  }
  if (addSvd){
    destSVD.reserve(s+howMany);
  }

  int k = 0;
  for (int i = 0; i<srcPointsMat.cols(); i++)
    for (int j=0; j<srcPointsMat.rows(); j++){
      if (! srcPointsMat(j,i))
	continue;
      const Vector6f& v=*srcPointsMat(j,i);
      if (v.tail<3>().squaredNorm()>1e-3){
	destPoints.push_back(v);
	if (addOmegas)
	  destOmegas.push_back(*srcOmegaMat(j,i));
	if (addSvd)
	  destSVD.push_back(*srcSvdMat(j,i));
      }
      k++;
    }
  destPoints.resize(s+k);
  if (addOmegas)
    destOmegas.resize(s+k);
  if (addSvd)
    destSVD.resize(s+k);
  return k;
}


void depth2cloud(Vector6fVector& cloud, const MatrixXf& depth, const Eigen::Matrix3f& cameraMatrix){
  cloud.resize(depth.cols()*depth.rows());
  int k = 0;
  Eigen::Matrix3f iK = cameraMatrix.inverse();
  Vector6f v;
  v.setZero();
  for(int x = 0; x<depth.cols(); x++)
    for (int y = 0;y<depth.rows(); y++){
      const float& d = depth(y,x);
      if (d<=0)
	continue;
      v.head<3>() = iK * Eigen::Vector3f((float)x*d, (float)y*d, d);
      cloud[k++] = v;
    }
  cloud.resize(k);
}

void cloud2depth(MatrixXf& depth, const Vector6fVector& cloud, const Eigen::Matrix3f& cameraMatrix){
  depth.fill(std::numeric_limits<float>::max());
  for (size_t i = 0; i<cloud.size(); i++){
    Eigen::Vector3f ip = cameraMatrix*cloud[i].head<3>();
    ip.head<2>() *= (1.0f/ip(2));
    int x = (int)ip(0);
    int y = (int)ip(1);
    if(y < 0 || y >= depth.rows() ||
       x < 0 || x >= depth.cols())
      continue;
    depth(y,x) = ip(2);
  }
}

void depth2img(MatrixXus& img, const Eigen::MatrixXf& depth){
  img.resize(depth.rows(), depth.cols());
  unsigned short* imptr = img.data();
  const float* depthptr = depth.data();
  int s = img.cols() * img.rows();
  for (int i =0; i<s; i++, depthptr++, imptr++){
    *imptr = (unsigned short) (1e3 * *depthptr);
  }
}

void img2depth(Eigen::MatrixXf& depth, const MatrixXus& img){
  depth.resize(img.rows(), img.cols());
  const unsigned short* imptr = img.data();
  float* depthptr = depth.data();
  int s = img.cols() * img.rows();
  for (int i =0; i<s; i++, depthptr++, imptr++){
    *depthptr = 1e-3 * (float)*imptr;
  } 
}

// Compute list of 6x6 moegas matrices given the covariances of points.
void svd2omega(Matrix6fVector &omega, const CovarianceSVDVector &covariance){
  
  float Kp = 1.0f;
  float Kn = 1.0f;
  float curvatureThreshold = 0.02f;
  float curvatureOffset = 1e-5;
  Diagonal3f flatOmegaDiagonal(1e3, 1e-1, 1e-1);
  omega.resize(covariance.size());
  for (size_t i=0; i<omega.size(); i++){
      const SVDMatrix3f &cov = covariance[i];
      Matrix6f finalOmega=Matrix6f::Identity();
      if(cov.lambda.squaredNorm()>1e-09){
	float curvature = cov.lambda[0] / (cov.lambda[0] + cov.lambda[1] + cov.lambda[2]);
	// Compute points omega block.
	Eigen::Matrix3f omegaP = Eigen::Matrix3f::Zero();
	Eigen::Matrix3f R = cov.isometry.linear();
	Diagonal3f invLambda;
      	if (curvature>curvatureThreshold)
	  invLambda = Diagonal3f(1.0f/cov.lambda[0], 1.0f/cov.lambda[1], 1.0f/cov.lambda[2]);
	else
	  invLambda = flatOmegaDiagonal;
	omegaP = R*invLambda*R.transpose();
	omegaP *= Kp;
	// Compute normals omega block.
	Eigen::Matrix3f omegaN = Eigen::Matrix3f::Identity();
	omegaN *= Kn * 1.0f * curvature / powf((curvature + curvatureOffset), 2);
	// Create final omega matrix
	finalOmega.block<3,3>(0,0) = omegaP;
	finalOmega.block<3,3>(3,3)= omegaN;
      }
      // Add the computed omega to the list.
      omega[i] = finalOmega;
  }
}

// Read and load an image from a .pgm file.
bool readPgm(MatrixXus &image, FILE *pgmFile)
{
  char version[3];
  int height, width;
  int max_value;
  int i, j;
  int lo, hi;
  char *fgetsRes;
  int fscanfRes;
 
  if (pgmFile==NULL)
    return false;
  
  // Check if it's a pgm image.
  fgetsRes = fgets(version, sizeof(version), pgmFile);
  if(fgetsRes && strcmp(version, "P5"))
    return false;
  
  // Read (width, height) of the image and the max gray value for a pixel.
  // Do this while skipping possible comments.
  skipComments(pgmFile);
  fscanfRes = fscanf(pgmFile, "%d", &width);
  if (fscanfRes != 1)
    return false;
  skipComments(pgmFile);
  fscanfRes = fscanf(pgmFile, "%d", &height);
  if (fscanfRes != 1)
    return false;
  skipComments(pgmFile);
  fscanfRes = fscanf(pgmFile, "%d", &max_value);
  if (fscanfRes != 1)
    return false;
  fgetc(pgmFile);
  
  // Read image data (expected 16 bit unsigned char).
  image = MatrixXus(height, width);
  int pixel;
  if (max_value>0xFF){
    for (i=0; i<height; ++i){
      for (j=0; j<width; ++j){
	hi = fgetc(pgmFile);
	lo = fgetc(pgmFile);
	pixel = (hi << 8) + lo;
	image(i, j) = pixel;
      }
    }
  }
  else{
    for (i=0; i<height; ++i){
      for (j=0; j<width; ++j){
	pixel = fgetc(pgmFile);
	image(i, j) = pixel;
      }
    }
  }
  
  return true;
}

// Write an image to a .pgm file.
bool writePgm(const MatrixXus& img, FILE *pgmFile)
{
  int i, j;
  int hi, lo;
  unsigned int max_value = 0xFFFF;
  
  if (pgmFile==NULL)
    return false;

  // Write header for a .pgm file.
  fprintf(pgmFile, "P5 ");
  fprintf(pgmFile, "%d %d ", (int)img.cols(), (int)img.rows());
  fprintf(pgmFile, "%d ", max_value);

  // Write image data.
  for (i=0; i<img.rows(); i++){
    for (j=0; j<img.cols(); j++){
      hi = HI(img(i, j));
      lo = LO(img(i, j));
      fputc(hi, pgmFile);
      fputc(lo, pgmFile);
    }
  }
  
  return true;
}


// Skip possible comments when reading a file .pgm
void skipComments(FILE *fp)
{
  int ch;
  char line[1024];
  char* fgetsRes;
  
  // Each comment begin with a '#', skip all lines
  // that begin with a sharp.
  while ((ch = fgetc(fp)) != EOF && isspace(ch));
  if (ch=='#'){
    fgetsRes = fgets(line, sizeof(line), fp);
    if(fgetsRes)
      skipComments(fp);
  }
  else
    fseek(fp, -1, SEEK_CUR);
}
