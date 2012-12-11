#include <sstream>
#include <string>
#include "pwn_cloud.h"
#include "pwn_math.h"

using namespace Eigen;
using namespace std;

void cloud2mat(Vector6fPtrMatrix& pointsMat,
	       Matrix6fPtrMatrix& informationMat,
	       Vector6fVector& points,
	       Matrix6fVector& omegas,
	       const Isometry3f& X,
	       const Matrix3f& cameraMatrix,
	       MatrixXf& zBuffer /* temp zBuffer for working. Allocate it outside */) {
  zBuffer.resize(pointsMat.rows(), pointsMat.cols());
  if(omegas.size())
    informationMat.fill(0);

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
      zBuffer(y, x) = pt.z();
      pointsMat(y, x) = &p;
      if(omegas.size())
	informationMat(y, x) = &omegas[i];
    }
  }
}


void cloud2mat(Vector6fPtrMatrix& pointsMat,
               Vector6fVector& points,
               const Isometry3f& X,
               const Matrix3f& cameraMatrix,
               MatrixXf& zBuffer /* temp zBuffer for working. Allocate it outside*/) {
  static Matrix6fVector dummy_omegas;
  static Matrix6fPtrMatrix dummy_informationMat(0,0);
  cloud2mat(pointsMat, dummy_informationMat, points, dummy_omegas, X, cameraMatrix, zBuffer );
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

void depth2img(MatrixXus& img, const Eigen::MatrixXf& depth, const Eigen::Matrix3f& cameraMatrix){
  img.resize(depth.rows(), depth.cols());
  unsigned short* imptr = img.data();
  const float* depthptr = depth.data();
  int s = img.cols() * img.rows();
  for (int i =0; i<s; i++, depthptr++, imptr++){
    *imptr = (unsigned short) (1e3 * *depthptr);
  }
}

void img2depth(Eigen::MatrixXf& depth, const MatrixXus& img, const Eigen::Matrix3f& cameraMatrix){
  depth.resize(img.rows(), img.cols());
  const unsigned short* imptr = img.data();
  float* depthptr = depth.data();
  int s = img.cols() * img.rows();
  for (int i =0; i<s; i++, depthptr++, imptr++){
    *depthptr = 1e-3 * (float)*imptr;
  }
}

const int max_line_dim = 1024;

bool readPgm (MatrixXus& img, istream& is){
  char buf[4];
  string s;
  is >> s;
  if (!is.good() || s != "P5")
    return false;
  // wkip comments
  int width = -1, height = -1, max_val = -1;
  do {
    char buf[max_line_dim];
    is.getline(buf, max_line_dim);
    if (buf[0] && buf[0]=='#')
      continue;
    if (! is.good())
      return 0;
    int v;
    is >> v;
    if (! is.good())
      return 0;
    if (width<0)
      width = v;
    else if (height<0)
      height = v;
    else
      max_val = v;
  } while (max_val<0 && is.good());
  int bpp;
  if (max_val<=0xFF){
    bpp = 1;
    return 0; // unhandled 16 bit image
  } else if (max_val<=0xFFFF){
    bpp = 2;
  } else return false;
  int bsize = bpp*width*height;
  img.resize(width,height);
  is.read((char*)img.data(), bsize);
  if (! is.good())
    return false;
  img = img.transpose();
  return true;
}

bool writePgm(MatrixXus&, ostream& is){
}
