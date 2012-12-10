#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/StdVector>

using namespace std;
using namespace Eigen;

typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<float, 6, 6> Matrix6f;

typedef std::vector<Vector6f,Eigen::aligned_allocator<Vector6f> > Vector6fVector;
typedef std::vector<Matrix6f,Eigen::aligned_allocator<Matrix6f> > Matrix6fVector;

typedef Eigen::Matrix<Vector6f*, Dynamic, Dynamic> Vector6fPtrMatrix;
typedef Eigen::Matrix<Matrix6f*, Dynamic, Dynamic> Matrix6fPtrMatrix;

inline Matrix3f quat2mat(const Vector3f& q) {
  const float& qx = q.x();
  const float& qy = q.y();
  const float& qz = q.z();
  float qw = sqrt(1.f - q.squaredNorm());
  Matrix3f R;
  R << 
    qw*qw + qx*qx - qy*qy - qz*qz, 	 2*(qx*qy - qw*qz) ,  2*(qx*qz + qw*qy),
    2*(qx*qy + qz*qw) , qw*qw - qx*qx + qy*qy - qz*qz,  2*(qy*qz - qx*qw),
    2*(qx*qz - qy*qw) , 2*(qy*qz + qx*qw), qw*qw - qx*qx - qy*qy + qz*qz;
  return R;
}

inline Vector3f mat2quat(const Matrix3f& R) {
  float n = 1./(2*sqrt(1+R(0,0)+R(1,1)+R(2,2)));
  return Vector3f(n*(R(2,1)-R(1,2)),
		  n*(R(0,2)-R(2,0)),
		  n*(R(1,0)-R(0,1)));
}

inline Isometry3f v2t(const Vector6f& x) {
  Isometry3f X;
  X.linear() = quat2mat(x.tail<3>());
  X.translation() = x.head<3>();
  return X;
}

inline Vector6f t2v(const Isometry3f& X){
  Vector6f v;
  v.head<3>() = X.translation();
  v.tail<3>() = mat2quat(X.linear());
}

inline Matrix3f skew(const Vector3f& v) {
  const float& tx = v.x();
  const float& ty = v.y();
  const float& tz = v.z();
  Matrix3f S;
  S << 0, (2*tz), (-2*ty), 
       (-2*tz), 0, (2*tx),
       (2*ty),  (-2*tx),0;
  return S;
}

inline Vector6f remapPoint(const Isometry3f& X, const Vector6f p){
  Vector6f p2;
  p2.head<3>() = X.linear()*p.head<3>() + X.translation();
  p2.tail<3>() = X.linear()*p.tail<3>();
}


inline Matrix6f jacobian(const Eigen::Isometry3f& X, const Vector6f& p){
  Matrix6f J = Matrix6f::Zero();
  const Matrix3f& R = X.linear();
  const Vector3f& t = p.head<3>();
  const Vector3f& n = p.tail<3>();
  J.block<3,3>(0,0) = R;
  J.block<3,3>(0,3) = R * skew(t);
  J.block<3,3>(3,3) = R * skew(n);
  return J;
}

inline int pwn_iteration(float& error, Isometry3f& Xnew, 
			 const Vector6f* const * refPoints, 
			 const Vector6f* const * currPoints, 
			 const Matrix6f* const * omegas,
			 size_t n,
			 const Isometry3f& X,
			 float inlierThreshold /*inlier threshold*/,
			 int minInliers) {
  Vector6f b = Vector6f::Zero();
  Matrix6f H = Matrix6f::Zero();
  error = 0;
  Xnew = X;

  int numInliers = 0;
  for (size_t i = 0; i<n; i++){
    if(!refPoints[i] || !currPoints[i])
      continue;
    const Vector6f& pref  = *refPoints[i];
    const Vector6f& pcurr = *currPoints[i];
    const Matrix6f& omega = *omegas[i];

    Vector6f e = pref - remapPoint(X, pcurr);
    float localError = e.transpose() * omega * e;
    if (localError>inlierThreshold)
      continue;
    error += localError;
    numInliers++;
    Matrix6f J = -jacobian(X, pcurr);
    b += J.transpose() * omega * e;
    H += J.transpose() * omega * J;
  }
  if (numInliers<minInliers)
    return numInliers;
  Vector6f dx = H.ldlt().solve(-b);
  Xnew  = X*v2t(dx);
  return numInliers;
}

inline void cloud2img(Vector6fPtrMatrix& pointsImage, 
		      Matrix6fPtrMatrix& informationImage,
		      Vector6fVector& points, 
		      Matrix6fVector& omegas,
		      const Isometry3f& X,
		      const Matrix3f& cameraMatrix,
		      MatrixXf& zBuffer /* temp zBuffer for working. Allocate it outside*/) {
  
  zBuffer.resize(pointsImage.rows(), pointsImage.cols());
  if (omegas.size())
    informationImage.fill(0);

  pointsImage.fill(0);
  zBuffer.fill(std::numeric_limits<double>::max());

  Isometry3f iX = X.inverse();
  AffineCompact3f T;
  T.setIdentity();
  T.linear()=cameraMatrix*iX.linear();
  T.affine()=cameraMatrix*iX.affine();

  for (size_t i=0; i<points.size(); i++){
    Vector6f& p = points[i];
    Vector3f pt = T * p.head<3>();
    Vector2f ip  = pt.head<2>() * (1.f/pt(2));
    int x=(int)ip(0);
    int y=(int)ip(1);
    if (y<0 || y >= pointsImage.cols() ||
	x<0 || x >= pointsImage.rows() )
      continue;
    if (zBuffer(y,x)>pt.z()){
      zBuffer(y,x)=pt.z();
      pointsImage(y,x) = &p;
      if (omegas.size())
	informationImage(y,x) = &omegas[i];
    }
  }
}


void cloud2img(Vector6fPtrMatrix& pointsImage, 
	       Vector6fVector& points, 
	       const Isometry3f& X,
	       const Matrix3f& cameraMatrix,
	       MatrixXf& zBuffer /* temp zBuffer for working. Allocate it outside*/) {
  static Matrix6fVector dummy_omegas;
  static Matrix6fPtrMatrix dummy_informationImage(0,0);
  cloud2img(pointsImage, dummy_informationImage, points, dummy_omegas, X, cameraMatrix, zBuffer );
}

/**
computes the aligment of range images with normals
@returns the number of inliers

Returned parameters:
float& error // final error of the function
Isometry3f& Xnew // transform that moves pref onto pcurr

Input parameters:
Vector6fVector& refPoints // points in the reference scan
Vector6fVector& currPoints,  // points in the moving scan
Matrix6fVector& currInformation, // omegas of the moving scan
Isometry3f& X, // initial transform
Matrix3f& cameraMatrix, // camer matrix
int rows, // how big image
int cols,
float inlierThreshold // error to consider a point an inlier
int minInliers, // how  many outliers
int innerIterations, // inner iterations for least sqares
int outerIterations, // outetr iterations
*/

inline int pwn_align(float& error,    
		     Isometry3f& Xnew, 
		     Vector6fVector& refPoints, 
		     Vector6fVector& currPoints, 
		     Matrix6fVector& currInformation,
		     //input
		     Isometry3f& X,
		     Matrix3f& cameraMatrix,
		     int rows,
		     int cols,
		     float inlierThreshold /*inlier threshold*/,
		     int minInliers,
		     int innerIterations,
		     int outerIterations) {
  // allocate the reference images
  Vector6fPtrMatrix refImage(rows, cols);

  Vector6fPtrMatrix currImage(rows, cols);
  Matrix6fPtrMatrix currInformationImage(rows, cols);
  
  MatrixXf zBuffer(rows, cols);
  // construct the pointer map for the current image
  cloud2img(currImage, currInformationImage,
	    currPoints, currInformation,
	    Isometry3f::Identity(), cameraMatrix, zBuffer);
  Isometry3f Xcurrent = X;
  int inliers = 0;
  int size=cols*rows;
  for (int i=0; i<outerIterations; i++){
    // remap the reference frame in the local one;
    cloud2img(refImage, refPoints, Xcurrent, cameraMatrix, zBuffer);
    for (int j=0; j<innerIterations; j++){
      cerr << "\tinner iteration" << j << endl;
      float iterationError;
      Isometry3f _xNew;
      inliers =  pwn_iteration(iterationError, _xNew, 
			       refImage.data(), currImage.data(), currInformationImage.data(), size, 
			       Xcurrent, inlierThreshold, minInliers);
      cerr << "\t\tinliers: " << inliers <<  " error:" << error << endl;
      if (inliers<minInliers) {
	return inliers;
	cerr << "\t\treject! " << endl;
      }
      Xcurrent = _xNew;
      cerr << "\t\taccept! " << endl;
      cerr << Xcurrent.linear() <<  " " << Xcurrent.translation() << endl; 
    }
  }
  if (inliers<minInliers)
    Xnew = Xcurrent;
}


/*
todo:
unit tests quat2mat/mat2quat
unit tests t2v/v2t
unit tests remapPoint
unit tests cloud2img
unit tests pwn_iteration (compare with octave !!!!)

//
validation of the concept fro dinamic association
- plot the associations and see how they look like
- run the optimization in step-by-step mode
- pimp the code
*/
