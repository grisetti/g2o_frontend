#include "dm_solve.h"
#include "dm_math.h"
#include <iostream>
#include <cassert>

using namespace Eigen;
using namespace std;

inline Matrix6f jacobian(const Eigen::Isometry3f& X, const Vector6f& p)
{
    Matrix6f J = Matrix6f::Zero();
    const Matrix3f& R = X.linear();
    const Vector3f& t = p.head<3>();
    const Vector3f& n = p.tail<3>();
    J.block<3, 3>(0, 0) = R;
    J.block<3, 3>(0, 3) = R * skew(t);
    J.block<3, 3>(3, 3) = R * skew(n);
    return J;
}

inline int _pwn_linearize(Matrix6f& H, Vector6f& b, float& error,
			  const Vector6f* const * refPoints,
			  const Vector6f* const * currPoints,
			  const Matrix6f* const * omegas,
			  size_t n,
			  const Isometry3f& X,
			  float inlierThreshold){
    b = Vector6f::Zero();
    H = Matrix6f::Zero();
    error = 0;
    int numInliers = 0;
    for(size_t i = 0; i < n; i++)
    {
        if(!refPoints[i] || !currPoints[i])
            continue;
        const Vector6f& pref  = *refPoints[i];
        const Vector6f& pcurr = *currPoints[i];
        const Matrix6f& omega = *omegas[i];

        Vector6f e = remapPoint(X, pref) - pcurr;
	float localError = e.transpose() * omega * e;
        if(localError > inlierThreshold)
            continue;
        numInliers++;
	error += localError;
        Matrix6f J = jacobian(X, pcurr);
        b += J.transpose() * omega * e;
        H += J.transpose() * omega * J;
    }
    return numInliers;
}


inline int _pwn_iteration(float& error, Isometry3f& Xnew,
                         const Vector6f* const * refPoints,
                         const Vector6f* const * currPoints,
                         const Matrix6f* const * omegas,
                         size_t n,
                         const Isometry3f& X,
                         float inlierThreshold, /*inlier threshold*/
			 int minInliers, float lambda=0)
{

    Vector6f b = Vector6f::Zero();
    Matrix6f H = Matrix6f::Zero();
    
    float initialError;
    int initialInliers = _pwn_linearize(H, b, initialError,
					refPoints, currPoints, omegas,
					n, X, inlierThreshold);
    if(initialInliers < minInliers)
        return initialInliers;

    
    Matrix6f H2 = H +Matrix6f::Identity()*lambda;
    Vector6f dx = H2.ldlt().solve(-b);
    Xnew  = X*v2t(dx);
    
    error = 0;
    int inliers = 0;
    // now compute the error
    for(size_t i = 0; i < n; i++) {
      if(!refPoints[i] || !currPoints[i])
	continue;
      const Vector6f& pref  = *refPoints[i];
      const Vector6f& pcurr = *currPoints[i];
      const Matrix6f& omega = *omegas[i];
      
      Vector6f e = remapPoint(Xnew, pref) - pcurr;
      float localError = e.transpose() * omega * e;
      if(localError > inlierThreshold)
            continue;
      error += localError;
      inliers++;
    }
    return inliers;
}

int pwn_iteration(float& error, Isometry3f& Xnew,
		  const Vector6f* const * refPoints,
		  const Vector6f* const * currPoints,
		  const Matrix6f* const * omegas,
		  size_t n,
		  const Isometry3f& X,
		  float inlierThreshold, /*inlier threshold*/
		  int minInliers){
  return _pwn_iteration(error, Xnew,
			refPoints, currPoints, omegas,
			n, X, inlierThreshold, minInliers);
}

int pwn_align(float& error,
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
  assert("this function is wrong" && 0);
  
    // allocate the reference images
    Vector6fPtrMatrix refImage(rows, cols);

    Vector6fPtrMatrix currImage(rows, cols);
    Matrix6fPtrMatrix currInformationImage(rows, cols);

    MatrixXf zBuffer(rows, cols);
    // construct the pointer map for the current image
    cloud2mat(currImage, currInformationImage,
              currPoints, currInformation,
              Isometry3f::Identity(), cameraMatrix, zBuffer);
    Isometry3f Xcurrent = X;
    int inliers = 0;
    int size = cols*rows;
    for(int i = 0; i < outerIterations; i++)
    {
        // remap the reference frame in the local one;
        cloud2mat(refImage, refPoints, Xcurrent, cameraMatrix, zBuffer);
        for(int j = 0; j < innerIterations; j++)
        {
            cerr << "\tinner iteration" << j << endl;
            float iterationError;
            Isometry3f _xNew;
            inliers =  _pwn_iteration(iterationError, _xNew,
                                     refImage.data(), currImage.data(), currInformationImage.data(), size,
                                     Xcurrent, inlierThreshold, minInliers);
	    error = iterationError;
            cerr << "\t\tinliers: " << inliers <<  " error:" << error << endl;
            if(inliers<minInliers)
            {
                return inliers;
                cerr << "\t\treject! " << endl;
            }
            Xcurrent = _xNew;
            cerr << "\t\taccept! " << endl;
            cerr << Xcurrent.linear() <<  " " << endl << Xcurrent.translation() << endl;
        }
    }
    if(inliers>minInliers)
        Xnew = Xcurrent;
    return inliers;
}
