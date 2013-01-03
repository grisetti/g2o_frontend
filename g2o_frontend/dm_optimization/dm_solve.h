#ifndef DM_SOLVE_H
#define DM_SOLVE_H

#include "dm_cloud.h"

int pwn_iteration(float& error, Eigen::Isometry3f& Xnew,
		   const Vector6f* const * refPoints,
		   const Vector6f* const * currPoints,
		   const Matrix6f* const * omegas,
		   size_t n,
		  const Eigen::Isometry3f& X,
		   float inlierThreshold, /*inlier threshold*/
		  int minInliers,
		  float lambda=0);

int pwn_solveLinear(float& error,
		    Eigen::Isometry3f& Xnew,
		    const Vector6f* const * refPoints,
		    const Vector6f* const * currPoints,
		    const Matrix6f* const * omegas,
		    size_t n,
		    const Eigen::Isometry3f& X,
		    float inlierThreshold, 
		    int minInliers,
		    float lambda=0);

int pwn_align(float& error,
	      Eigen::Isometry3f& Xnew,
	      Vector6fVector& refPoints,
	      Vector6fVector& currPoints,
	      Matrix6fVector& currInformation,
	      //input
	      Eigen::Isometry3f& X,
	      Eigen::Matrix3f& cameraMatrix,
	      int rows,
	      int cols,
	      float inlierThreshold /*inlier threshold*/,
	      int minInliers,
	      int innerIterations,
	      int outerIterations);
 
#endif
