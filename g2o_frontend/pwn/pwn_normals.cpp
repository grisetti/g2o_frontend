#include "pwn_normals.h"
#include <math.h>
#include <Eigen/SVD>

// Computes normal of the input point considering a set of the points around it.
void computeIterativeCovariance(Matrix3f &covariance, 
				const Vector6fPtrMatrix &points, const Vector2i &imgPoint, 
				char **visitedMask, Vector2iVector &visited, 
				float rw2, int ri2)
{/*
  // Check if point is not (0.0, 0.0, 0.0).
  Vector2i p0i = imgPoint;
  Vector6f *p0Ptr = points(p0i[0], p0i[1]);
  if (!p0Ptr)
    return;
  
  // Fill the queue with around point if they satisfy the distance constraints.
  Vector3f p0 = (*p0Ptr).head<3>();
  Vector2iQueue q;
  q.push(p0i);
  visitedMask[p0i[0]][p0i[1]] = 1;
  visited.push_back(p0i);
  while (!q.empty()){
    Vector2i pki = q.front();
    q.pop();
    for(int xx=-1; xx<1; xx++){
      for(int yy=-1; yy<1; yy++){
	Vector2i pNi(pki[0] + xx, pki[1] + yy);
	Vector6f pNPtr = points(pNi[0], pNi[1]);
	if (!pNPtr)
	  continue;
	Vector3f pN = (*pNPtr).head<3>();
	if (pNi[0]<0 || pNi[1]<0 || pNi[0]>=points.rows() || pNi[1]>=points.cols())
	  continue;
       	if (xx==0 && yy==0)
	  continue;
	if (visitedMask[pNi[0]][pNi[1]])
	  continue;
	if((pN - p0).squaredNorm()>rw2 || (pNi - p0i).squaredNorm()>ri2)
	  continue;
	q.push(pNi);
	visitedMask[pNi[0]][pNi[1]] = 1;
	visited.push_back(pNi);
      }
    }
  }

  size_t minPoints = 3;
  if (visited.size()<minPoints)
    return;

  // Compute the mean and clear the mask.
  Vector3f mean = Vector3f::Zero();
  for(size_t k=0; k<visited.size(); k++){
    Vector2i pNi = visited[k];
    Vector3f pN = (*points(pNi[0], pNi[1])).head<3>();
    mean += pN;
    visitedMask[pNi[0]][pNi[1]] = 0;
  }
  mean *= (1.0f / (float)visited.size());

  // Compute covariance matrix.
  covariance = Matrix3f::Zero();
  for(size_t k=0; k<visited.size(); k++){
    Vector2i pNi = visited[k];
    Vector3f pN = (*points(pNi[0], pNi[1])).head<3>() - mean;
    covariance += (pN * pN.transpose());
  }
  covariance *= (1.0f / (float)visited.size());

  // Clear the list of point.
  visited.clear();*/
}

// Computes curvature and normals using integral images.
void computeIntegralCovariance(Matrix3f &covariance, 
			       const Vector6fPtrMatrix &points, const Vector2i &imgPoint,  
			       MatrixVector9f &integImage, MatrixXi &integMask, 
			       const Matrix3f &cameraMatrix, float r)
{
  Vector2i p0i = imgPoint;
  Vector6f *p0Ptr = points(p0i[0], p0i[1]);
  
  // Check if it is a valid point.
  if (!p0Ptr)
    return;
  
  Vector3f p0 = (*p0Ptr).head<3>();
  // Get the side of the square to analyze.
  int offset = 0;
  getOffset(offset, 
	    p0, cameraMatrix,
	    r, p0i[1]);

  // Get region and mask.
  Vector9f region;
  int mask = 0;
  int numValidPoint = 0;
  getRegion(region, mask, numValidPoint, 
	    integImage, integMask, 
	    offset, p0i);

  // Check if there are enough points.
  int minPoints = 3;
  if(numValidPoint < minPoints)
    return;
  region *= (1.0f / (float)numValidPoint);

  // Compute covariance matrix.
  covariance = Matrix3f::Zero();
  covariance(0, 0) = region[0] - region[6] * region[6];
  covariance(0, 1) = region[1] - region[6] * region[7];
  covariance(0, 2) = region[2] - region[6] * region[8];
  covariance(1, 1) = region[3] - region[7] * region[7];
  covariance(1, 2) = region[4] - region[7] * region[8];
  covariance(2, 2) = region[5] - region[8] * region[8];
  covariance(1, 0) = covariance(0, 1);
  covariance(2, 0) = covariance(0, 2);
  covariance(2, 1) = covariance(1, 2);
}

void computeNormalAndCurvature(Vector3f normal, float &curvature, 
			       const Matrix3f covariance)
{
  // Extract eigenvectors and eigenvalues.
  SelfAdjointEigenSolver<Matrix3f> eigenSolver(covariance);
  Matrix3f eigenVectors = eigenSolver.eigenvectors();
  Vector3f eigenValues = eigenSolver.eigenvalues().normalized();
  Vector3f minEigenVector(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2, 0));
  
  // Check round error.
  if(eigenValues[0] < 0)
    eigenValues[0] = 0.0f;

  // Extract normal to the point.
  normal = Vector3f(minEigenVector[0], minEigenVector[1], minEigenVector[2]);
   
  // Compute curvature.
  curvature = eigenValues[0] / (eigenValues[0] + eigenValues[1] + eigenValues[2]);    
}

// Compute integral images for the matrix diven in input.
void computeIntegralImage(MatrixVector9f &integImage, MatrixXi &integMask, 
			  const Vector6fPtrMatrix &points)
{
  VectorXf acc(9);
  int maskAcc = 0;
  for (int i=0; i<points.rows(); i++){
    for (int j=0; j<points.cols(); j++){
      Vector6f *pointPtr = points(i, j);
      Vector3f point(0.0f, 0.0f, 0.0f);
      if (pointPtr)
	point = (*pointPtr).head<3>();
      if (j==0){
	acc[0] = point[0] * point[0];
	acc[1] = point[0] * point[1];
	acc[2] = point[0] * point[2];
	acc[3] = point[1] * point[1];
	acc[4] = point[1] * point[2]; 
	acc[5] = point[2] * point[2]; 
	acc[6] = point[0];
	acc[7] = point[1];
	acc[8] = point[2];
	if (point==Vector3f::Zero())
	  maskAcc = 1;
	else
	  maskAcc = 0;
      }
      else{
	acc[0] += point[0] * point[0];
	acc[1] += point[0] * point[1];
	acc[2] += point[0] * point[2];
	acc[3] += point[1] * point[1];
	acc[4] += point[1] * point[2];
	acc[5] += point[2] * point[2];
	acc[6] += point[0];
	acc[7] += point[1];
	acc[8] += point[2];
	if (point==Vector3f::Zero())
	  maskAcc += 1;
      }
      integImage(i, j) = acc;
      integMask(i, j) = maskAcc;
    }
  }

  for (int j=0; j<points.cols(); j++){
    for (int i=0; i<points.rows(); i++){
      if (i==0){
	maskAcc = integMask(i, j);
	acc = integImage(i, j);
      }
      else{
	maskAcc += integMask(i, j);
	acc += integImage(i, j);
      }
      integImage(i, j) = acc;
      integMask(i, j) = maskAcc;
    }
  }
}

// Computes the normals of 3D points.
void computeNormals(Vector6fPtrMatrix &cloud, MatrixXf &curvature, 
		    const Matrix3f &cameraMatrix, float r, float d)
{
  /*
  // Initialize mask putting 1 where there are (0.0, 0.0, 0.0) points.
  char **visitedMask = new char*[cloud.rows()];
  for (int i=0; i<cloud.rows(); i++){
    visitedMask[i] = new char[cloud.cols()];
    for (int j=0; j<cloud.cols(); j++){
      if(cloud(i, j)[0]==0.0f && cloud(i, j)[1]==0.0f && cloud(i, j)[2]==0.0f)
        visitedMask[i][j] = 1;
      else
        visitedMask[i][j] = 0;
    }
  }
  Vector2iVector visited;
  */

  // Compute integral image.
  MatrixXi integralImageMask(cloud.rows(), cloud.cols());
  MatrixVector9f integralImage(cloud.rows(), cloud.cols());
  computeIntegralImage(integralImage, integralImageMask, 
		       cloud);

  // Compute normals.
  for (int i=0; i<cloud.rows(); i++){
    for (int j=0; j<cloud.cols(); j++){
      Vector2i pointCoord(i, j);
      Matrix3f covariance = Matrix3f::Zero();
      computeIntegralCovariance(covariance, 
				cloud, pointCoord, 
				integralImage, integralImageMask,
				cameraMatrix, r);
      Vector3f norm = Vector3f::Zero();
      float curv = 0;
      if (covariance!=Matrix3f::Zero()){
	computeNormalAndCurvature(norm, curv, 
				  covariance);
	(*cloud(i, j))[3] = norm[0];
	(*cloud(i, j))[4] = norm[1];
	(*cloud(i, j))[5] = norm[2];
	curvature(i, j) = curv;
      }
    }
  }

  /*
  // Delete mask.
  for(int i = 0; i < cloud.rows(); i++)
    delete[] visitedMask[i];
  delete[] visitedMask;
  */
}

// Return the half side of the square of image points to analize.
void getOffset(int &offset, 
	       Vector3f point, const Matrix3f &cameraMatrix, 
	       float r, int col)
{
  point += Vector3f(r, r, 0.0f);
  Vector3f coord = cameraMatrix * point;
  coord[0] *= 1.0f / coord[2];
  offset = abs(col - (int)coord[0]);
}

// Computes region value of an integral image and it's mask value for acc algorithm.
void getRegion(Vector9f &region, int &regionMask, int &numValidPoints,
	       const MatrixVector9f &integImage, const MatrixXi &integMask, 
	       int offset, Vector2i p0i)
{
  // Check if offset does not exceed image boundaries
  int leftOffsetX = offset;
  int rightOffsetX = offset;
  int leftOffsetY = offset;
  int rightOffsetY = offset;
  if (p0i[0] - offset<0)
    leftOffsetX = offset + (p0i[0] - offset);
  if (p0i[0] + offset>integImage.rows() - 1)
    rightOffsetX = offset - (p0i[0] + offset - (integImage.rows() - 1));
  if (p0i[1] - offset<0)
    leftOffsetY = offset + (p0i[1] - offset);
  if (p0i[1] + offset>integImage.cols() - 1)
    rightOffsetY = offset - (p0i[1] + offset - (integImage.cols() - 1));

  // Get integralImage region.
  VectorXf Tot(9), A(9), AB(9), AC(9), D(9);
  Tot = integImage(p0i[0] + rightOffsetX, p0i[1] + rightOffsetY);
  if (p0i[0] - leftOffsetX - 1<0 || p0i[1] - leftOffsetY - 1<0)
    A.setZero(9);
  else
    A = integImage(p0i[0] - leftOffsetX - 1, p0i[1] - leftOffsetY - 1);
  if (p0i[0] - leftOffsetX - 1<0)
    AB.setZero(9);
  else
    AB = integImage(p0i[0] - leftOffsetX - 1, p0i[1] + rightOffsetY);
  if (p0i[1] - leftOffsetY - 1<0)
    AC.setZero(9);
  else
    AC = integImage(p0i[0] + rightOffsetX, p0i[1] - leftOffsetY - 1);
  D = Tot - AB - AC + A;

  // Get integralMask region.
  int maskTot, maskA, maskAB, maskAC, maskD;
  maskTot = integMask(p0i[0] + rightOffsetX, p0i[1] + rightOffsetY);
  if (p0i[0] - leftOffsetX - 1<0 || p0i[1] - leftOffsetY - 1<0)
    maskA = 0;
  else
    maskA = integMask(p0i[0] - leftOffsetX - 1, p0i[1] - leftOffsetY - 1);
  if (p0i[0] - leftOffsetX - 1<0)
    maskAB = 0;
  else
    maskAB = integMask(p0i[0] - leftOffsetX - 1, p0i[1] + rightOffsetY);
  if (p0i[1] - leftOffsetY - 1<0)
    maskAC = 0;
  else
    maskAC = integMask(p0i[0] + rightOffsetX, p0i[1] - leftOffsetY - 1);
  maskD = maskTot - maskAB - maskAC + maskA;

  int numPoint = (leftOffsetX + rightOffsetX + 1) * (leftOffsetY + rightOffsetY + 1);

  numValidPoints = numPoint - maskD;
  region = D;
  regionMask = maskD;
}
