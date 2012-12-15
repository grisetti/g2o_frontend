#include <iostream>
#include <string>
#include <qapplication.h>
#include "pwn_normals.h"
#include "pwn_cloud.h"
#include "pwn_solve.h"
#include "pwn_qglviewer_2x.h"
#include "pwn_utils.h"
#include "pwn_math.h"
#include "g2o/stuff/command_args.h"

using namespace std;

int main(int argc, char** argv){
  /************************************************************************************
   *                                                                                  *
   *  Read input.                                                                     *
   *                                                                                  *
   ************************************************************************************/
  // Set argumets parameters.
  g2o::CommandArgs arg;
  string nameImage0;
  float pointSize;
  float normalLength;
  float ellipsoidScale;
  float r;
  float d; 
  int minPoints;
  int step;
  arg.param("r", r, 0.1f, "radius of the ball to compute the normals, in world coordinates");
  arg.param("d", d, 0.0f, "radius of the ball to compute the normals, in image coordinates (if =0 normal is computed on integral image)");
  arg.param("mp", minPoints, 50, "min points to compiut de normal");
  arg.param("step", step, 2, "compute the normal each x rows and col");
  arg.param("ps", pointSize, 1.0f, "point size"); 
  arg.param("nl", normalLength, 0.01f, "normal length"); 
  arg.param("es", ellipsoidScale, 0.05f, "ellipsoid scale"); 
  arg.param("i0", nameImage0, "", "first image");
  arg.parseArgs(argc, argv);

  // Check if the name of the input image is valid.
  if (nameImage0.length()==0){
    cerr << "No image provided." << endl;
    return 0;
  }
  
  /************************************************************************************
   *                                                                                  *
   *  Read depth image.                                                               *
   *                                                                                  *
   ************************************************************************************/
  // Read image.
  MatrixXus image0;
  FILE* file;
  file = fopen(nameImage0.c_str(), "rb");
  if (!readPgm(image0, file)){
    cout << "Error while reading first depth image." << endl;
    exit(-1);
  }
  fclose(file);

  // Get rows and columns of the input images
  int rows = image0.rows();
  int cols = image0.cols();

  /************************************************************************************
   *                                                                                  *
   *  Compute 3D points from the depth images.                                        *
   *                                                                                  *
   ************************************************************************************/
  // Create camera matrix.
  Matrix3f cameraMatrix;
  cameraMatrix << 525.0f, 0.0f, 319.5f,
                  0.0f, 525.0f, 239.5f,
                  0.0f, 0.0f, 1.0f;

  // Transform depth images to float type.
  MatrixXf depth0(rows, cols), depth1(rows, cols);
  img2depth(depth0, image0);
  img2depth(depth1, image0);

  // Create 3D point clouds with normals.
  Vector6fVector cloud0, cloud1;
  depth2cloud(cloud0, depth0, cameraMatrix);
  depth2cloud(cloud1, depth1, cameraMatrix);

  /************************************************************************************
   *                                                                                  *
   *  Compute normals and curvature of the 3D points.                                 *
   *                                                                                  *
   ************************************************************************************/
  // Create matrices of pointers.
  MatrixXf curvature0(rows, cols), curvature1(rows, cols);
  MatrixXf zBuffer(rows, cols);
  Vector6fPtrMatrix cloud0Ptr(rows, cols), cloud1Ptr(rows, cols);
  CovarianceSVDPtrMatrix svd0Ptr(rows, cols), svd1Ptr(rows, cols);
  Matrix6fVector _dummyOmega;
  Matrix6fPtrMatrix _dummyOmegaMat(0,0);
  CovarianceSVDVector svd0(cloud0.size());
  CovarianceSVDVector svd1(cloud1.size());
  cloud2mat(cloud0Ptr,
	    _dummyOmegaMat,
	    svd0Ptr,
            cloud0,
	    _dummyOmega,
	    svd0,
            Isometry3f::Identity(), 
	    cameraMatrix,
	    zBuffer);

  cloud2mat(cloud1Ptr,
	    _dummyOmegaMat,
	    svd1Ptr,
            cloud1,
	    _dummyOmega,
	    svd1,
            Isometry3f::Identity(), 
	    cameraMatrix,
	    zBuffer);
 
  // Compute normals.
  cerr << "computing normals0... ";
  computeNormals(cloud0Ptr, curvature0, svd0Ptr, cameraMatrix, r, d, step, minPoints);
  cerr << "done !" << endl;

  cerr << "computing normals1... ";
  computeNormals(cloud1Ptr, curvature1, svd1Ptr, cameraMatrix, r, d, step, minPoints);
  cerr << "done !" << endl;

  /************************************************************************************
   *                                                                                  *
   *  Compute omega matrices.                                                         *
   *                                                                                  *
   ************************************************************************************/
  // Compute 6x6 omega matrices.
  Matrix6fVector omegas1, omegas0;
  cerr << "computing omega0... ";
  svd2omega(omegas0, svd0);
  cerr << "done !" << endl;

  cerr << "computing omega1... ";
  svd2omega(omegas1, svd1);
  cerr << "done !" << endl;

  /************************************************************************************
   *                                                                                  *
   *  Apply perturbation to first cloud.                                              *
   *                                                                                  *
   ************************************************************************************/
  // Create perturbation.
  Vector6f v;
  v << 0.0f, 0.0f, -0.2f, 0.0f, 0.00f, 0.00f;
  Isometry3f T = v2t(v);
  
  // Apply perturbation to cloud0.
  Vector6fVector cloud0Pert;
  Matrix6fVector omegas0Pert;
  CovarianceSVDVector svd0Pert;
  remapCloud(cloud0Pert,
	     omegas0Pert,
	     svd0Pert,
	     T,
	     cloud0,
	     omegas0,
	     svd0);

  // Computes pointers matrices for the perturbed cloud.
  Vector6fPtrMatrix cloud0PtrPert(rows, cols);
  CovarianceSVDPtrMatrix svd0PtrPert(rows, cols);
  Matrix6fPtrMatrix omegas0PtrPert(rows, cols);
  cloud2mat(cloud0PtrPert,
	    omegas0PtrPert,
	    svd0PtrPert,
	    cloud0Pert,
	    omegas0Pert,
	    svd0Pert,
	    Isometry3f::Identity(),
	    cameraMatrix,
	    zBuffer);

  /************************************************************************************
   *                                                                                  *
   *  Scale down the clouds.                                                          *
   *                                                                                  *
   ************************************************************************************/
  // Scale factors.
  float scale = 1.0f / 4.0f;
  Matrix3f cameraMatrixScaled = cameraMatrix;
  cameraMatrixScaled.block<2,3>(0, 0) *= scale;
  int _r = ((float)image0.rows()*scale);
  int _c = ((float)image0.cols()*scale);

  // Create scaled clouds variables.
  Vector6fPtrMatrix cloud0PtrPertScaled(_r, _c);
  Matrix6fPtrMatrix omegas0PtrPertScaled(_r, _c);
  CovarianceSVDPtrMatrix svd0PtrPertScaled(_r, _c);
  Vector6fPtrMatrix cloud1PtrScaled(_r, _c);
  Matrix6fPtrMatrix omega1PtrScaled(_r, _c);
  CovarianceSVDPtrMatrix svd1PtrScaled(_r, _c);

  // Scale cloud1.
  cloud2mat(cloud1PtrScaled,
	    omega1PtrScaled,
	    svd1PtrScaled,
	    cloud1,
	    omegas1,
	    svd1,
	    Isometry3f::Identity(),
	    cameraMatrixScaled,
	    zBuffer);
  
  // Scale cloud0.
  cloud2mat(cloud0PtrPertScaled,
	    omegas0PtrPertScaled,
	    svd0PtrPertScaled,
	    cloud0Pert,
	    omegas0Pert,
	    svd0Pert,
	    Isometry3f::Identity(),
	    cameraMatrixScaled,
	    zBuffer);

  /************************************************************************************
   *                                                                                  *
   *  Compute correspondances                                                         *
   *                                                                                  *
   ************************************************************************************/
  // Thresholds for normals matching.
  float curvatureThreshold = 0.02;
  float normalThreshold = M_PI/6;
  
  // Compute correspondances.
  CorrVector correspondences;
  Matrix6fPtrMatrix corrOmegas0Pert(_r, _c);
  Vector6fPtrMatrix corrCloud0Pert(_r, _c);
  Vector6fPtrMatrix corrCloud1(_r, _c);
  correspondences.clear();
  int corrFound = 0;
  for (int i =0; i<cloud1PtrScaled.cols(); i++){
    for (int j =0; j<cloud1PtrScaled.rows(); j++){
      if (!cloud0PtrPertScaled(j, i) || !cloud1PtrScaled(j, i))
	continue;
      Vector6f& p0 = *(cloud0PtrPertScaled(j, i));
      Vector6f& p1 = *(cloud1PtrScaled(j, i));
      if (p0.tail<3>().squaredNorm()<=1e-3 || p1.tail<3>().squaredNorm()<=1e-3)
	continue;
      SVDMatrix3f& svd0 = *svd0PtrPertScaled(j,i);
      SVDMatrix3f& svd1 = *svd1PtrScaled(j,i);
      float c0 = svd0.curvature();
      float c1 = svd1.curvature();
      if (c0>curvatureThreshold || c1>curvatureThreshold)
	continue;
      Vector6f p0Remapped = p0;//remapPoint(T1_0, p0);
      if (p0Remapped.tail<3>().dot(p1.tail<3>())<normalThreshold)
	continue;
      correspondences.push_back(Corr(cloud0PtrPertScaled(j, i), cloud1PtrScaled(j, i)));
      corrCloud0Pert(j, i) = &p0;
      corrCloud1(j, i) = &p1;
      corrOmegas0Pert(j, i) = omegas0PtrPertScaled(j, i);
      corrFound++;
    }
  }
  cerr << "found " << corrFound << " correspondences" << endl;

  /************************************************************************************
   *                                                                                  *
   *  Compute transformation                                                          *
   *                                                                                  *
   ************************************************************************************/
  // Run optimization algorithm.
  int size = _r*_c;
  Isometry3f result = Isometry3f::Identity();
  float error = 0.0f;
  for(int i=0; i<10; i++){
    clock_t start = getMilliSecs();
    int inl = pwn_iteration(error, result,
			    corrCloud0Pert.data(),
			    corrCloud1.data(),
			    corrOmegas0Pert.data(),
			    size,
			    Isometry3f::Identity(),
			    numeric_limits<float>::max(),
			    0);
    cout << "i: " << i << " " << inl << " " << error << " " << endl << t2v(result) << endl;
    cout << "Time elapsed: " << getMilliSecs() - start << " ms" << endl;
    cout << "---------------------------------------------------------------" << endl;    
  }

  /************************************************************************************
   *                                                                                  *
   *  Compute first cloud transformed.                                                *
   *                                                                                  *
   ************************************************************************************/
  for(size_t i=0; i<cloud0Pert.size(); i++){
    Vector6f pT = cloud0Pert[i];
    Vector6f &point = cloud0Pert[i];
    point = remapPoint(result, pT);
  }

  /************************************************************************************
   *                                                                                  *
   *  Draw stuffs.                                                                    *
   *                                                                                  *
   ************************************************************************************/
  // Create viever application object.
  QApplication application(argc, argv);
  
  // Create and set viewer oprions.
  PWNQGLViewer2X viewer;
  viewer.setPointSize(pointSize);
  viewer.setNormalLength(normalLength);
  viewer.setEllipsoidScale(ellipsoidScale);
  viewer.setPoints0(&cloud0Pert);
  viewer.setPoints1(&cloud1);
  viewer.setEllipsoids0(&svd0Pert);
  viewer.setEllipsoids1(&svd1);
  viewer.setCorrVector(&correspondences);
  viewer.setWindowTitle("Viewer");

  // Make the viewer window visible on screen.
  viewer.show();

  // Run main loop.
  return application.exec();
    
  return 0;
}
