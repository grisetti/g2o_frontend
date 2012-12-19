#include <qapplication.h>
#include <iostream>
#include <string>
#include "dm_main_window.h"
#include "g2o/stuff/command_args.h"
#include "dm_qglviewer.h"
#include "drawable.h"
#include "drawable_points.h"
#include "drawable_normals.h"
#include "drawable_covariances.h"
#include "drawable_correspondences.h"
#include "gl_parameter.h"
#include "gl_parameter_points.h"
#include "gl_parameter_normals.h"
#include "gl_parameter_covariances.h"
#include "gl_parameter_correspondences.h"
#include "../pwn/pwn_normals.h"
#include "../pwn/pwn_cloud.h"
#include "../pwn/pwn_solve.h"
#include "../pwn/pwn_utils.h"
#include "../pwn/pwn_math.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  /**** REGISTRATION COMPUTING ****/
  clock_t begin = getMilliSecs();
  g2o::CommandArgs arg;
  string imageName0;
  string imageName1;
  float r;
  float d; 
  int minPoints;
  int step;
  float pointSize;
  float normalLength;
  float ellipsoidScale;
  float scale;
  arg.param("r", r, 0.1f, "radius of the ball to compute the normals, in world coordinates");
  arg.param("d", d, 0.0f, "radius of the ball to compute the normals, in image coordinates (if =0 normal is computed on integral image)");
  arg.param("mp", minPoints, 50, "min points to compiut de normal");
  arg.param("step", step, 2, "compute the normal each x rows and col");
  arg.param("scale", scale, 0.25f, "scale of the range image"); 
  arg.param("ps", pointSize, 1.0f, "point size"); 
  arg.param("nl", normalLength, 0.01f, "normal length"); 
  arg.param("es", ellipsoidScale, 0.05f, "ellipsoid scale"); 
  
  arg.paramLeftOver("image0", imageName0 , "", "image0", true);
  arg.paramLeftOver("image1", imageName1 , "", "image1", true);
  arg.parseArgs(argc, argv);
  
if (imageName0.length() == 0) {
    cerr << "no image provided" << endl;
    return 0;
  }
  if (imageName1.length() == 0) {
    cerr << "no image provided" << endl;
    return 0;
  }

  // Create camera matrix.
  Matrix3f cameraMatrix;
  cameraMatrix << 525.0f, 0.0f, 319.5f,
                  0.0f, 525.0f, 239.5f,
                  0.0f, 0.0f, 1.0f;

  /************************************************************************************
   *                                                                                  *
   *  Read depth images.                                                              *
   *                                                                                  *
   ************************************************************************************/
  MatrixXus image0, image1;
  FILE* file;
  file = fopen(imageName0.c_str(), "rb");
  if (!readPgm(image0, file)) {
    cout << "Error while reading first depth image." << endl;
    exit(-1);
  }
  fclose(file);
  file = fopen(imageName1.c_str(), "rb");
  if (!readPgm(image1, file)) {
    cout << "Error while reading first depth image." << endl;
    exit(-1);
  }
  fclose(file);

  // Get rows and columns for the input images.
  int rows = image0.rows();
  int cols = image0.cols();

  /************************************************************************************
   *                                                                                  *
   *  Compute 3D points from the depth images.                                        *
   *                                                                                  *
   ************************************************************************************/
  // Cast images to float type.
  MatrixXf depth0(rows, cols), depth1(rows, cols);
  img2depth(depth0, image0);
  img2depth(depth1, image1);

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
  Matrix6fVector omega0, omega1;
  Matrix6fPtrMatrix omega0Ptr(0, 0), omega1Ptr(0, 0);
  CovarianceSVDVector svd0(cloud0.size()), svd1(cloud1.size());
  cloud2mat(cloud0Ptr,
	    omega0Ptr,
	    svd0Ptr,
            cloud0,
	    omega0,
	    svd0,
            Isometry3f::Identity(), cameraMatrix,
	    zBuffer);
  cloud2mat(cloud1Ptr,
	    omega1Ptr,
	    svd1Ptr,
            cloud1,
	    omega1,
	    svd1,
            Isometry3f::Identity(), cameraMatrix,
	    zBuffer);
  
  // Compute normals.
  cerr << "computing normals0... ";
  computeNormals(cloud0Ptr, curvature0, svd0Ptr, cameraMatrix, r, d, step, minPoints);
  svd2omega(omega0, svd0);
  cerr << "done !" << endl;
  cerr << "computing normals1... ";
  computeNormals(cloud1Ptr, curvature1, svd1Ptr, cameraMatrix, r, d, step, minPoints);
  svd2omega(omega1, svd1);
  cerr << "done !" << endl;
  
  /************************************************************************************
   *                                                                                  *
   *  Scale down the clouds.                                                          *
   *                                                                                  *
   ************************************************************************************/
  // Scale factors.
  Matrix3f cameraMatrixScaled = cameraMatrix;
  cameraMatrixScaled.block<2,3>(0, 0) *= scale;
  int _r=((float)image0.rows()*scale);
  int _c=((float)image0.cols()*scale);

  // Create scaled clouds variables.
  Vector6fPtrMatrix cloud0PtrScaled(_r, _c);
  Matrix6fPtrMatrix omega0PtrScaled(_r, _c);
  CovarianceSVDPtrMatrix svd0PtrScaled(_r, _c);
  Vector6fPtrMatrix cloud1PtrScaled(_r, _c);
  Matrix6fPtrMatrix omega1PtrScaled(_r, _c);
  CovarianceSVDPtrMatrix svd1PtrScaled(_r, _c);
  Matrix6fPtrMatrix corrOmegas1(_r, _c);
  Vector6fPtrMatrix corrP0(_r,_c);
  Vector6fPtrMatrix corrP1(_r,_c);

  // Scale cloud1.
  cloud2mat(cloud1PtrScaled,
	    omega1PtrScaled,
	    svd1PtrScaled,
	    cloud1,
	    omega1,
	    svd1,
	    Isometry3f::Identity(),
	    cameraMatrixScaled,
	    zBuffer);
  Isometry3f T1_0 =Isometry3f::Identity();

  /************************************************************************************
   *                                                                                  *
   *  Compute registration.                                                           *
   *                                                                                  *
   ************************************************************************************/
  int iterations=10;
  CorrespondenceVector correspondences;
  Vector6fVector  cloudT;
  CovarianceSVDVector svdT;
  Matrix6fVector omegaT;

  // Thresholds for normals matching.
  float curvatureThreshold=0.02;
  float normalThreshold = M_PI/6;
  for (int k = 0; k < iterations; k++) {
    clock_t kstart = getMilliSecs();
    
    /**********************************************************************************
     *                                                                                *
     *  Update cloud.                                                                 *
     *                                                                                *
     **********************************************************************************/
    omega0PtrScaled.fill(0);
    cloud0PtrScaled.fill(0);
    svd0PtrScaled.fill(0);
    cloud2mat(cloud0PtrScaled,
	      omega0PtrScaled,
	      svd0PtrScaled,
	      cloud0,
	      omega0,
	      svd0,
	      T1_0.inverse(),
	      cameraMatrixScaled,
	      zBuffer);

    corrOmegas1.fill(0);
    corrP0.fill(0);
    corrP1.fill(0);

    /**********************************************************************************
     *                                                                                *
     *  Compute correspondances.                                                      *
     *                                                                                *
     **********************************************************************************/
    correspondences.clear();
    int corrFound = 0;
    for (int i = 0; i < cloud1PtrScaled.cols(); i++) {
      for (int j = 0; j < cloud1PtrScaled.rows(); j++) {
	if (! cloud0PtrScaled(j, i) || !cloud1PtrScaled(j, i))
	  continue;
	Vector6f& p0 = *(cloud0PtrScaled(j, i));
	Vector6f& p1 = *(cloud1PtrScaled(j, i));
	if (p0.tail<3>().squaredNorm() <= 1e-3 || p1.tail<3>().squaredNorm() <= 1e-3)
	  continue;
	SVDMatrix3f& svd0 = *svd0PtrScaled(j, i);
	SVDMatrix3f& svd1 = *svd1PtrScaled(j, i);

	float c0 = svd0.curvature();
	float c1 = svd1.curvature();
      
	if (c0 > curvatureThreshold || c1 > curvatureThreshold)
	  continue;
	Vector6f p0Remapped = remapPoint(T1_0, p0);
	Vector3f n0Remapped = p0Remapped.tail<3>();
	Vector3f n1 = p1.tail<3>();

	if (n0Remapped.dot(n1) < normalThreshold)
	  continue;
	correspondences.push_back(Correspondence(cloud0PtrScaled(j, i), cloud1PtrScaled(j, i)));	
	corrP0(j, i) = &p0;
	corrP1(j, i) = &p1;
	corrOmegas1(j, i) = omega0PtrScaled(j, i);
	corrFound ++;
      }
    }
    cerr << "found " << corrFound << " correspondences" << endl;
    
    /************************************************************************************
     *                                                                                  *
     *  Compute transformation.                                                         *
     *                                                                                  *
     ************************************************************************************/
    // Run optimization algorithm.
    Isometry3f result = Isometry3f::Identity();
    float error = 0.0f;
    int size = _r*_c;
    clock_t start = getMilliSecs();
    int inl; 
    for(int i=0; i<10; i++){
      inl = pwn_iteration(error, result,
			  corrP0.data(),
			  corrP1.data(),
			  corrOmegas1.data(),
			  size,
			  T1_0,
			  numeric_limits<float>::max(),
			  0);
      T1_0 = result;
    }
    cout << "k: " << k << " " << inl << " " << error << " " << endl;
    cout << "Time optimization : " << getMilliSecs() - start << " ms" << endl;
    cout << "Time global iteration: " << getMilliSecs() - kstart << " ms" << endl;
    cout << "---------------------------------------------------------------" << endl;  
  }
  cerr << "Total time needed to compute the whole stuffs: " << getMilliSecs() - begin << " ms" << endl;

  /**** REGISTRATION VISUALIZATION****/
  QApplication qApplication(argc, argv);
  DMMainWindow dmMW;
  dmMW.show();
  int *stepViewer = 0;
  float *pointsViewer = 0, *normalsViewer = 0, *covariancesViewer = 0, *correspondencesViewer = 0;
  GLParameterPoints *p0Param = new GLParameterPoints();
  GLParameterPoints *p1Param = new GLParameterPoints();
  GLParameterNormals *n0Param = new GLParameterNormals();
  GLParameterNormals *n1Param = new GLParameterNormals();
  GLParameterCovariances *c0Param = new GLParameterCovariances();
  GLParameterCovariances *c1Param = new GLParameterCovariances();
  GLParameterCorrespondences *corrParam = new GLParameterCorrespondences();
  DrawablePoints* dp0 = new DrawablePoints(Isometry3f::Identity(), (GLParameter*)p0Param, 1, &cloud0);
  DrawablePoints* dp1 = new DrawablePoints(T1_0.inverse(), (GLParameter*)p1Param, 1, &cloud1);
  DrawableNormals *dn0 = new DrawableNormals(Isometry3f::Identity(), (GLParameter*)n0Param, 1, &cloud0);
  DrawableNormals *dn1 = new DrawableNormals(T1_0.inverse(), (GLParameter*)n0Param, 1, &cloud1);
  DrawableCovariances *dc0 = new DrawableCovariances(Isometry3f::Identity(), (GLParameter*)c0Param, 1, &svd0);
  DrawableCovariances *dc1 = new DrawableCovariances(T1_0.inverse(), (GLParameter*)c1Param, 1, &svd1);
  DrawableCorrespondences* dcorr = new DrawableCorrespondences(T1_0.inverse(), (GLParameter*)corrParam, 1, &correspondences);
  while (true) {
    qApplication.processEvents();
    
    // Update state variables value.
    stepViewer = dmMW.step();
    pointsViewer = dmMW.points();
    normalsViewer = dmMW.normals();
    covariancesViewer = dmMW.covariances();
    correspondencesViewer = dmMW.correspondences();
    
    // Checking state variable value.
    dmMW.viewer_3d->clearDrawableList();
    if (pointsViewer[0]) {
      p0Param->setColor(Vector4f(1.0f, 0.0f, 0.0f, 0.5f));
      p0Param->setPointSize(pointsViewer[1]);
      p1Param->setPointSize(pointsViewer[1]);
      if (stepViewer[0]) {
	dp0->setStep(stepViewer[1]);
	dp1->setStep(stepViewer[1]);
      }
      dmMW.viewer_3d->addDrawable((Drawable*)dp0);
      dmMW.viewer_3d->addDrawable((Drawable*)dp1);
    }
    if (normalsViewer[0]) {
      n0Param->setNormalLength(normalsViewer[1]);
      n1Param->setNormalLength(normalsViewer[1]);
      if (stepViewer[0]) {
	dn0->setStep(stepViewer[1]);
	dn1->setStep(stepViewer[1]);
      }
      dmMW.viewer_3d->addDrawable((Drawable*)dn0);
      dmMW.viewer_3d->addDrawable((Drawable*)dn1);
    }
    if(covariancesViewer[0]) {
      c0Param->setEllipsoidScale(covariancesViewer[1]);
      c1Param->setEllipsoidScale(covariancesViewer[1]);
      if (stepViewer[0]) {
	dc0->setStep(stepViewer[1]);
	dc1->setStep(stepViewer[1]);
      }
      dmMW.viewer_3d->addDrawable((Drawable*)dc0);
      dmMW.viewer_3d->addDrawable((Drawable*)dc1);
    }
    if(correspondencesViewer[0]) {
      corrParam->setLineWidth(correspondencesViewer[1]);
      if (stepViewer[0])
	dcorr->setStep(stepViewer[1]);	
      dmMW.viewer_3d->addDrawable((Drawable*)dcorr);
    }
    dmMW.viewer_3d->updateGL();

    usleep(10000);
  }
  dmMW.viewer_3d->clearDrawableList();
 
  delete(p0Param);
  delete(p1Param);
  delete(dp0);
  delete(dp1);

  return 0;
}
