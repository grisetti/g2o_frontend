#include <iostream>
#include <string>
#include <qapplication.h>
#include "pwn_normals.h"
#include "pwn_cloud.h"
#include "pwn_solve.h"
#include "pwn_qglviewer.h"
#include "pwn_utils.h"
#include "pwn_math.h"
#include "g2o/stuff/command_args.h"

using namespace std;
using namespace Eigen;

// struct PointsWithNormalPyramidLevel{
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   PointsWithNormalPyramidLevel(int width_, int height_){
//     _width = width_;
//     _height = height_;
//   }
//   int width() const {return _width;}
//   int height() const {return _height;}
//   void setWidth(int width_) {_width = width_;}
//   void setHeight(int height_) {_height = height_;}
//   Vector6fVector points;
//   CovarianceSVDVector svdCov;
//   Matrix6fVector omegas;
//   int _width;
//   int _height;
// };

// void constructPyramid(PointsWithNormalPyramidLevel & newLevel, PointsWithNormalPyramidLevel & previousLevel, const Matrix3f& cameraMatrix){
//   newLevel.width = previousLevel.width/2;
//   newLevel.height = previousLevel.height/2;
// }



int main(int argc, char** argv)
{
  g2o::CommandArgs arg;
  string imageName;
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

  arg.paramLeftOver("image", imageName , "", "image", true);
  arg.parseArgs(argc, argv);
  
  if (imageName.length()==0){
    cerr << "no image provided" << endl;
    return 0;
  }

  QApplication application(argc, argv);

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
  MatrixXus image0;
  FILE* file;
  file = fopen(imageName.c_str(), "rb");
  if (!readPgm(image0, file)){
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
  MatrixXf depth0(rows, cols);
  img2depth(depth0, image0);

  // Create 3D point clouds with normals.
  Vector6fVector cloud0;
  depth2cloud(cloud0, depth0, cameraMatrix);
  CovarianceSVDVector svd0(cloud0.size());
  
  
  /************************************************************************************
   *                                                                                  *
   *  Compute normals and curvature of the 3D points.                                 *
   *                                                                                  *
   ************************************************************************************/
  // Create matrices of pointers.
  MatrixXf curvature0(rows, cols);
  MatrixXf zBuffer(rows, cols);
  Vector6fPtrMatrix cloud0Ptr(rows, cols);
  CovarianceSVDPtrMatrix svd0Ptr(rows, cols);
  Matrix6fVector omega0;
  Matrix6fPtrMatrix omega0Ptr(0,0);
  cloud2mat(cloud0Ptr,
	    omega0Ptr,
	    svd0Ptr,
            cloud0,
	    omega0,
	    svd0,
            Isometry3f::Identity(), cameraMatrix,
	    zBuffer);
  // Compute normals.
  cerr << "minPoints " <<  minPoints << endl;
  cerr << "computing normals1... ";
  computeNormals(cloud0Ptr, curvature0, svd0Ptr, cameraMatrix, r, d, step, minPoints);
  svd2omega(omega0, svd0);
  cerr << "done !" << endl;


  
  Vector6f x;
  x << 0.1f, 0.1f, -0.2f, 0.1f, 0.05f, 0.05f;
  Isometry3f X=v2t(x);

  Vector6fVector cloud1;
  Matrix6fVector omega1;
  CovarianceSVDVector svd1;
  remapCloud(cloud1, omega1, svd1, X, cloud0, omega0, svd0);
 

  // Scale all.
  //float scale = 1.0f / 4.0f;
  int _r=((float)image0.rows()*scale);
  int _c=((float)image0.cols()*scale);

  Vector6fPtrMatrix cloud0PtrScaled(_r, _c);
  Matrix6fPtrMatrix omega0PtrScaled(_r, _c);
  CovarianceSVDPtrMatrix svd0PtrScaled(_r, _c);
  Vector6fPtrMatrix cloud1PtrScaled(_r, _c);
  Matrix6fPtrMatrix omega1PtrScaled(_r, _c);
  CovarianceSVDPtrMatrix svd1PtrScaled(_r, _c);
  Matrix3f cameraMatrixScaled = cameraMatrix;
  cameraMatrixScaled.block<2,3>(0, 0) *= scale;
  Matrix6fPtrMatrix corrOmegas1(_r, _c);
  Vector6fPtrMatrix corrP0(_r,_c);
  Vector6fPtrMatrix corrP1(_r,_c);

  cloud2mat(cloud1PtrScaled,
	    omega1PtrScaled,
	    svd1PtrScaled,
	    cloud1,
	    omega1,
	    svd1,
	    Isometry3f::Identity(),
	    cameraMatrixScaled,
	    zBuffer);
  {
    MatrixXus img0(_r, _c);
    depth2img(img0, zBuffer);
    FILE* file;
    file = fopen("cloud1.pgm", "wb");
    if (!writePgm(img0, file)){
      cout << "Error while reading first depth image." << endl;
      exit(-1);
    }
    fclose(file);
  }

  Isometry3f T1_0 =Isometry3f::Identity();
  CorrVector correspondences;


  /************************************************************************************
   *                                                                                  *
   *  Draw 3D points with normals.                                                    *
   *                                                                                  *
   ************************************************************************************/;
  

  PWNQGLViewer viewer;
  viewer.setPointSize(pointSize);
  viewer.setNormalLength(normalLength);
  viewer.setEllipsoidScale(ellipsoidScale);
  viewer.setPoints(&cloud1);
  viewer.setPoints2(&cloud0);
  viewer.setEllipsoids(&svd0);
  viewer.setCorrVector(&correspondences);
  viewer.setWindowTitle("Viewer");
  // Make the viewer window visible on screen.
  viewer.show();

  
  Vector6fVector  cloudT;
  CovarianceSVDVector svdT;
  Matrix6fVector omegaT;

  int iterations=10;
  for (int k=0; k<iterations; k++) {
    clock_t kstart = getMilliSecs();
    
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

    char buf[1024];
    sprintf (buf, "cloud0-%02d.png", k);
    MatrixXus img0(_r, _c);
    depth2img(img0, zBuffer);
    FILE* file;
    file = fopen(buf, "wb");
    if (!writePgm(img0, file)){
      cout << "Error while reading first depth image." << endl;
      exit(-1);
    }
    fclose(file);

    corrOmegas1.fill(0);
    corrP0.fill(0);
    corrP1.fill(0);

    float curvatureThreshold=0.02;
    float normalThreshold = M_PI/6;
    correspondences.clear();
    int corrFound = 0;
    for (int i =0; i<cloud1PtrScaled.cols(); i++){
      for (int j =0; j<cloud1PtrScaled.rows(); j++){
	if (! cloud0PtrScaled(j,i) || !cloud1PtrScaled(j,i) ) {
	  continue;
	}
	Vector6f& p0 = *(cloud0PtrScaled(j,i));
	Vector6f& p1 = *(cloud1PtrScaled(j,i));
	if (p0.tail<3>().squaredNorm()<=1e-3 || p1.tail<3>().squaredNorm()<=1e-3){
	  continue;
	}
	SVDMatrix3f& svd0 = *svd0PtrScaled(j,i);
	SVDMatrix3f& svd1 = *svd1PtrScaled(j,i);

	float c0 = svd0.curvature();
	float c1 = svd1.curvature();
      
	if (c0>curvatureThreshold || c1>curvatureThreshold) {
	  continue;
	}
	Vector6f p0Remapped = remapPoint(T1_0, p0);
	Vector3f n0Remapped = p0Remapped.tail<3>();
	Vector3f n1 = p1.tail<3>();

	if (n0Remapped.dot(n1) <normalThreshold) {
	  continue;
	}
	correspondences.push_back(Corr(cloud0PtrScaled(j,i), cloud1PtrScaled(j,i)));
	corrP0(j,i) = &p0;
	corrP1(j,i) = &p1;
	corrOmegas1(j,i) = omega0PtrScaled(j,i);
	corrFound ++;
      }
    }
    cerr << "found " << corrFound << " correspondences" << endl;
    int size = _r*_c;
    // Compute transformation.
    Isometry3f result = Isometry3f::Identity();
    float error = 0.0f;
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
    cout << "k: " << k << " " << inl << " " << error << " " << endl ;//<< t2v(result) << endl;
    cout << "Time optimization : " << getMilliSecs() - start << " ms" << endl;
    cout << "Time global iteration: " << getMilliSecs() - kstart << " ms" << endl;
    cout << "---------------------------------------------------------------" << endl;  
  }

  cerr << "difference between the true and the computed transformation" << t2v(T1_0.inverse()*X) << endl;

  for (size_t i=0; i<cloud0.size(); i++){
    Vector6f p = cloud0[i];
    Vector6f &pRef = cloud0[i];
    pRef = remapPoint(T1_0, p);
  }



  // Run main loop.
  return application.exec();
    
  return 0;
}
