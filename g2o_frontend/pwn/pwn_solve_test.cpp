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
  string nameImage0;
  string nameImage1;
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
  
  if (nameImage0.length()==0){
    cerr << "no image provided" << endl;
    return 0;
  }

  QApplication application(argc, argv);
  
  // Create camera matrix.
  Matrix3f cameraMatrix;
  cameraMatrix << 525.0f, 0.0f, 319.5f,
                  0.0f, 525.0f, 239.5f,
                  0.0f, 0.0f, 1.0f;
  
  // Transformation init.
  Vector6f v;
  v << 0.1f, 0.1f, -0.1f, 0.2f, 0.1f, -0.3f;
  Isometry3f T = v2t(v);
  Isometry3f T_1 = v2t(-v);
  
  /************************************************************************************
   *                                                                                  *
   *  Read depth image.                                                              *
   *                                                                                  *
   ************************************************************************************/
  MatrixXus image0;
  FILE* file;
  file = fopen(nameImage0.c_str(), "rb");
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
  MatrixXf depth0(rows, cols), depth1(rows, cols);
  img2depth(depth0, image0);
  img2depth(depth1, image0);

  // Create 3D point clouds with normals.
  Vector6fVector cloud0, cloud1;
  depth2cloud(cloud0, depth0, cameraMatrix);
  depth2cloud(cloud1, depth1, cameraMatrix);
  CovarianceSVDVector svd0(cloud0.size());
  CovarianceSVDVector svd1(cloud1.size());
  
  // Change viewpoint for cloud1.
  for (size_t i=0; i<cloud1.size(); i++){
    Vector6f p = cloud1[i];
    Vector6f &pRef = cloud1[i];
    pRef = remapPoint(T, p);
  }
  for (size_t i=0; i<cloud1.size(); i++){
    Vector6f p = cloud1[i];
    Vector6f &pRef = cloud1[i];
    pRef = remapPoint(T_1, p);
  }
  
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
  cerr << "minPoints " <<  minPoints << endl;
  cerr << "computing normals1... ";
  computeNormals(cloud0Ptr, curvature0, svd0Ptr, cameraMatrix, r, d, step, minPoints);
  cerr << "done !" << endl;

  cerr << "computing normals2... ";
  computeNormals(cloud1Ptr, curvature1, svd1Ptr, cameraMatrix, r, d, step, minPoints);
  cerr << "done !" << endl;

  // Compute 6x6 omega matrix.
  Matrix6fVector omega1, omega0;
  cerr << "computing omega... ";
  svd2omega(omega0, svd0);
  svd2omega(omega1, svd1);
  cerr << "done !" << endl;

  // construct the pyramids
  // int nPyramids = 3;

  // Scale all.
  float scale = 1.0f / 1.0f;
  Vector6fPtrMatrix cloud0PtrScaled((int)((float)image0.rows()*scale), (int) ((float)image0.cols()*scale));
  Matrix6fPtrMatrix omega0PtrScaled((int)((float)image0.rows()*scale), (int) ((float)image0.cols()*scale));
  CovarianceSVDPtrMatrix svd0PtrScaled((int)((float)image0.rows()*scale), (int) ((float)image0.cols()*scale));
  Vector6fPtrMatrix cloud1PtrScaled((int)((float)image0.rows()*scale), (int) ((float)image0.cols()*scale));
  Matrix6fPtrMatrix omega1PtrScaled((int)((float)image0.rows()*scale), (int) ((float)image0.cols()*scale));
  CovarianceSVDPtrMatrix svd1PtrScaled((int)((float)image0.rows()*scale), (int) ((float)image0.cols()*scale));
  Matrix3f cameraMatrixScaled = cameraMatrix;
  cameraMatrixScaled.block<2,3>(0, 0) *= scale;
  cloud2mat(cloud0PtrScaled,
	    omega0PtrScaled,
	    svd0PtrScaled,
	    cloud0,
	    omega0,
	    svd0,
	    Isometry3f::Identity(),
	    cameraMatrixScaled,
	    zBuffer);
  cloud2mat(cloud1PtrScaled,
	    omega1PtrScaled,
	    svd1PtrScaled,
	    cloud1,
	    omega1,
	    svd1,
	    Isometry3f::Identity(),
	    cameraMatrixScaled,
	    zBuffer);
  int size = rows*cols;
  // Compute transformation.
  Isometry3f result = Isometry3f::Identity();
  float error = 0.0f;
  for(int i=0; i<10; i++){
    clock_t start = getMilliSecs();
    int inl = pwn_iteration(error, result,
			    cloud0PtrScaled.data(),
			    cloud1PtrScaled.data(),
			    omega1PtrScaled.data(),
			    size,
			    result,
			    0.01f,
			    0);
    cout << "i: " << i << " " << inl << " " << error << " " << endl << t2v(result) << endl;
    cout << "Time elapsed: " << getMilliSecs() - start << " ms" << endl;
    cout << "---------------------------------------------------------------" << endl;  
  }

  /************************************************************************************
   *                                                                                  *
   *  Draw 3D points with normals.                                                    *
   *                                                                                  *
   ************************************************************************************/;
  for(size_t i=0; i<cloud1.size(); i++){
    Vector6f pT = cloud1[i];
    Vector6f &point = cloud1[i];
    point = remapPoint(result, pT);
  }
  PWNQGLViewer viewer;
  viewer.setPointSize(pointSize);
  viewer.setNormalLength(normalLength);
  viewer.setEllipsoidScale(ellipsoidScale);
  viewer.setPoints(&cloud0);
  viewer.setPoints2(&cloud1);
  viewer.setEllipsoids(&svd0);
  viewer.setWindowTitle("Viewer");

  // Make the viewer window visible on screen.
  viewer.show();

  // Run main loop.
  return application.exec();
    
  return 0;
}
