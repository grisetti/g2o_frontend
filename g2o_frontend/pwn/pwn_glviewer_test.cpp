#include <iostream>
#include <string>
#include <cstdio>
#include <qapplication.h>
#include "pwn_normals.h"
#include "pwn_cloud.h"
#include "pwn_qglviewer.h"
#include "pwn_imageview.h"
#include "pwn_math.h"
#include "g2o/stuff/command_args.h"

using namespace std;

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
  arg.param("r", r, 0.1f, "radius of the ball to compute the normals, in world coordinates");
  arg.param("d", d, 0.0f, "radius of the ball to compute the normals, in image coordinates (if =0 normal is computed on integral image)");
  arg.param("mp", minPoints, 50, "min points to compiut de normal");
  arg.param("step", step, 2, "compute the normal each x rows and col");
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
  Matrix6fVector _dummyOmega;
  Matrix6fPtrMatrix _dummyOmegaMat(0,0);
  cloud2mat(cloud0Ptr,
	    _dummyOmegaMat,
	    svd0Ptr,
            cloud0,
	    _dummyOmega,
	    svd0,
            Isometry3f::Identity(), cameraMatrix,
	    zBuffer);
  // Compute normals.
  cerr << "minPoints " <<  minPoints << endl;
  cerr << "computing normals1... ";
  computeNormals(cloud0Ptr, curvature0, svd0Ptr, cameraMatrix, r, d, step, minPoints);
  cerr << "done !" << endl;


  // try to apply a translation

  // Vector6f x;
  // x << 1.0, 2.0, -1.0, 0.5, 0.5, 0.5;
  // Isometry3f X=v2t(x);

  // Vector6fVector cloudT;
  // Matrix6fVector omegaT;
  // CovarianceSVDVector svdT;
  // remapCloud(cloudT, omegaT, svdT, X, cloud0, Matrix6fVector(), svd0);

  // cloudT.insert(cloudT.end(), cloud0.begin(), cloud0.end());
  // svdT.insert(svdT.end(), svd0.begin(), svd0.end());


  PWNQGLViewer viewer;
  viewer.setPointSize(pointSize);
  viewer.setNormalLength(normalLength);
  viewer.setEllipsoidScale(ellipsoidScale);

  viewer.setPoints(&cloud0);
  viewer.setEllipsoids(&svd0);
  viewer.setWindowTitle("Viewer");

  // Make the viewer window visible on screen.
  viewer.show();

  // Run main loop.
  return application.exec();
    
  return 0;
}
