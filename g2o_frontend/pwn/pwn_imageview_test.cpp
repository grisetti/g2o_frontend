#include <iostream>
#include <string>
#include <cstdio>
#include <qapplication.h>
#include "pwn_normals.h"
#include "pwn_cloud.h"
#include "pwn_qglviewer.h"
#include "pwn_imageview.h"
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
  int dmin;
  int dmax;
  float scale;
  arg.param("r", r, 0.1f, "radius of the ball to compute the normals, in world coordinates");
  arg.param("d", d, 0.0f, "radius of the ball to compute the normals, in image coordinates (if =0 normal is computed on integral image)");
  arg.param("s", scale, 1.0f, "scaling factor to apply to the image");
  arg.param("mp", minPoints, 50, "min points to compiut de normal");
  arg.param("step", step, 2, "compute the normal each x rows and col");
  arg.param("dmin", dmin, 300, "dmin");
  arg.param("dmax", dmax, 7000, "dmax");
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
  
  Matrix3f cameraMatrix2 = cameraMatrix;
  cameraMatrix2.block<2,3>(0,0)*=scale;

  MatrixXf depth2((int)((float)image0.rows()*scale), (int) ((float)image0.cols()*scale));
  cloud2depth(depth2, cloud0, cameraMatrix2);
  MatrixXus image2(depth2.rows(), depth2.cols());
  depth2img(image2, depth2);
  
  ColorMap cmap;
  cmap.compute(dmin, dmax, 0xff);
  QImage qImage;
  toQImage(qImage, image2, cmap);

  QLabel myLabel;
  myLabel.setPixmap(QPixmap::fromImage(qImage));
  myLabel.show();

  // Run main loop.
  return application.exec();
    
  return 0;
}
