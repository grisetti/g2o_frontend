#include "normalgenerator.h"
#include "omegagenerator.h"

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include <iostream>

#include "pointwithnormal.h"

using namespace std;

int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  
  
  // Depth image file (path+filename).
  string filename;

  // Variables for the input parameters. Just type on the command line
  // ./pwn_normal_extraction -h to have more details about them.
  float ng_scale = 1.0f;
  float ng_curvatureThreshold = 1.0f;

  // Define the camera matrix, place here the values for the particular 
  // depth camera used (Kinect, Xtion or any other type). This particular
  // matrix is the one related to the Kinect.
  Matrix3f cameraMatrix;
  cameraMatrix <<     
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  
  // Input parameters handling.
  g2o::CommandArgs arg;
  
  // Optional input parameters.
  arg.param("ng_scale", ng_scale, 1.0f, "Specify the scaling factor to apply on the depth image. [float]");
  arg.param("ng_curvatureThreshold", ng_curvatureThreshold, 1.0f, "Specify the max surface curvature threshold for which normals are discarded. [float]");
  
  // Last parameter has to be the depth image file.
  arg.paramLeftOver("depthImageFile", filename, "./test.pgm", "Depth image file (.pgm image) to analyze. [string]", true);

  // Set parser input.
  arg.parseArgs(argc, argv);
  
  // The DepthImage object is used read a depth image from a .pgm image file. It is an extended Eigen 
  // matrix of unsigned char. 
  DepthImage depthImage;
  // Try to read the depth image given in input.
  if(!depthImage.load(filename.c_str())) {
    cout << "Failure while loading the depth image: " << filename << ", quitting program!" << endl;
    exit(-1);
  }

  // This is an hack since in the old code the images are loaded column-wise. 
  depthImage.transposeInPlace();
  cout << endl << "Loaded a depth image of size: " << depthImage.rows() << "x" << depthImage.cols() << endl;
  
  /************************************************************************
   *                         Normal Computation                           *
   ************************************************************************/

  // Creating the normal generator object and setting some parameters. 
  NormalGenerator normalGenerator;
  // Set the scale factor to apply on the depth image.
  normalGenerator.setScale(ng_scale);
  // Set the camera matrix
  normalGenerator.setCameraMatrix(cameraMatrix);
  
  // Here will go the points.
  HomogeneousPoint3fVector imagePoints; 
  // Here will go the normals.
  HomogeneousNormal3fVector imageNormals;
  
  // Normals computation.
  normalGenerator.compute(imagePoints, imageNormals, depthImage, ng_curvatureThreshold);
  
  /************************************************************************
   *                         Omega Computation                            *
   ************************************************************************/
  PointOmegaGenerator pointOmegaGenerator;
  NormalOmegaGenerator normalOmegaGenerator;
  HomogeneousPoint3fOmegaVector pointOmega;
  HomogeneousPoint3fOmegaVector normalOmega;

  pointOmegaGenerator.compute(pointOmega, normalGenerator.scaledStats, imageNormals);
  normalOmegaGenerator.compute(normalOmega, normalGenerator.scaledStats, imageNormals);

  /************************************************************************
   *                         Correspondence Computation                   *
   ************************************************************************/
  //CorrespondenceGenerator correspondenceGenerator;
  

  // This is just to check that the result is correct
  PointWithNormalVector pnv(imagePoints.size());
  for(size_t i = 0; i < pnv.size(); ++i) {
    pnv[i].head<3>() = imagePoints[i].head<3>();
    pnv[i].tail<3>() = imageNormals[i].head<3>();
  }
  pnv.save("out.pwn", true);
  
  return 0;
}
