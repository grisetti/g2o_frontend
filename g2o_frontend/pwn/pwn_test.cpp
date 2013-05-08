#include "pinholepointprojector.h"
#include "homogeneouspoint3fstatsgenerator.h"
#include "omegagenerator.h"
#include "aligner.h"

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  // Depth image files (path+filename).
  string currentFilename, referenceFilename;

  // Variables for the input parameters. Just type on the command line
  // ./pwn_normal_extraction -h to have more details about them.
  float ng_scale = 1.0f;
  float ng_curvatureThreshold = 1.0f;
  int al_innerIterations = 1;
  int al_outerIterations = 10;
  int vz_step = 1;

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
  arg.param("al_innerIterations", al_innerIterations, 1, "Specify the inner iterations. [int]");
  arg.param("al_outerIterations", al_outerIterations, 10, "Specify the outer iterations. [int]");
  arg.param("vz_step", vz_step, 1, "A graphic element is drawn each vz_step elements. [int]");

  // Last parameter has to be the depth image file.
  arg.paramLeftOver("depthImageFile1", referenceFilename, "./test1.pgm", "First depth image file (.pgm image) to analyze. [string]", true);
  arg.paramLeftOver("depthImageFile2", currentFilename, "./test2.pgm", "Secodn depth image file (.pgm image) to analyze. [string]", true);

  // Set parser input.
  arg.parseArgs(argc, argv);
  
  /************************************************************************
   *                         Reading Depth Image                          *
   ************************************************************************/ 
  // The DepthImage object is used read a depth image from a .pgm image file. It is an extended Eigen 
  // matrix of unsigned char. 
  DepthImage referenceDepthImage, currentDepthImage;
  
  // Try to read the depth images given in input.
  if(!referenceDepthImage.load(referenceFilename.c_str(), true)) {
    cout << "Failure while loading the depth image: " << referenceFilename<< ", quitting program!" << endl;
    exit(-1);
  }
  if(!currentDepthImage.load(currentFilename.c_str(), true)) {
    cout << "Failure while loading the depth image: " << currentFilename << ", quitting program!" << endl;
    exit(-1);
  }
  
  cout << "Loaded " << referenceFilename.c_str() << " depth image of size: " << referenceDepthImage.rows() << "x" << referenceDepthImage.cols() << endl;
  cout << "Loaded " << currentFilename.c_str() << " depth image of size: " << currentDepthImage.rows() << "x" << currentDepthImage.cols() << endl;

  /************************************************************************
   *                         Point Unprojection                           *
   ************************************************************************/
  cout << "Unprojecting points...";

  // Update the size of the index image.
  Eigen::MatrixXi referenceIndexImage, currentIndexImage;
  referenceIndexImage.resize(referenceDepthImage.rows(), referenceDepthImage.cols());
  currentIndexImage.resize(currentDepthImage.rows(), currentDepthImage.cols());
    
  // Set the camera matrix of the projector object.
  PinholePointProjector projector;
  projector.setCameraMatrix(cameraMatrix);
  
  // Get the points in the 3d euclidean space.
  HomogeneousPoint3fScene referenceScene, currentScene;
  projector.unProject(referenceScene.points(), referenceIndexImage, referenceDepthImage);
  projector.unProject(currentScene.points(), currentIndexImage, currentDepthImage);
  
  cout << " done." << endl;
  
  /************************************************************************
   *                         Normal Computation                           *
   ************************************************************************/
  cout << "Computing normals...";

  HomogeneousPoint3fIntegralImage referenceIntegralImage, currentIntegralImage;
  MatrixXi referenceIntervalImage, currentIntervalImage;
  
  // Compute the integral images.
  referenceIntegralImage.compute(referenceIndexImage, referenceScene.points());
  currentIntegralImage.compute(currentIndexImage, currentScene.points());
    
  // Compute the intervals.
  projector.projectIntervals(referenceIntervalImage, referenceDepthImage, 0.1f);
  projector.projectIntervals(currentIntervalImage, currentDepthImage, 0.1f);
    
  // Creating the stas generator object. 
  HomogeneousPoint3fStatsGenerator statsGenerator;
  
  // Stats and normals computation.
  statsGenerator.compute(referenceScene.normals(), referenceScene.stats(),
   			 referenceScene.points(),
   			 referenceIntegralImage, referenceIntervalImage, referenceIndexImage,
   			 ng_curvatureThreshold);
  statsGenerator.compute(currentScene.normals(), currentScene.stats(),
  			 currentScene.points(), 
  			 currentIntegralImage, currentIntervalImage, currentIndexImage,
  			 ng_curvatureThreshold);
    
  cout << " done." << endl;

  /************************************************************************
   *                         Omega Computation                            *
   ************************************************************************/
  cout << "Computing omegas...";

  // Creating the omegas generators objects.
  PointOmegaGenerator pointOmegaGenerator;
  NormalOmegaGenerator normalOmegaGenerator;
  
  // Omegas computation.
  pointOmegaGenerator.compute(referenceScene.pointOmegas(), referenceScene.stats(), referenceScene.normals());
  normalOmegaGenerator.compute(referenceScene.normalOmegas(), referenceScene.stats(), referenceScene.normals());
  pointOmegaGenerator.compute(currentScene.pointOmegas(), currentScene.stats(), currentScene.normals());
  normalOmegaGenerator.compute(currentScene.normalOmegas(), currentScene.stats(), currentScene.normals());

  cout << " done." << endl;

  /************************************************************************
   *                         Alignment Computation                        *
   ************************************************************************/
  Aligner aligner;
  
  aligner.setOuterIterations(al_outerIterations);
  aligner.setInnerIterations(al_innerIterations);
  
  aligner.correspondenceGenerator().setReferenceIndexImage(&referenceIndexImage);
  aligner.correspondenceGenerator().setCurrentIndexImage(&currentIndexImage);
  aligner.correspondenceGenerator().setReferenceDepthImage(&referenceDepthImage);
  aligner.correspondenceGenerator().setCurrentDepthImage(&currentDepthImage);
  aligner.correspondenceGenerator().setSize(referenceIndexImage.rows(), referenceIndexImage.cols());
  
  aligner.setProjector(&projector);
  
  aligner.setReferenceScene(&referenceScene);
  aligner.setCurrentScene(&currentScene);
  
  Isometry3f initialGuess = Isometry3f::Identity();
  Isometry3f sensorOffset = Isometry3f::Identity();
  aligner.setInitialGuess(initialGuess);
  aligner.setSensorOffset(sensorOffset);
  
  aligner.align();
  	
  cout << "Final transformation: " << endl << aligner.T().matrix() << endl;

  return 0;
}
