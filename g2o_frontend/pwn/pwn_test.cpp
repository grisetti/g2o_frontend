#include "homogeneousvector4f.h"
#include "depthimage.h"
#include "pinholepointprojector.h"
#include "homogeneouspoint3fintegralimage.h"
#include "homogeneouspoint3fstatsgenerator.h"
#include "omegagenerator.h"
#include "correspondencegenerator.h"
#include "aligner.h"

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "pointwithnormal.h"

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
  int vz_step = 5;

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
  arg.param("vz_step", vz_step, 5, "A graphic element is drawn each vz_step elements. [int]");

  // Last parameter has to be the depth image file.
  arg.paramLeftOver("depthImageFile1", referenceFilename, "./test1.pgm", "First depth image file (.pgm image) to analyze. [string]", true);
  arg.paramLeftOver("depthImageFile2", currentFilename, "./test2.pgm", "Secodn depth image file (.pgm image) to analyze. [string]", true);

  // Set parser input.
  arg.parseArgs(argc, argv);
  

  // The HomogeneousPoint3fScene object is used to store a scene. 
  HomogeneousPoint3fScene referenceScene, currentScene;
  // Try to read the depth images given in input.
  if(!referenceScene.depthImage().load(referenceFilename.c_str(), true)) {
    cout << "Failure while loading the depth image: " << referenceFilename<< ", quitting program!" << endl;
    exit(-1);
  }
  if(!currentScene.depthImage().load(currentFilename.c_str(), true)) {
    cout << "Failure while loading the depth image: " << currentFilename << ", quitting program!" << endl;
    exit(-1);
  }
  cout << endl << "Loaded first depth image of size: " << referenceScene.depthImage().rows() << "x" << referenceScene.depthImage().cols() << endl;
  cout << endl << "Loaded second depth image of size: " << currentScene.depthImage().rows() << "x" << currentScene.depthImage().cols() << endl;
  
  /************************************************************************
   *                         Point Unprojection                           *
   ************************************************************************/
  cout << "Unprojecting points...";

  // Projector object.
  PinholePointProjector projector;

  // Update the size of the index image.
  referenceScene.indexImage().resize(referenceScene.depthImage().rows(), referenceScene.depthImage().cols());
  currentScene.indexImage().resize(currentScene.depthImage().rows(), currentScene.depthImage().cols());

  // Set the camera matrix of the projector object.
  projector.setCameraMatrix(cameraMatrix);
  
  // Get the points in the 3d euclidean space.
  projector.unProject(referenceScene.points(), referenceScene.indexImage(), referenceScene.depthImage());
  projector.unProject(currentScene.points(), currentScene.indexImage(), currentScene.depthImage());

  cout << " done." << endl;

  /************************************************************************
   *                         Normal Computation                           *
   ************************************************************************/
  cout << "Computing normals...";

  HomogeneousPoint3fIntegralImage referenceIntegralImage, currentIntegralImage;
  MatrixXi referenceIntervalImage, currentIntervalImage;
  
  // Compute the integral images.
  referenceIntegralImage.compute(referenceScene.indexImage(), referenceScene.points());
  currentIntegralImage.compute(currentScene.indexImage(), currentScene.points());

  // Compute the intervals.
  projector.projectIntervals(referenceIntervalImage, referenceScene.depthImage(), 0.1f);
  projector.projectIntervals(currentIntervalImage, currentScene.depthImage(), 0.1f);
  
  // Resize the vector containing the stats to have the same length of the vector of points.
  referenceScene.stats().resize(referenceScene.points().size());
  currentScene.stats().resize(currentScene.points().size());
  std::fill(referenceScene.stats().begin(), referenceScene.stats().end(), HomogeneousPoint3fStats());
  std::fill(currentScene.stats().begin(), currentScene.stats().end(), HomogeneousPoint3fStats());
  
  // Creating the stas generator object. 
  HomogeneousPoint3fStatsGenerator statsGenerator;
  
  // Stats and normals computation.
  statsGenerator.compute(referenceScene.normals(),
			 referenceScene.stats(),
			 referenceScene.points(),
			 referenceIntegralImage,
			 referenceIntervalImage,
			 referenceScene.indexImage(),
			 ng_curvatureThreshold);
  statsGenerator.compute(currentScene.normals(),
			 currentScene.stats(),
			 currentScene.points(),
			 currentIntegralImage,
			 currentIntervalImage,
			 currentScene.indexImage(),
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
  pointOmegaGenerator.compute(currentScene.pointOmegas(), currentScene.stats(), currentScene.normals());
  normalOmegaGenerator.compute(currentScene.normalOmegas(), currentScene.stats(), currentScene.normals());

  cout << " done." << endl;
  
  /************************************************************************
   *                         Alignment Computation                        *
   ************************************************************************/
  cout << "Computing alignment transformation...";

  Aligner aligner;
  aligner.setProjector(&projector);
  aligner.setReferenceScene(&referenceScene);
  aligner.setCurrentScene(&currentScene);
  aligner.setOuterIterations(al_outerIterations);
  aligner.setInnerIterations(al_innerIterations);
  
  Isometry3f initialGuess = Isometry3f::Identity();
  Isometry3f sensorOffset = Isometry3f::Identity();
  aligner.setInitialGuess(initialGuess);
  aligner.setSensorOffset(sensorOffset);
  
  aligner.align();

  cout << " done." << endl;
  
  cout << "Final transformation: " << endl << aligner.T().matrix() << endl;

  // This is just to check that the result is correct
  PointWithNormalVector referencePWNV(referenceScene.points().size());
  for(size_t i = 0; i < referencePWNV.size(); ++i) {
    referencePWNV[i].head<3>() = referenceScene.points()[i].head<3>();
    referencePWNV[i].tail<3>() = referenceScene.normals()[i].head<3>();
  }
  referencePWNV.save("reference.pwn", true);
  
  PointWithNormalVector currentPWNV(currentScene.points().size());
  for(size_t i = 0; i < currentPWNV.size(); ++i) {
    currentPWNV[i].head<3>() = currentScene.points()[i].head<3>();
    currentPWNV[i].tail<3>() = currentScene.normals()[i].head<3>();
  }
  currentPWNV.save("current.pwn", true);

  Isometry3f T = aligner.T();

  PointWithNormalVector alignedPWNV(currentScene.points().size());
  for(size_t i = 0; i < alignedPWNV.size(); ++i) {
    alignedPWNV[i].head<3>() = T.linear()*currentScene.points()[i].head<3>() + T.translation();
    alignedPWNV[i].tail<3>() = T.linear()*currentScene.normals()[i].head<3>();
  }
  alignedPWNV.save("aligned.pwn", true);
   
  return 0;
}
