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
  
  // The DepthImage object is used read a depth image from a .pgm image file. It is an extended Eigen 
  // matrix of unsigned char. 
  DepthImage referenceDepthImage, currentDepthImage;
  // Try to read the depth images given in input.
  if(!referenceDepthImage.load(referenceFilename.c_str())) {
    cout << "Failure while loading the depth image: " << referenceFilename<< ", quitting program!" << endl;
    exit(-1);
  }
  if(!currentDepthImage.load(currentFilename.c_str())) {
    cout << "Failure while loading the depth image: " << currentFilename << ", quitting program!" << endl;
    exit(-1);
  }

  // This is an hack since in the old code the images are loaded column-wise. 
  referenceDepthImage.transposeInPlace();
  currentDepthImage.transposeInPlace();
  cout << "Loaded first depth image of size: " << referenceDepthImage.rows() << "x" << referenceDepthImage.cols() << endl;
  cout << "Loaded second depth image of size: " << currentDepthImage.rows() << "x" << currentDepthImage.cols() << endl;
  
  /************************************************************************
   *                         Normal Computation                           *
   ************************************************************************/
  cout << "Computing normals...";

  PinholePointProjector projector;
  HomogeneousPoint3fVector referencePoints, currentPoints;
  MatrixXi referenceIndexImage, currentIndexImage;
    
  // Set the transformation and the camera matrix of the projector object.
  projector.setTransform(Isometry3f::Identity());
  projector.setCameraMatrix(cameraMatrix);
  // Get the points of the reference depth image in the 3d euclidean space.
  projector.unProject(referencePoints, referenceIndexImage, referenceDepthImage);
  // Get the points of the current depth image in the 3d euclidean space.
  projector.unProject(currentPoints, currentIndexImage, currentDepthImage);

  HomogeneousPoint3fIntegralImage referenceIntegralImage, currentIntegralImage;
  // Compute the integral images.
  referenceIntegralImage.compute(referenceIndexImage, referencePoints);
  currentIntegralImage.compute(currentIndexImage, currentPoints);
  
  MatrixXi referenceIntervalImage, currentIntervalImage;
  // Compute interval images.
  projector.projectIntervals(referenceIntervalImage, referenceDepthImage, 0.1f);
  projector.projectIntervals(currentIntervalImage, currentDepthImage, 0.1f);
  
  HomogeneousNormal3fVector referenceNormals, currentNormals;
  HomogeneousPoint3fStatsVector referenceStats, currentStats;
  HomogeneousPoint3fStatsGenerator statsGenerator;
  // Resize the vector containing normals and stats to have the same length of the vector of points.
  referenceNormals.resize(referencePoints.size());
  currentNormals.resize(currentPoints.size());
  referenceStats.resize(referencePoints.size());
  currentStats.resize(currentPoints.size());
  // Compute stats and normals.
  statsGenerator.compute(referenceNormals, referenceStats, referencePoints, referenceIntegralImage, referenceIntervalImage, referenceIndexImage, 0.02f);
  statsGenerator.compute(currentNormals, currentStats, currentPoints, currentIntegralImage, currentIntervalImage, currentIndexImage, 0.02f);
  
  cout << " done." << endl;

  /************************************************************************
   *                         Omega Computation                            *
   ************************************************************************/
  cout << "Computing omegas...";
  
  // Creating the omegas generators objects.
  PointOmegaGenerator pointOmegaGenerator;
  NormalOmegaGenerator normalOmegaGenerator;
  
  HomogeneousPoint3fOmegaVector currentPointOmega;
  HomogeneousPoint3fOmegaVector currentNormalOmega;
  // Omegas computation.
  pointOmegaGenerator.compute(currentPointOmega, currentStats, currentNormals);
  normalOmegaGenerator.compute(currentNormalOmega, currentStats, currentNormals);

  cout << " done." << endl;

  Isometry3f T = Isometry3f::Identity();
  for (int i = 0; i < al_outerIterations; i++) {
    cerr << "****************** ITERATION " << i << " ******************" << endl;
    /************************************************************************
     *                         Correspondence Computation                   *
     ************************************************************************/
    cout << "Computing correspondences...";
    // Creating the correspondences generator objects.
    CorrespondenceGenerator correspondenceGenerator;
    // Here will go the omegas.
    CorrespondenceVector correspondences;
    
    projector.setTransform(T.inverse());
    projector.project(referenceIndexImage,
		      referenceDepthImage,
		      referencePoints);
    
    // Correspondences computation.    
    correspondenceGenerator.compute(correspondences,
  				    referencePoints, currentPoints,
  				    referenceNormals, currentNormals,
  				    referenceIndexImage, currentIndexImage,
  				    referenceStats, currentStats,
  				    T);
  
    cout << " done." << endl;
    cout << "# inliers found: " << correspondenceGenerator.numCorrespondences() << endl;

    /************************************************************************
     *                            Alignment                                 *
     ************************************************************************/
    cout << "Computing alignment transformation...";
    Aligner aligner;
    Linearizer linearizer;
    aligner.setProjector(&projector);
    aligner.setLinearizer(&linearizer);
    aligner.setPoints(&referencePoints, &currentPoints);
    aligner.setNormals(&referenceNormals, &currentNormals);
    aligner.setStats(&referenceStats, &currentStats);
    aligner.setCurrentOmegas(&currentPointOmega, &currentNormalOmega);
    aligner.setCorrespondences(correspondences);
    aligner.setNumCorrespondences(correspondenceGenerator.numCorrespondences());
    linearizer.setAligner(&aligner);
    for (int k = 0; k < al_innerIterations; k++) {
      Matrix6f& H = linearizer.H();
      Vector6f& b = linearizer.b();
      H.setZero();
      b.setZero();
      linearizer.setT(T);
      linearizer.update();
      Vector6f dx = linearizer.H().ldlt().solve(-linearizer.b());
      Eigen::Isometry3f dT = v2t(dx);
      T = dT * T;
    }
    cout << " done." << endl;
    cout << "H: " << endl << linearizer.H() << endl;
    cout << "b: " << endl << linearizer.b() << endl;
  }

  T = T.inverse();
  cerr << "Final transformation: " << endl << T.matrix() << endl;  

  // This is just to check that the result is correct
   PointWithNormalVector referencePWNV(referencePoints.size());
   for(size_t i = 0; i < referencePWNV.size(); ++i) {
     referencePWNV[i].head<3>() = referencePoints[i].head<3>();
     referencePWNV[i].tail<3>() = referenceNormals[i].head<3>();
   }
   referencePWNV.save("reference.pwn", true);
  
   PointWithNormalVector currentPWNV(currentPoints.size());
   for(size_t i = 0; i < currentPWNV.size(); ++i) {
     currentPWNV[i].head<3>() = currentPoints[i].head<3>();
     currentPWNV[i].tail<3>() = currentNormals[i].head<3>();
   }
   currentPWNV.save("current.pwn", true);

  PointWithNormalVector alignedPWNV(currentPoints.size());
  for(size_t i = 0; i < alignedPWNV.size(); ++i) {
    alignedPWNV[i].head<3>() = T.linear()*currentPoints[i].head<3>() + T.translation();
    alignedPWNV[i].tail<3>() = T.linear()*currentNormals[i].head<3>();
  }
  alignedPWNV.save("aligned.pwn", true);

  return 0;
}
