#include "g2o_frontend/pwn/normalgenerator.h"
#include "g2o_frontend/pwn/omegagenerator.h"
#include "g2o_frontend/pwn/correspondencegenerator.h"
#include "g2o_frontend/pwn/pinholepointprojector.h"
#include "g2o_frontend/pwn/aligner.h"

#include <qapplication.h>
#include "g2o_frontend/pwn_viewer/pwn_qglviewer.h"
#include "g2o_frontend/pwn_viewer/pwn_imageview.h"
#include "g2o_frontend/pwn_viewer/drawable_points.h"
#include "g2o_frontend/pwn_viewer/drawable_normals.h"
#include "g2o_frontend/pwn_viewer/drawable_covariances.h"
#include "g2o_frontend/pwn_viewer/drawable_correspondences.h"
#include "g2o_frontend/pwn_viewer/gl_parameter.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_points.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_normals.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_covariances.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_correspondences.h"
#include "pwn_gui_main_window.h"

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include <iostream>
#include <fstream>


#include "g2o_frontend/pwn/pointwithnormal.h"
#include "g2o_frontend/basemath/bm_se3.h"
using namespace std;

int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  
  // Depth image file (path+filename).
  string currentFilename, referenceFilename;

  // Variables for the input parameters. Just type on the command line
  // ./pwn_normal_extraction -h to have more details about them.
  float ng_scale = 1.0f;
  float ng_curvatureThreshold = 1.0f;
  int al_innerIterations = 5;
  int al_outerIterations = 5;
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
  arg.param("al_innerIterations", al_innerIterations, 5, "Specify the inner iterations. [int]");
  arg.param("al_outerIterations", al_outerIterations, 5, "Specify the outer iterations. [int]");
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
  if(!referenceDepthImage.load(referenceFilename.c_str(), true)) {
    cout << "Failure while loading the depth image: " << referenceFilename<< ", quitting program!" << endl;
    exit(-1);
  }
  if(!currentDepthImage.load(currentFilename.c_str(), true)) {
    cout << "Failure while loading the depth image: " << currentFilename << ", quitting program!" << endl;
    exit(-1);
  }
  cout << endl << "Loaded first depth image of size: " << referenceDepthImage.rows() << "x" << referenceDepthImage.cols() << endl;
  cout << endl << "Loaded second depth image of size: " << currentDepthImage.rows() << "x" << currentDepthImage.cols() << endl;
  
  /************************************************************************
   *                         Normal Computation                           *
   ************************************************************************/
  cout << "Computing normals...";
  // Creating the normal generator object and setting some parameters. 
  NormalGenerator referenceNormalGenerator, currentNormalGenerator;
  // Set the scale factor to apply on the depth image.
  referenceNormalGenerator.setScale(ng_scale);
  currentNormalGenerator.setScale(ng_scale);
  // Set the camera matrix
  referenceNormalGenerator.setCameraMatrix(cameraMatrix);
  currentNormalGenerator.setCameraMatrix(cameraMatrix);
  
  // Here will go the points.
  HomogeneousPoint3fVector referenceImagePoints; 
  HomogeneousPoint3fVector currentImagePoints; 
  // Here will go the normals.
  HomogeneousNormal3fVector referenceImageNormals;
  HomogeneousNormal3fVector currentImageNormals;

  // Normals computation.
  referenceNormalGenerator.compute(referenceImagePoints, referenceImageNormals, referenceDepthImage, ng_curvatureThreshold);
  currentNormalGenerator.compute(currentImagePoints, currentImageNormals, currentDepthImage, ng_curvatureThreshold);
  cout << " done." << endl;

  /************************************************************************
   *                         Omega Computation                            *
   ************************************************************************/
  cout << "Computing omegas...";
  // Creating the omegas generators objects.
  PointOmegaGenerator pointOmegaGenerator;
  NormalOmegaGenerator normalOmegaGenerator;
  // Here will go the omegas.
  HomogeneousPoint3fOmegaVector currentPointOmega;
  HomogeneousPoint3fOmegaVector currentNormalOmega;

  // Omegas computation.
  pointOmegaGenerator.compute(currentPointOmega, currentNormalGenerator.scaledStats, currentImageNormals);
  normalOmegaGenerator.compute(currentNormalOmega, currentNormalGenerator.scaledStats, currentImageNormals);
  cout << " done." << endl;
  
  /************************************************************************
   *                         Alignment Computation                        *
   ************************************************************************/
  Aligner aligner;
  Linearizer linearizer;
  aligner.setProjector(&currentNormalGenerator.projector);
  aligner.setLinearizer(&linearizer);
  linearizer.setAligner(&aligner);
  aligner.setPoints(&referenceImagePoints, &currentImagePoints);
  aligner.setNormals(&referenceImageNormals, &currentImageNormals);
  aligner.setStats(&referenceNormalGenerator.scaledStats, &currentNormalGenerator.scaledStats);
  aligner.setCurrentOmegas(&currentPointOmega, &currentNormalOmega);
  aligner.setOuterIterations(al_outerIterations);
  aligner.setInnerIterations(al_innerIterations);
  aligner.setImageSize(currentNormalGenerator.scaledIndexImage.rows(), currentNormalGenerator.scaledIndexImage.cols());
  
  Isometry3f initialGuess = Isometry3f::Identity();
  Isometry3f sensorOffset = Isometry3f::Identity();
  aligner.setInitialGuess(initialGuess);
  aligner.setSensorOffset(sensorOffset);
  
  aligner.align();
  
  cout << "Final transformation: " << endl << aligner.T().matrix() << endl;
  
  // This is just to check that the result is correct
  PointWithNormalVector* referencePWNV = new PointWithNormalVector(referenceImagePoints.size());
  for(size_t i = 0; i < referencePWNV->size(); ++i) {
    referencePWNV->at(i).head<3>() = referenceImagePoints[i].head<3>();
    referencePWNV->at(i).tail<3>() = referenceImageNormals[i].head<3>();
  }
  
  Isometry3f finalT = aligner.T().inverse();
  PointWithNormalVector* currentPWNV = new PointWithNormalVector(currentImagePoints.size());
  for(size_t i = 0; i < currentPWNV->size(); ++i) {
    currentPWNV->at(i).head<3>() = currentImagePoints[i].head<3>();
    currentPWNV->at(i).tail<3>() = currentImageNormals[i].head<3>();
  }
  
  QApplication qApplication(argc, argv);
  PWNGuiMainWindow pwnGMW;
  pwnGMW.show();

  GLParameterPoints *pParamReference = new GLParameterPoints(1.0f, Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
  pParamReference->setStep(vz_step);
  GLParameterNormals *nParamReference = new GLParameterNormals(1.0f, Vector4f(1.0f, 1.0f, 0.0f, 1.0f), 0.02f);
  nParamReference->setStep(vz_step);
  GLParameterPoints *pParamCurrent = new GLParameterPoints(1.0f, Vector4f(1.0f, 0.5f, 0.0f, 1.0f));
  pParamCurrent->setStep(vz_step);
  GLParameterNormals *nParamCurrent = new GLParameterNormals(1.0f, Vector4f(1.0f, 1.0f, 0.0f, 1.0f), 0.02f);
  nParamCurrent->setStep(vz_step);
  GLParameterCovariances *cParam = new GLParameterCovariances(0.5f, Vector4f(0.0f, 1.0f, 0.0f, 1.0f), Vector4f(1.0f, 0.0f, 0.0f, 1.0f), 0.02f, 0.03f);
  cParam->setStep(vz_step);
  GLParameterCorrespondences *corrParam = new GLParameterCorrespondences(1.0f, Vector4f(0.0f, 0.0f, 1.0f, 1.0f), 1.0f);
  corrParam->setStep(vz_step);
  
  vector<Drawable*> drawableList = pwnGMW.viewer_3d->drawableList();
  DrawablePoints* dpReference = new DrawablePoints(Isometry3f::Identity(), (GLParameter*)pParamReference, referencePWNV);
  DrawableNormals *dnReference = new DrawableNormals(Isometry3f::Identity(), (GLParameter*)nParamReference, referencePWNV);
  DrawablePoints* dpCurrent = new DrawablePoints(finalT, (GLParameter*)pParamCurrent, currentPWNV);
  DrawableNormals *dnCurrent = new DrawableNormals(finalT, (GLParameter*)nParamCurrent, currentPWNV);
  DrawableCovariances *dcov = new DrawableCovariances(Isometry3f::Identity(), (GLParameter*)cParam, &referenceNormalGenerator.scaledStats);
  DrawableCorrespondences* dcorr = new DrawableCorrespondences(finalT, (GLParameter*)corrParam, 0, 0);
  dcorr->setPoints1(referencePWNV);
  dcorr->setPoints2(currentPWNV);
  dcorr->setCorrespondences(&aligner.correspondences());
  dcorr->setNumCorrespondences(aligner.numCorrespondences());

  pwnGMW.viewer_3d->addDrawable((Drawable*)dpReference);
  //pwnGMW.viewer_3d->addDrawable((Drawable*)dnReference);
  pwnGMW.viewer_3d->addDrawable((Drawable*)dpCurrent);
  //pwnGMW.viewer_3d->addDrawable((Drawable*)dnCurrent);
  //pwnGMW.viewer_3d->addDrawable((Drawable*)dcov);
  //pwnGMW.viewer_3d->addDrawable((Drawable*)dcorr);

  while (!(*pwnGMW.closing())) {
    qApplication.processEvents();
    pwnGMW.viewer_3d->updateGL();
    usleep(10000);
  }
  return 0;
}
