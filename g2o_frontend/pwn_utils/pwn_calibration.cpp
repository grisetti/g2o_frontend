#include <iostream>
#include <fstream>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>

#include "g2o/stuff/command_args.h"

#include "g2o_frontend/pwn2/depthimage.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"

#include "g2o_frontend/basemath/bm_se3.h"

#include "g2o_frontend/pwn_viewer/pwn_qglviewer.h"
#include "g2o_frontend/pwn_viewer/drawable_points.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_points.h"
#include "g2o_frontend/pwn_viewer/drawable_normals.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_normals.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pwn;

int main(int argc, char **argv) {
  float pointSize;
  float pointStep;
  float normalLenght;
  float normalStep;
  float alphaColor;
  int initialSensorOffset;
  string filename;

  // Input parameters handling.
  g2o::CommandArgs arg;
  arg.param("pointSize", pointSize, 1.0f, "Size of the points");
  arg.param("normalLenght", normalLenght, 0, "Lenght of the normals");
  arg.param("alpha", alphaColor, 1.0f, "Alpha channel for points");
  arg.param("pointStep", pointStep, 1, "Step of the points");
  arg.param("normalStep", normalStep, 1, "Step of the normals");
  arg.param("initialSensorOffset", initialSensorOffset, 0, "Choose if use the initial sensor offset");
  arg.paramLeftOver("filename", filename, "", "Input depth image or .pwn file", true);
 
  // Set parser input.
  arg.parseArgs(argc, argv);

  Isometry3f sensorOffsetInit = Isometry3f::Identity();
  if(initialSensorOffset) {
    sensorOffsetInit.translation() = Vector3f(0.15f, 0.0f, 0.05f);
    Quaternionf quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
    sensorOffsetInit.linear() = quaternion.toRotationMatrix();
  }
  sensorOffsetInit.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  // Windows name
  static const char CALIBRATE_WINDOW[] = "Calibrate Window";

  // Calibration angles
  int maxTrackBarValue = 1800;
  int alpha = maxTrackBarValue / 2; // Around x
  int beta = maxTrackBarValue / 2;  // Around y
  int theta = maxTrackBarValue / 2; // Around z

  // Create the window and the trackbars
  cv::namedWindow(CALIBRATE_WINDOW);
  cvMoveWindow(CALIBRATE_WINDOW, 0, 0);
  cvResizeWindow(CALIBRATE_WINDOW, 500, 200);
  cvCreateTrackbar("Calibrate X", CALIBRATE_WINDOW, &alpha, 1800, NULL);
  cvCreateTrackbar("Calibrate Y", CALIBRATE_WINDOW, &beta, 1800, NULL);
  cvCreateTrackbar("Calibrate Z", CALIBRATE_WINDOW, &theta, 1800, NULL);  
  
  // Create objects in order to read the input depth image / pwn file
  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;

  string extension;
  extension = filename.substr(filename.rfind(".") + 1);

  Frame frame;
  Isometry3f globalT = Isometry3f::Identity();
  cerr << "Loading " << filename.c_str() << endl;
  if(extension == "pgm") {
    PinholePointProjector projector;
    projector.setCameraMatrix(cameraMatrix);
    StatsCalculator statsCalculator;
    PointInformationMatrixCalculator pointInformationMatrixCalculator;
    NormalInformationMatrixCalculator normalInformationMatrixCalculator;
    DepthImageConverter depthImageConverter(&projector, &statsCalculator,
					    &pointInformationMatrixCalculator,
					    &normalInformationMatrixCalculator);
    
    DepthImage inputImage;
    inputImage.load(filename.c_str(),true);
    
    cerr << "Computing stats... ";
    depthImageConverter.compute(frame, inputImage);
    cerr << " done" << endl;
  }
  else if(extension == "pwn") {
    frame.load(globalT, filename.c_str());
  }
  else {
    cerr << "File extension nor recognized, quitting." << endl;
    return(0);
  }

  Isometry3f oldSensorOffset = Isometry3f::Identity();
  oldSensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  QApplication application(argc,argv);
  QWidget* mainWindow = new QWidget();
  mainWindow->setWindowTitle("pwn_calibration");
  QHBoxLayout* hlayout = new QHBoxLayout();
  mainWindow->setLayout(hlayout);
  QVBoxLayout* vlayout = new QVBoxLayout();
  hlayout->addItem(vlayout);
  
  PWNQGLViewer* viewer = new PWNQGLViewer(mainWindow);
  vlayout->addWidget(viewer);
  
  viewer->init();
  viewer->show();
  viewer->setAxisIsDrawn(true);
  mainWindow->show();
  
  GLParameterPoints *pointsParams = new GLParameterPoints(pointSize, Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
  pointsParams->setStep(pointStep);
  DrawablePoints *drawablePoints = new DrawablePoints(oldSensorOffset, pointsParams, &frame.points(), &frame.normals());
  viewer->addDrawable(drawablePoints);
  
  GLParameterNormals *normalParams = new GLParameterNormals(pointSize, Eigen::Vector4f(0.0f, 0.0f, 1.0f, alphaColor), normalLenght);
  DrawableNormals *drawableNormals = new DrawableNormals(oldSensorOffset, normalParams, &frame.points(), &frame.normals());
  normalParams->setStep(normalStep);
  normalParams->setNormalLength(normalLenght);
  viewer->addDrawable(drawableNormals);

  // Keep cycling
  Isometry3f sensorOffset;
  while(mainWindow->isVisible()) {
    // Updating variables
    float alphar = 2.0f * 3.14*((float)alpha - maxTrackBarValue / 2) / maxTrackBarValue;
    float betar = 2.0f * 3.14*((float)beta - maxTrackBarValue / 2) / maxTrackBarValue;
    float thetar = 2.0f * 3.14*((float)theta - maxTrackBarValue / 2) / maxTrackBarValue;
    
    // Generate rotations
    Quaternion<float> qx, qy, qz; 
    qx = AngleAxis<float>(alphar, Vector3f(1.0f, 0.0f, 0.0f));
    qy = AngleAxis<float>(betar, Vector3f(0.0f, 1.0f, 0.0f));
    qz = AngleAxis<float>(thetar, Vector3f(0.0f, 0.0f, 1.0f));
    
    Matrix3f totalRotation = qz.toRotationMatrix() * qy.toRotationMatrix() * qx.toRotationMatrix();
    sensorOffset = Isometry3f::Identity();
    sensorOffset.translation() = Vector3f(0.0f, 0.0f, 0.0f);
    sensorOffset.linear() = totalRotation * sensorOffsetInit.linear();
    sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

    drawablePoints->setTransformation(sensorOffset);
    drawableNormals->setTransformation(sensorOffset);

    oldSensorOffset = sensorOffset;

    viewer->updateGL();
    application.processEvents();

    cv::waitKey(33);
  }

  ofstream os("sensorOffset.txt");
  Vector6f offset = t2v(sensorOffset);
  os << offset[0] << " " << offset[1] << " " << offset[2] << " " 
     << offset[3] << " " << offset[4] << " " << offset[5] << endl;

  return(0);
}
