#include <iostream>

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QListWidget>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/statscalculatorintegralimage.h"
#include "g2o_frontend/pwn2/informationmatrixcalculator.h"
#include "g2o_frontend/pwn2/depthimageconverterintegralimage.h"
#include "g2o_frontend/pwn2/aligner.h"

#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"

#include "g2o_frontend/pwn_viewer/pwn_qglviewer.h"
#include "g2o_frontend/pwn_viewer/drawable_points.h"
#include "g2o_frontend/pwn_viewer/drawable_transform_covariance.h"

using namespace std;
using namespace Eigen;
using namespace pwn;
using namespace boss;

std::vector<Serializable*> readConfig(Aligner *&aligner, DepthImageConverter *&converter, const std::string &configFile);

int main(int argc, char **argv) {
  int vz_step;
  float di_scaleFactor, di_scale, vz_ellipsoidScale;
  string configFilename, referenceDepthFilename, currentDepthFilename;
  
  // Input parameters handling.
  g2o::CommandArgs arg;

  arg.param("di_scale", di_scale, 1.0f, "Scaling factor to apply on the size of the depth image");
  arg.param("di_scaleFactor", di_scaleFactor, 0.001f, "Depth image values scaling factor");
  arg.param("vz_step", vz_step, 1, "Save in the output file one point each vz_step points");
  arg.param("vz_ellipsoidScale", vz_ellipsoidScale, 100.0f, "Scale for the covariance's ellipsoid");

  // Last parameter has to be the working directory.
  arg.paramLeftOver("configFilename", configFilename, "", "Boss config filename", true);  
  arg.paramLeftOver("referenceDepthFilename", referenceDepthFilename, "", "Reference depth image filename", true);
  arg.paramLeftOver("currentDepthFilename", currentDepthFilename, "", "Current depth image filename", true);

  // Set parser input.
  arg.parseArgs(argc, argv);

  // Load depth images
  IntImage referenceIndex;
  DepthImage referenceDepth, referenceScaledDepth;
  referenceDepth.load(referenceDepthFilename.c_str(), true, di_scaleFactor);
  DepthImage::scale(referenceScaledDepth, referenceDepth, di_scale);
  if (referenceScaledDepth.rows() != referenceIndex.rows() || referenceScaledDepth.cols() != referenceIndex.cols()) {
    referenceIndex.resize(referenceScaledDepth.rows(), referenceScaledDepth.cols());
  }

  IntImage currentIndex;
  DepthImage currentDepth, currentScaledDepth;
  currentDepth.load(currentDepthFilename.c_str(), true, di_scaleFactor);
  DepthImage::scale(currentScaledDepth, currentDepth, di_scale);
  if (currentScaledDepth.rows() != currentIndex.rows() || currentScaledDepth.cols() != currentIndex.cols()) {
    currentIndex.resize(currentScaledDepth.rows(), currentScaledDepth.cols());
  }

  DepthImageConverter *converter = 0;  
  Aligner *aligner = 0;
  std::vector<Serializable*> elements = readConfig(aligner, converter, configFilename);

  Eigen::Isometry3f sensorOffset = Eigen::Isometry3f::Identity();
  sensorOffset.translation() = Vector3f(0.0f, 0.0f, 0.0f);
  Quaternionf quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
  sensorOffset.linear() = quaternion.toRotationMatrix();
  sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  Frame referenceFrame, currentFrame;
  converter->compute(referenceFrame, referenceScaledDepth, sensorOffset);
  //converter->compute(currentFrame, currentScaledDepth, sensorOffset);
  converter->compute(currentFrame, referenceScaledDepth, sensorOffset);
  Isometry3f displacement = Isometry3f::Identity();
  displacement.translation() = Vector3f(0.2f, 0.0f, -0.2f);
  displacement.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  currentFrame.transformInPlace(displacement);

  Eigen::Isometry3f initialGuess = Eigen::Isometry3f::Identity();
  initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  aligner->setReferenceFrame(&referenceFrame);
  aligner->setCurrentFrame(&currentFrame);
  aligner->setInitialGuess(initialGuess);
  aligner->setSensorOffset(sensorOffset);
  aligner->align();  
  std::cout << "Resulting T: " << std::endl << aligner->T().matrix() << std::endl;
  std::cout << "Resulting Omega: " << std::endl << aligner->omega() << std::endl;
  
  Eigen::Matrix3f covariance = aligner->omega().inverse().block<3, 3>(0, 0);
  
  // Create viewer
  QApplication application(argc,argv);
  QWidget* mainWindow = new QWidget();
  mainWindow->setWindowTitle("pwn_dtcovariance_test");
  QHBoxLayout* hlayout = new QHBoxLayout();
  mainWindow->setLayout(hlayout);
  QVBoxLayout* vlayout = new QVBoxLayout();
  hlayout->addItem(vlayout);
  QVBoxLayout* vlayout2 = new QVBoxLayout();
  hlayout->addItem(vlayout2);
  hlayout->setStretch(1,1);

  QListWidget* listWidget = new QListWidget(mainWindow);
  listWidget->setSelectionMode(QAbstractItemView::MultiSelection);
  listWidget->addItem(QString("reference"));
  listWidget->addItem(QString("current"));
  listWidget->addItem(QString("aligned"));
  listWidget->addItem(QString("covariance"));

  vlayout->addWidget(listWidget);
  PWNQGLViewer* viewer = new PWNQGLViewer(mainWindow);
  vlayout2->addWidget(viewer);

  viewer->init();
  mainWindow->show();
  viewer->show();
  listWidget->show();
  viewer->setAxisIsDrawn(true);

  // Reference drawable
  GLParameterPoints *referenceParams = new GLParameterPoints(1.0f, Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
  referenceParams->setStep(vz_step);
  DrawablePoints *referenceDrawable = new DrawablePoints(Isometry3f::Identity(), referenceParams, &referenceFrame.points(), &referenceFrame.normals());
  viewer->addDrawable(referenceDrawable);

  // Current drawable
  GLParameterPoints *currentParams = new GLParameterPoints(1.0f, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f));
  currentParams->setStep(vz_step);
  DrawablePoints *currentDrawable = new DrawablePoints(Isometry3f::Identity(), currentParams, &currentFrame.points(), &currentFrame.normals());
  viewer->addDrawable(currentDrawable);

  // Aligned drawable
  GLParameterPoints *alignedParams = new GLParameterPoints(1.0f, Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
  alignedParams->setStep(vz_step);
  DrawablePoints *alignedDrawable = new DrawablePoints(aligner->T(), alignedParams, &currentFrame.points(), &currentFrame.normals());
  viewer->addDrawable(alignedDrawable);

  // Transform covariance drawable
  GLParameterTransformCovariance *covarianceParams = new GLParameterTransformCovariance(Eigen::Vector4f(1.0f, 0.0f, 1.0f, 1.0f), vz_ellipsoidScale);
  covarianceParams->setStep(vz_step);
  DrawableTransformCovariance *alignedCovariance = new DrawableTransformCovariance(Isometry3f::Identity(), covarianceParams, covariance, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
  viewer->addDrawable(alignedCovariance);

  while (mainWindow->isVisible()) {
    for (size_t i = 0; i < viewer->drawableList().size(); i++) {
      if (listWidget->item(i)->isSelected()) {
	viewer->drawableList()[i]->parameter()->setShow(true);
      }
      else {
	viewer->drawableList()[i]->parameter()->setShow(false);
      }
    }
    
    viewer->updateGL();
    application.processEvents();
  }

  return 0;
}

std::vector<Serializable*> readConfig(Aligner *&aligner, DepthImageConverter *&converter, const std::string &configFile) {
  aligner = 0;
  converter = 0;
  Deserializer des;
  des.setFilePath(configFile);
  Serializable *s;
  std::vector<Serializable*> instances;
  cerr << "Reading configuration file" << endl;
  while ((s=des.readObject())){
    instances.push_back(s);
    Aligner *al = dynamic_cast<Aligner*>(s);
    if (al) {
      cerr << "Got aligner" << endl;
      aligner = al;
    }
    DepthImageConverter *conv = dynamic_cast<DepthImageConverter*>(s);
    if (conv) {      
      cerr << "Got converter" << endl;
      converter = conv;
    }
  }

  return instances;
}
