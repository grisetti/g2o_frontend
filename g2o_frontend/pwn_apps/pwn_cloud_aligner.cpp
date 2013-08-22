#include <dirent.h>
#include <set>
#include <sys/stat.h>
#include <sys/types.h>

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QListWidget>
#include <QCheckBox>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QGraphicsView>

#include "g2o/stuff/command_args.h"

#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/cylindricalpointprojector.h"
#include "g2o_frontend/pwn2/multipointprojector.h"
#include "g2o_frontend/pwn2/statscalculator.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/informationmatrixcalculator.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/pwn2/frame.h"

#include "g2o_frontend/pwn_viewer/pwn_qglviewer.h"
#include "g2o_frontend/pwn_viewer/pwn_imageview.h"
#include "g2o_frontend/pwn_viewer/drawable_points.h"
#include "g2o_frontend/pwn_viewer/drawable_normals.h"
#include "g2o_frontend/pwn_viewer/drawable_covariances.h"
#include "g2o_frontend/pwn_viewer/drawable_correspondences.h"
#include "g2o_frontend/pwn_viewer/drawable_frame.h"
#include "g2o_frontend/pwn_viewer/gl_parameter.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_points.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_normals.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_covariances.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_correspondences.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_frame.h"

using namespace std;
using namespace Eigen;
using namespace g2o;
using namespace pwn;

set<string> readDirectory(string dir);
void applySettings();
void checkProjectorSelection();

// Projectors
PointProjector *projector;
PinholePointProjector pinholePointProjector;
CylindricalPointProjector cylindricalPointProjector;
MultiPointProjector multiPointProjector;
Isometry3f sensorOffset;
Isometry3f referencePose;
Isometry3f currentPose;

// Stats calculator
StatsCalculator statsCalculator;

// Depth image converter
DepthImageConverter converter;
DepthImage depthImage, scaledDepthImage;
MatrixXi indexImage, scaledIndexImage;

// Correspondece finder and linearizer
CorrespondenceFinder correspondenceFinder;
Linearizer linearizer;

// Information matrix calculators
PointInformationMatrixCalculator pointInformationMatrixCalculator;
NormalInformationMatrixCalculator normalInformationMatrixCalculator;

// Aligner
Aligner aligner;

float fx, fy, cx, cy;

Matrix3f cameraMatrix;

QCheckBox *pinholePointProjectorCheckBox, *cylindricalPointProjectorCheckBox, *multiPointProjectorCheckBox;
QSpinBox *ng_minImageRadiusSpinBox, *ng_maxImageRadiusSpinBox, *ng_minPointsSpinBox, *al_innerIterationsSpinBox, *al_outerIterationsSpinBox, *al_minNumInliersSpinBox, *imageRowsSpinBox, *imageColsSpinBox;
QDoubleSpinBox *ng_worldRadiusSpinBox, *ng_scaleSpinBox, *ng_curvatureThresholdSpinBox, *cf_inlierNormalAngularThresholdSpinBox, *cf_flatCurvatureThresholdSpinBox, *cf_inlierCurvatureRatioThresholdSpinBox, *cf_inlierDistanceThresholdSpinBox, *al_minErrorSpinBox, *al_inlierMaxChi2SpinBox, *fxSpinBox, *fySpinBox, *cxSpinBox, *cySpinBox, *angularFOVSpinBox;

int main(int argc, char **argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string working_directory, g2o_filename;

  int ng_minImageRadius, ng_maxImageRadius, ng_minPoints;
  float ng_worldRadius, ng_scale, ng_curvatureThreshold;

  float cf_inlierNormalAngularThreshold, cf_flatCurvatureThreshold, cf_inlierCurvatureRatioThreshold, cf_inlierDistanceThreshold;

  int al_innerIterations, al_outerIterations, al_minNumInliers; 
  float al_minError, al_inlierMaxChi2;

  int vz_step;

  int imageRows, imageCols;
  float angularFOV;

  // Input parameters handling.
  g2o::CommandArgs arg;
  
  // Optional input parameters.
  arg.param("ng_minImageRadius", ng_minImageRadius, 10, "Specify the minimum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_maxImageRadius", ng_maxImageRadius, 30, "Specify the maximum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_minPoints", ng_minPoints, 50, "Specify the minimum number of points to be used to compute a normal");
  arg.param("ng_worldRadius", ng_worldRadius, 0.1f, "Specify the max distance for a point to be used to compute a normal");
  arg.param("ng_scale", ng_scale, 1.0f, "Specify the scaling factor to apply on the depth image");
  arg.param("ng_curvatureThreshold", ng_curvatureThreshold, 1.0f, "Specify the max surface curvature threshold for which normals are discarded");
  
  arg.param("cf_inlierNormalAngularThreshold", cf_inlierNormalAngularThreshold, 1.0f, "Maximum angle between the normals of two points to regard them as iniliers");
  arg.param("cf_flatCurvatureThreshold", cf_flatCurvatureThreshold, 0.02f, "Maximum curvature value for a point to be used for data association");
  arg.param("cf_inlierCurvatureRatioThreshold", cf_inlierCurvatureRatioThreshold, 1.3f, "Maximum curvature ratio value between two points to regard them as iniliers");
  arg.param("cf_inlierDistanceThreshold", cf_inlierDistanceThreshold, 1.0f, "Maximum metric distance between two points to regard them as iniliers");

  arg.param("al_innerIterations", al_innerIterations, 1, "Specify the inner iterations");
  arg.param("al_outerIterations", al_outerIterations, 10, "Specify the outer iterations");
  arg.param("al_minNumInliers", al_minNumInliers, 10000, "Specify the minimum number of inliers to consider an alignment good");
  arg.param("al_minError", al_minError, 10.0f, "Specify the minimum error to consider an alignment good");
  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, 9e3, "Max chi2 error value for the alignment step");
  
  arg.param("vz_step", vz_step, 1, "A graphic element is drawn each vz_step elements");

  arg.param("imageRows", imageRows, 120, "Number of rows of the depth images");
  arg.param("imageCols", imageCols, 320, "Number of columns of the depth images");
  arg.param("angularFOV", angularFOV, M_PI, "Angular field of view for the cylindrical point projector");
  arg.param("fx", fx, 525.0f, "fx value of the camera matrix");
  arg.param("fy", fy, 100.0f, "fy value of the camera matrix");
  arg.param("cx", cx, 319.5f, "cx value of the camera matrix");
  arg.param("cy", cy, 239.5f, "cy value of the camera matrix");

  // Last parameter has to be the working directory.
  arg.paramLeftOver("working_directory", working_directory, ".", "Path of the working directory", true);
  //arg.paramLeftOver("g2o_input_filename", g2o_filename, "", "g2o input filename", false);

  // Set parser input.
  arg.parseArgs(argc, argv);

  /************************************************************************
   *                           Window Creation                            *
   ************************************************************************/
  QApplication application(argc, argv);
  QWidget *mainWindow = new QWidget();
  mainWindow->setWindowTitle("pwn_cloud_aligner");
  QHBoxLayout *mainLayout = new QHBoxLayout();
  mainWindow->setLayout(mainLayout);
  QVBoxLayout *settingsLayout = new QVBoxLayout();
  mainLayout->addItem(settingsLayout);
  QVBoxLayout *viewerLayout = new QVBoxLayout();
  mainLayout->addItem(viewerLayout);
  mainLayout->setStretch(1, 1);

  PWNQGLViewer *viewer = new PWNQGLViewer(mainWindow);
  viewer->init();
  viewer->setAxisIsDrawn(true);
  viewerLayout->addWidget(viewer);
  QHBoxLayout *projectorsLayout = new QHBoxLayout();
  viewerLayout->addItem(projectorsLayout);
  pinholePointProjectorCheckBox = new QCheckBox("Pinhole point projector", mainWindow);
  pinholePointProjectorCheckBox->setChecked(false);
  pinholePointProjectorCheckBox->setEnabled(false);
  cylindricalPointProjectorCheckBox = new QCheckBox("Cylindrical point projector", mainWindow);
  cylindricalPointProjectorCheckBox->setChecked(true);
  cylindricalPointProjectorCheckBox->setEnabled(true);
  multiPointProjectorCheckBox = new QCheckBox("Multiple point projector", mainWindow);
  multiPointProjectorCheckBox->setChecked(false);
  multiPointProjectorCheckBox->setEnabled(false);
  QPushButton *correspondencesButton = new QPushButton("Correspondences", mainWindow);
  projectorsLayout->addWidget(pinholePointProjectorCheckBox);
  projectorsLayout->addWidget(cylindricalPointProjectorCheckBox);
  projectorsLayout->addWidget(multiPointProjectorCheckBox);
  projectorsLayout->addWidget(correspondencesButton);
  QGridLayout *parametersGridLayout = new QGridLayout();
  viewerLayout->addItem(parametersGridLayout);
  QLabel *ng_minImageRadiusLabel = new QLabel("ng_minImageRadius", mainWindow);
  QLabel *ng_maxImageRadiusLabel = new QLabel("ng_maxImageRadius", mainWindow);
  QLabel *ng_minPointsLabel = new QLabel("ng_minPoints", mainWindow);
  QLabel *ng_worldRadiusLabel = new QLabel("ng_worldRadius", mainWindow);
  QLabel *ng_scaleLabel = new QLabel("ng_scale", mainWindow);
  QLabel *ng_curvatureThresholdLabel = new QLabel("ng_curvatureThreshold", mainWindow);
  QLabel *cf_inlierNormalAngularThresholdLabel = new QLabel("cf_inlierNormalAngularThreshold", mainWindow);
  QLabel *cf_flatCurvatureThresholdLabel = new QLabel("cf_flatCurvatureThreshold", mainWindow);
  QLabel *cf_inlierCurvatureRatioThresholdLabel = new QLabel("cf_inlierCurvatureRatioThreshold", mainWindow);
  QLabel *cf_inlierDistanceThresholdLabel = new QLabel("cf_inlierDistanceThreshold", mainWindow);
  QLabel *al_innerIterationsLabel = new QLabel("al_innerIterations", mainWindow);
  QLabel *al_outerIterationsLabel = new QLabel("al_outerIterations", mainWindow);
  QLabel *al_minNumInliersLabel = new QLabel("al_minNumInliers", mainWindow);
  QLabel *al_minErrorLabel = new QLabel("al_minError", mainWindow);
  QLabel *al_inlierMaxChi2Label = new QLabel("al_inlierMaxChi2", mainWindow);
  QLabel *imageRowsLabel = new QLabel("imageRows", mainWindow);
  QLabel *imageColsLabel = new QLabel("imageCols", mainWindow);
  QLabel *angularFOVLabel = new QLabel("angularFOV", mainWindow);
  QLabel *fxLabel = new QLabel("fx", mainWindow);
  QLabel *fyLabel = new QLabel("fy", mainWindow);
  QLabel *cxLabel = new QLabel("cx", mainWindow);
  QLabel *cyLabel = new QLabel("cy", mainWindow);
  ng_minImageRadiusSpinBox = new QSpinBox(mainWindow);
  ng_minImageRadiusSpinBox->setMinimum(0);
  ng_minImageRadiusSpinBox->setSingleStep(1);
  ng_minImageRadiusSpinBox->setValue(ng_minImageRadius);
  ng_maxImageRadiusSpinBox = new QSpinBox(mainWindow);
  ng_maxImageRadiusSpinBox->setMinimum(0);
  ng_maxImageRadiusSpinBox->setSingleStep(1);
  ng_maxImageRadiusSpinBox->setValue(ng_maxImageRadius);
  ng_minPointsSpinBox = new QSpinBox(mainWindow);
  ng_minPointsSpinBox->setMinimum(0);
  ng_minPointsSpinBox->setMaximum(1000);
  ng_minPointsSpinBox->setSingleStep(1);
  ng_minPointsSpinBox->setValue(ng_minPoints);
  ng_worldRadiusSpinBox = new QDoubleSpinBox(mainWindow);
  ng_worldRadiusSpinBox->setMinimum(0.0f);
  ng_worldRadiusSpinBox->setSingleStep(0.01f);
  ng_worldRadiusSpinBox->setValue(ng_worldRadius);
  ng_scaleSpinBox = new QDoubleSpinBox(mainWindow);
  ng_scaleSpinBox->setMinimum(1.0f);
  ng_scaleSpinBox->setSingleStep(1.0f);
  ng_scaleSpinBox->setValue(ng_scale);
  ng_curvatureThresholdSpinBox = new QDoubleSpinBox(mainWindow);
  ng_curvatureThresholdSpinBox->setMinimum(0.0f);
  ng_curvatureThresholdSpinBox->setSingleStep(0.01f);
  ng_curvatureThresholdSpinBox->setValue(ng_curvatureThreshold);
  cf_inlierNormalAngularThresholdSpinBox = new QDoubleSpinBox(mainWindow);
  cf_inlierNormalAngularThresholdSpinBox->setMinimum(0.0f);
  cf_inlierNormalAngularThresholdSpinBox->setSingleStep(0.01f);
  cf_inlierNormalAngularThresholdSpinBox->setValue(cf_inlierNormalAngularThreshold);
  cf_flatCurvatureThresholdSpinBox = new QDoubleSpinBox(mainWindow);
  cf_flatCurvatureThresholdSpinBox->setMinimum(0.0f);
  cf_flatCurvatureThresholdSpinBox->setSingleStep(0.01f);
  cf_flatCurvatureThresholdSpinBox->setValue(cf_flatCurvatureThreshold);
  cf_inlierCurvatureRatioThresholdSpinBox = new QDoubleSpinBox(mainWindow);
  cf_inlierCurvatureRatioThresholdSpinBox->setMinimum(0.0f);
  cf_inlierCurvatureRatioThresholdSpinBox->setSingleStep(0.01f);
  cf_inlierCurvatureRatioThresholdSpinBox->setValue(cf_inlierCurvatureRatioThreshold);
  cf_inlierDistanceThresholdSpinBox = new QDoubleSpinBox(mainWindow);
  cf_inlierDistanceThresholdSpinBox->setMinimum(0.0f);
  cf_inlierDistanceThresholdSpinBox->setSingleStep(0.01f);
  cf_inlierDistanceThresholdSpinBox->setValue(cf_inlierDistanceThreshold);
  al_innerIterationsSpinBox = new QSpinBox(mainWindow);
  al_innerIterationsSpinBox->setMinimum(0);
  al_innerIterationsSpinBox->setSingleStep(1);
  al_innerIterationsSpinBox->setValue(al_innerIterations);
  al_outerIterationsSpinBox = new QSpinBox(mainWindow);
  al_outerIterationsSpinBox->setMinimum(0);
  al_outerIterationsSpinBox->setSingleStep(1);
  al_outerIterationsSpinBox->setValue(al_outerIterations);
  al_minNumInliersSpinBox = new QSpinBox(mainWindow);
  al_minNumInliersSpinBox->setMinimum(0);
  al_minNumInliersSpinBox->setMaximum(100000);
  al_minNumInliersSpinBox->setSingleStep(1);
  al_minNumInliersSpinBox->setValue(al_minNumInliers);
  al_minErrorSpinBox = new QDoubleSpinBox(mainWindow);
  al_minErrorSpinBox->setMinimum(0.0f);
  al_minErrorSpinBox->setSingleStep(0.01f);
  al_minErrorSpinBox->setValue(al_minError);
  al_inlierMaxChi2SpinBox = new QDoubleSpinBox(mainWindow);
  al_inlierMaxChi2SpinBox->setMinimum(0.0f);
  al_inlierMaxChi2SpinBox->setMaximum(100000.0f);
  al_inlierMaxChi2SpinBox->setSingleStep(0.01f);
  al_inlierMaxChi2SpinBox->setValue(al_inlierMaxChi2);
  imageRowsSpinBox = new QSpinBox(mainWindow);
  imageRowsSpinBox->setMinimum(0);
  imageRowsSpinBox->setMaximum(10000);
  imageRowsSpinBox->setSingleStep(1);
  imageRowsSpinBox->setValue(imageRows);
  imageColsSpinBox = new QSpinBox(mainWindow);
  imageColsSpinBox->setMinimum(0);
  imageColsSpinBox->setMaximum(10000);
  imageColsSpinBox->setSingleStep(1);
  imageColsSpinBox->setValue(imageCols);
  angularFOVSpinBox = new QDoubleSpinBox(mainWindow);
  angularFOVSpinBox->setMinimum(0.0f);
  angularFOVSpinBox->setSingleStep(0.01f);
  angularFOVSpinBox->setValue(angularFOV);
  fxSpinBox = new QDoubleSpinBox(mainWindow);
  fxSpinBox->setMinimum(0.0f);
  fxSpinBox->setMaximum(1000.0f);
  fxSpinBox->setSingleStep(0.01f);
  fxSpinBox->setValue(fx);
  fySpinBox = new QDoubleSpinBox(mainWindow);
  fySpinBox->setMinimum(0.0f);
  fySpinBox->setMaximum(1000.0f);
  fySpinBox->setSingleStep(0.01f);
  fySpinBox->setValue(fy);
  cxSpinBox = new QDoubleSpinBox(mainWindow);
  cxSpinBox->setMinimum(0.0f);
  cxSpinBox->setMaximum(1000.0f);
  cxSpinBox->setSingleStep(0.01f);
  cxSpinBox->setValue(cx);
  cySpinBox = new QDoubleSpinBox(mainWindow);
  cySpinBox->setMinimum(0.0f);
  cySpinBox->setMaximum(1000.0f);
  cySpinBox->setSingleStep(0.01f);
  cySpinBox->setValue(cy);
  parametersGridLayout->addWidget(ng_minImageRadiusLabel, 0, 0, Qt::AlignLeft);
  parametersGridLayout->addWidget(ng_minImageRadiusSpinBox, 0, 1, Qt::AlignLeft);
  parametersGridLayout->addWidget(ng_maxImageRadiusLabel, 1, 0, Qt::AlignLeft);
  parametersGridLayout->addWidget(ng_maxImageRadiusSpinBox, 1, 1, Qt::AlignLeft);
  parametersGridLayout->addWidget(ng_minPointsLabel, 2, 0, Qt::AlignLeft);
  parametersGridLayout->addWidget(ng_minPointsSpinBox, 2, 1, Qt::AlignLeft);
  parametersGridLayout->addWidget(ng_worldRadiusLabel, 3, 0, Qt::AlignLeft);
  parametersGridLayout->addWidget(ng_worldRadiusSpinBox, 3, 1, Qt::AlignLeft);
  parametersGridLayout->addWidget(ng_scaleLabel, 4, 0, Qt::AlignLeft);  
  parametersGridLayout->addWidget(ng_scaleSpinBox, 4, 1, Qt::AlignLeft);
  parametersGridLayout->addWidget(ng_curvatureThresholdLabel, 5, 0, Qt::AlignLeft);
  parametersGridLayout->addWidget(ng_curvatureThresholdSpinBox, 5, 1, Qt::AlignLeft);
  parametersGridLayout->addWidget(cf_inlierNormalAngularThresholdLabel, 0, 2, Qt::AlignLeft);
  parametersGridLayout->addWidget(cf_inlierNormalAngularThresholdSpinBox, 0, 3, Qt::AlignLeft);
  parametersGridLayout->addWidget(cf_flatCurvatureThresholdLabel, 1, 2, Qt::AlignLeft);
  parametersGridLayout->addWidget(cf_flatCurvatureThresholdSpinBox, 1, 3, Qt::AlignLeft);
  parametersGridLayout->addWidget(cf_inlierCurvatureRatioThresholdLabel, 2, 2, Qt::AlignLeft);
  parametersGridLayout->addWidget(cf_inlierCurvatureRatioThresholdSpinBox, 2, 3, Qt::AlignLeft);
  parametersGridLayout->addWidget(cf_inlierDistanceThresholdLabel, 3, 2, Qt::AlignLeft);
  parametersGridLayout->addWidget(cf_inlierDistanceThresholdSpinBox, 3, 3, Qt::AlignLeft);
  parametersGridLayout->addWidget(al_innerIterationsLabel, 0, 4, Qt::AlignLeft);
  parametersGridLayout->addWidget(al_innerIterationsSpinBox, 0, 5, Qt::AlignLeft);
  parametersGridLayout->addWidget(al_outerIterationsLabel, 1, 4, Qt::AlignLeft);
  parametersGridLayout->addWidget(al_outerIterationsSpinBox, 1, 5, Qt::AlignLeft);
  parametersGridLayout->addWidget(al_minNumInliersLabel, 2, 4, Qt::AlignLeft);
  parametersGridLayout->addWidget(al_minNumInliersSpinBox, 2, 5, Qt::AlignLeft);
  parametersGridLayout->addWidget(al_minErrorLabel, 3, 4, Qt::AlignLeft);
  parametersGridLayout->addWidget(al_minErrorSpinBox, 3, 5, Qt::AlignLeft);
  parametersGridLayout->addWidget(al_inlierMaxChi2Label, 4, 4, Qt::AlignLeft);
  parametersGridLayout->addWidget(al_inlierMaxChi2SpinBox, 4, 5, Qt::AlignLeft);
  parametersGridLayout->addWidget(imageRowsLabel, 0, 6, Qt::AlignLeft);
  parametersGridLayout->addWidget(imageRowsSpinBox, 0, 7, Qt::AlignLeft);
  parametersGridLayout->addWidget(imageColsLabel, 1, 6, Qt::AlignLeft);
  parametersGridLayout->addWidget(imageColsSpinBox, 1, 7, Qt::AlignLeft);
  parametersGridLayout->addWidget(angularFOVLabel, 2, 6, Qt::AlignLeft);
  parametersGridLayout->addWidget(angularFOVSpinBox, 2, 7, Qt::AlignLeft);
  parametersGridLayout->addWidget(fxLabel, 3, 6, Qt::AlignLeft);
  parametersGridLayout->addWidget(fxSpinBox, 3, 7, Qt::AlignLeft);
  parametersGridLayout->addWidget(fyLabel, 4, 6, Qt::AlignLeft);
  parametersGridLayout->addWidget(fySpinBox, 4, 7, Qt::AlignLeft);
  parametersGridLayout->addWidget(cxLabel, 5, 6, Qt::AlignLeft);
  parametersGridLayout->addWidget(cxSpinBox, 5, 7, Qt::AlignLeft);
  parametersGridLayout->addWidget(cyLabel, 6, 6, Qt::AlignLeft);
  parametersGridLayout->addWidget(cySpinBox, 6, 7, Qt::AlignLeft);

  QLabel *stepLabel = new QLabel("Step", mainWindow);
  QLabel *pointsLabel = new QLabel("Points", mainWindow);
  QLabel *normalsLabel = new QLabel("Normals", mainWindow);
  QLabel *covariancesLabel = new QLabel("Covariances", mainWindow);
  QLabel *correspondencesLabel = new QLabel("Correspondences", mainWindow);
  QCheckBox *stepByStepCheckBox = new QCheckBox("Step-By-Step", mainWindow);
  QSpinBox *stepSpinBox = new QSpinBox(mainWindow);
  stepSpinBox->setMinimum(1);
  stepSpinBox->setSingleStep(1);
  stepSpinBox->setValue(1);
  QDoubleSpinBox *pointsSpinBox = new QDoubleSpinBox(mainWindow);
  pointsSpinBox->setMinimum(0.0f);
  pointsSpinBox->setSingleStep(0.25f);
  pointsSpinBox->setValue(1.0f);
  QDoubleSpinBox *normalsSpinBox = new QDoubleSpinBox(mainWindow);
  normalsSpinBox->setMinimum(0.0f);
  normalsSpinBox->setSingleStep(0.01f);
  QDoubleSpinBox *covariancesSpinBox = new QDoubleSpinBox(mainWindow);
  covariancesSpinBox->setMinimum(0.0f);
  covariancesSpinBox->setSingleStep(0.01f);
  QDoubleSpinBox *correspondencesSpinBox = new QDoubleSpinBox(mainWindow);
  correspondencesSpinBox->setMinimum(0.0f);
  correspondencesSpinBox->setSingleStep(0.25f);
  QPushButton *addCloudButton = new QPushButton("Add Cloud", mainWindow);
  QGridLayout *gridLayout1 = new QGridLayout();
  settingsLayout->addItem(gridLayout1);
  gridLayout1->addWidget(stepLabel, 0, 0);
  gridLayout1->addWidget(stepSpinBox, 0, 1);
  gridLayout1->addWidget(pointsLabel, 1, 0);
  gridLayout1->addWidget(pointsSpinBox, 1, 1);
  gridLayout1->addWidget(normalsLabel, 2, 0);
  gridLayout1->addWidget(normalsSpinBox, 2, 1);
  gridLayout1->addWidget(covariancesLabel, 3, 0);
  gridLayout1->addWidget(covariancesSpinBox, 3, 1);
  gridLayout1->addWidget(correspondencesLabel, 4, 0);
  gridLayout1->addWidget(correspondencesSpinBox, 4, 1);
  gridLayout1->addWidget(stepByStepCheckBox, 5, 0);
  gridLayout1->addWidget(addCloudButton, 5, 1);

  QHBoxLayout *listWidgetLayout = new QHBoxLayout();
  settingsLayout->addItem(listWidgetLayout);
  QListWidget *listWidget = new QListWidget(mainWindow);
  listWidgetLayout->addWidget(listWidget);
  
  QGridLayout *gridLayout2 = new QGridLayout();
  settingsLayout->addItem(gridLayout2);
  QPushButton *initialGuessButton = new QPushButton("Initial Guess", mainWindow);
  QPushButton *clearLastButton = new QPushButton("Clear Last", mainWindow);
  QPushButton *optimizeButton = new QPushButton("Optimize", mainWindow);
  QPushButton *clearAllButton = new QPushButton("Clear All", mainWindow);
  gridLayout2->addWidget(initialGuessButton, 0, 0);
  gridLayout2->addWidget(clearLastButton, 0, 1);
  gridLayout2->addWidget(optimizeButton, 1, 0);
  gridLayout2->addWidget(clearAllButton, 1, 1);
  
  QHBoxLayout *referenceDepthViewLayout = new QHBoxLayout();
  settingsLayout->addItem(referenceDepthViewLayout);
  QGraphicsView *referenceDepthView = new QGraphicsView(mainWindow);
  referenceDepthViewLayout->addWidget(referenceDepthView);

  QHBoxLayout *currentDepthViewLayout = new QHBoxLayout();
  settingsLayout->addItem(currentDepthViewLayout);
  QGraphicsView *currentDepthView = new QGraphicsView(mainWindow);
  currentDepthViewLayout->addWidget(currentDepthView);  
  settingsLayout->setStretch(1, 1);

  /************************************************************************
   *                     Working Directory Parsing                        *
   ************************************************************************/
  vector<string> filenames;
  set<string> filenamesSet = readDirectory(working_directory);
  for(set<string>::const_iterator it = filenamesSet.begin(); it != filenamesSet.end(); it++) {
    filenames.push_back(*it);       
    QString listItem(&(*it)[0]);
    if(listItem.endsWith(".pwn", Qt::CaseInsensitive))
      listWidget->addItem(listItem);
  }

  mainWindow->show();
  viewer->show();
  listWidget->show();
  QGraphicsScene *referenceScene = new QGraphicsScene();
  QGraphicsScene *currentScene = new QGraphicsScene();
  referenceDepthView->setScene(referenceScene);
  currentDepthView->setScene(currentScene);
  Frame *referenceFrame = 0, *currentFrame = 0;
  string referenceFilename, currentFilename;
  GLParameterFrame *referenceParameterFrame = new GLParameterFrame(vz_step);
  GLParameterFrame *currentParameterFrame = new GLParameterFrame(vz_step);
  Isometry3f initialGuess = Isometry3f::Identity();
  initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  while(mainWindow->isVisible()) {
    // Control that at most one projector is selected
    checkProjectorSelection();
    // Apply possible new alignment settings
    applySettings();

    // Apply possible new visualization settings
    for(size_t i = 0; i < viewer->drawableList().size(); i++) {
      Drawable *lastDrawable = viewer->drawableList()[i];
      DrawableFrame *lastDrawableFrame = dynamic_cast<DrawableFrame*>(lastDrawable);
      GLParameter *parameter = lastDrawableFrame->parameter();
      GLParameterFrame *parameterFrame = dynamic_cast<GLParameterFrame*>(parameter);
      if(lastDrawableFrame && parameterFrame) {
	lastDrawableFrame->drawablePoints()->setStep(stepSpinBox->value());
	lastDrawableFrame->drawablePoints()->setStep(stepSpinBox->value());
	lastDrawableFrame->drawableNormals()->setStep(stepSpinBox->value());
	lastDrawableFrame->drawableCovariances()->setStep(stepSpinBox->value());
	lastDrawableFrame->drawableCorrespondences()->setStep(stepSpinBox->value());
	parameterFrame->parameterPoints()->setPointSize(pointsSpinBox->value());
	parameterFrame->parameterNormals()->setNormalLength(normalsSpinBox->value());
	parameterFrame->parameterCovariances()->setEllipsoidScale(covariancesSpinBox->value());
	parameterFrame->parameterCorrespondences()->setLineWidth(correspondencesSpinBox->value());
      }
    }

    // Add cloud button pressed
    if(addCloudButton->isDown()) {
      addCloudButton->setEnabled(false);

      for(int k = 0; k < listWidget->count(); k++) {
	QListWidgetItem* item = listWidget->item(k);
	if(item) {
	  if(item->isSelected()) {
	    Frame *frame = 0;
	    if(!referenceFrame) {
	      referenceFrame = new Frame();
	      frame = referenceFrame;
	    }
	    else if(!currentFrame) {
	      currentFrame = new Frame();
	      frame = currentFrame;
	    }
	    else {
	      cerr << "WARNING: you are allowed to work with maximum two clouds at time!" << endl;
	      break;
	    }

	    string fname = item->text().toUtf8().constData();
	    Eigen::Isometry3f originPose = Eigen::Isometry3f::Identity();
	    originPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
	    // Load the cloud from the .pwn file
	    if(!frame->load(originPose, fname.c_str())) {      
	      cerr << "WARNING: the cloud was not added in the viewer because of a problem while loading it!" << endl;
	      frame = 0;
	      if(referenceFrame) {
		delete referenceFrame;
		referenceFrame = 0;
	      }
	      else {
		delete currentFrame;
		currentFrame = 0;
	      }
	    }
	    else {
	      originPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
	     // Computing information matrices
	      frame->pointInformationMatrix().resize(frame->points().size());
	      frame->normalInformationMatrix().resize(frame->points().size());
	      pointInformationMatrixCalculator.compute(frame->pointInformationMatrix(), frame->stats(), frame->normals());
	      normalInformationMatrixCalculator.compute(frame->normalInformationMatrix(), frame->stats(), frame->normals());
	      if(currentFrame) {
		currentPose=originPose;
		initialGuess = referencePose.inverse()*currentPose;
		DrawableFrame *currentDrawableFrame = new DrawableFrame(initialGuess, currentParameterFrame, currentFrame);
		//currentDrawableFrame->setTransformation(Isometry3f::Identity());
		viewer->addDrawable(currentDrawableFrame);
		currentFilename = fname;
	      }
	      else {
		DrawableFrame *referenceDrawableFrame = new DrawableFrame(Isometry3f::Identity(), referenceParameterFrame, referenceFrame);
		//referenceDrawableFrame->setTransformation(Isometry3f::Identity());
		viewer->addDrawable(referenceDrawableFrame);
		referenceFilename = fname;
		referencePose = originPose;
	      }	      
	    }
	    break;
	  }
	}
      }
      addCloudButton->setEnabled(true);
    }

    // Clear last button pressed
    if(clearLastButton->isDown()) {
      clearLastButton->setEnabled(false);
      if(viewer->drawableList().size() > 0) {
	Drawable *lastDrawable = viewer->drawableList().back();
	viewer->popBack();
	delete lastDrawable;
	
	if(currentFrame) {
	  delete currentFrame;
	  currentFrame = 0;
	}
	else {
	  delete referenceFrame;
	  referenceFrame = 0;
	}	
      }
      else {
	cerr << "WARNING: no cloud was removed since the list is empty!" << endl;
      }

      referenceScene->clear();
      currentScene->clear();

      clearLastButton->setEnabled(true);
    }

    // Clear all button pressed
    if(clearAllButton->isDown()) {
      clearAllButton->setEnabled(false);
      
      int listSize = viewer->drawableList().size();
      if(viewer->drawableList().size() > 0) {
	for(int i = 0; i < listSize; i++) {
	  Drawable *lastDrawable = viewer->drawableList().back();
	  viewer->popBack();
	  delete lastDrawable;	
	}
      }
      else {
	cerr << "WARNING: no cloud was removed since the list is empty!" << endl;
      }

      if(currentFrame) {
	delete currentFrame;
	currentFrame = 0;
      }
      if(referenceFrame) {
	delete referenceFrame;
	referenceFrame = 0;
      }
      
      referenceScene->clear();
      currentScene->clear();

      clearAllButton->setEnabled(true);
    }

    // Correspondences button pressed
    if(correspondencesButton->isDown() && referenceFrame && currentFrame) {
      correspondencesButton->setEnabled(false);
      projector->setTransform(sensorOffset);
      projector->project(correspondenceFinder.currentIndexImage(),
			 correspondenceFinder.currentDepthImage(),
			 currentFrame->points());
        
      projector->setTransform(initialGuess * sensorOffset);
      projector->project(correspondenceFinder.referenceIndexImage(),
			  correspondenceFinder.referenceDepthImage(),
			  referenceFrame->points());

      correspondenceFinder.compute(*referenceFrame, *currentFrame, aligner.T().inverse());

      Drawable *lastDrawable = viewer->drawableList().back();
      DrawableFrame *lastDrawableFrame = dynamic_cast<DrawableFrame*>(lastDrawable);
      if(lastDrawableFrame) {
	lastDrawableFrame->clearDrawableObjects();
	lastDrawableFrame->constructDrawableObjects();
	DrawableCorrespondences *drawableCorrespondences = lastDrawableFrame->drawableCorrespondences();
	drawableCorrespondences->setTransformation(Isometry3f::Identity());
	drawableCorrespondences->setReferencePointsTransformation(initialGuess.inverse());
	drawableCorrespondences->setReferencePoints(&referenceFrame->points());
	drawableCorrespondences->setCurrentPoints(&currentFrame->points());
	drawableCorrespondences->setNumCorrespondences(correspondenceFinder.numCorrespondences()); 
	drawableCorrespondences->setCorrespondences(&correspondenceFinder.correspondences());
      }
      
      // Show zBuffers.
      referenceScene->clear();
      currentScene->clear();
      QImage referenceQImage;
      QImage currentQImage;
      DepthImageView div;
      div.computeColorMap(300, 2000, 128);
      div.convertToQImage(referenceQImage, correspondenceFinder.referenceDepthImage());
      div.convertToQImage(currentQImage, correspondenceFinder.currentDepthImage());
      referenceScene->addPixmap(QPixmap::fromImage(referenceQImage));
      currentScene->addPixmap(QPixmap::fromImage(currentQImage));
      referenceDepthView->fitInView(referenceScene->itemsBoundingRect(), Qt::KeepAspectRatio);
      currentDepthView->fitInView(currentScene->itemsBoundingRect(), Qt::KeepAspectRatio);
      referenceDepthView->show();
      currentDepthView->show();

      correspondencesButton->setEnabled(true);
    }

    // Optimize button pressed without step-by-step mode selected
    if(optimizeButton->isDown() && referenceFrame && currentFrame && !stepByStepCheckBox->isChecked()) {
      optimizeButton->setEnabled(false);
      
      // Setting aligner
      //initialGuess = Isometry3f::Identity();
      initialGuess = referencePose.inverse()*currentPose;
      initialGuess.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      aligner.clearPriors();
      aligner.setOuterIterations(al_outerIterationsSpinBox->value());
      aligner.setInnerIterations(al_innerIterationsSpinBox->value());
      aligner.setReferenceFrame(referenceFrame);
      aligner.setCurrentFrame(currentFrame);
      aligner.setInitialGuess(initialGuess);
      aligner.setSensorOffset(sensorOffset);
      
      // Align
      aligner.align();  
      
      Drawable *lastDrawable = viewer->drawableList().back();
      DrawableFrame *lastDrawableFrame = dynamic_cast<DrawableFrame*>(lastDrawable);
      if(lastDrawableFrame) {
	lastDrawableFrame->setTransformation(aligner.T());
	lastDrawableFrame->clearDrawableObjects();
	lastDrawableFrame->constructDrawableObjects();	
	lastDrawableFrame->drawableCorrespondences()->setReferencePointsTransformation(aligner.T().inverse());
	lastDrawableFrame->drawableCorrespondences()->setReferencePoints(&referenceFrame->points());
	lastDrawableFrame->drawableCorrespondences()->setCurrentPoints(&currentFrame->points());
	lastDrawableFrame->drawableCorrespondences()->setCorrespondences(&correspondenceFinder.correspondences());
	lastDrawableFrame->drawableCorrespondences()->setNumCorrespondences(correspondenceFinder.numCorrespondences());
      }

      // Show zBuffers.
      referenceScene->clear();
      currentScene->clear();
      QImage referenceQImage;
      QImage currentQImage;
      DepthImageView div;
      div.computeColorMap(300, 2000, 128);
      div.convertToQImage(referenceQImage, correspondenceFinder.referenceDepthImage());
      div.convertToQImage(currentQImage, correspondenceFinder.currentDepthImage());
      referenceScene->addPixmap(QPixmap::fromImage(referenceQImage));
      currentScene->addPixmap(QPixmap::fromImage(currentQImage));
      referenceDepthView->fitInView(referenceScene->itemsBoundingRect(), Qt::KeepAspectRatio);
      currentDepthView->fitInView(currentScene->itemsBoundingRect(), Qt::KeepAspectRatio);
      referenceDepthView->show();
      currentDepthView->show();

      initialGuess = aligner.T();

      optimizeButton->setEnabled(true);
    }

    // Optimize button pressed with step-by-step mode selected
    if(optimizeButton->isDown() && referenceFrame && currentFrame && stepByStepCheckBox->isChecked()) {
      optimizeButton->setEnabled(false);
      
      // Setting aligner
      aligner.clearPriors();
      aligner.setOuterIterations(1);
      aligner.setInnerIterations(al_innerIterationsSpinBox->value());
      aligner.setReferenceFrame(referenceFrame);
      aligner.setCurrentFrame(currentFrame);
      aligner.setInitialGuess(initialGuess);
      aligner.setSensorOffset(sensorOffset);
      
      // Align
      aligner.align();  
      
      Drawable *lastDrawable = viewer->drawableList().back();
      DrawableFrame *lastDrawableFrame = dynamic_cast<DrawableFrame*>(lastDrawable);
      if(lastDrawableFrame) {
	lastDrawableFrame->setTransformation(aligner.T());
	lastDrawableFrame->clearDrawableObjects();
	lastDrawableFrame->constructDrawableObjects();	
	lastDrawableFrame->drawableCorrespondences()->setReferencePointsTransformation(aligner.T().inverse());
	lastDrawableFrame->drawableCorrespondences()->setReferencePoints(&referenceFrame->points());
	lastDrawableFrame->drawableCorrespondences()->setCurrentPoints(&currentFrame->points());
	lastDrawableFrame->drawableCorrespondences()->setCorrespondences(&correspondenceFinder.correspondences());
	lastDrawableFrame->drawableCorrespondences()->setNumCorrespondences(correspondenceFinder.numCorrespondences());
      }

      // Show zBuffers.
      referenceScene->clear();
      currentScene->clear();
      QImage referenceQImage;
      QImage currentQImage;
      DepthImageView div;
      div.computeColorMap(300, 2000, 128);
      div.convertToQImage(referenceQImage, correspondenceFinder.referenceDepthImage());
      div.convertToQImage(currentQImage, correspondenceFinder.currentDepthImage());
      referenceScene->addPixmap(QPixmap::fromImage(referenceQImage));
      currentScene->addPixmap(QPixmap::fromImage(currentQImage));
      referenceDepthView->fitInView(referenceScene->itemsBoundingRect(), Qt::KeepAspectRatio);
      currentDepthView->fitInView(currentScene->itemsBoundingRect(), Qt::KeepAspectRatio);
      referenceDepthView->show();
      currentDepthView->show();
      initialGuess = aligner.T();

      optimizeButton->setEnabled(true);
    }

    // Initial guess button pressed
    if(initialGuessButton->isDown() && referenceFrame && currentFrame) {
      initialGuess = referencePose.inverse()*currentPose;
      Drawable *lastDrawable = viewer->drawableList().back();
      DrawableFrame *lastDrawableFrame = dynamic_cast<DrawableFrame*>(lastDrawable);
      if(lastDrawableFrame) {
	lastDrawableFrame->clearDrawableObjects();
	lastDrawableFrame->constructDrawableObjects();	
	lastDrawableFrame->setTransformation(initialGuess);
	lastDrawableFrame->drawableCorrespondences()->setReferencePointsTransformation(initialGuess.inverse());
	lastDrawableFrame->drawableCorrespondences()->setReferencePoints(&referenceFrame->points());
	lastDrawableFrame->drawableCorrespondences()->setCurrentPoints(&currentFrame->points());
	lastDrawableFrame->drawableCorrespondences()->setCorrespondences(&correspondenceFinder.correspondences());
	lastDrawableFrame->drawableCorrespondences()->setNumCorrespondences(correspondenceFinder.numCorrespondences());
      }
      initialGuessButton->setEnabled(true);
    }

    viewer->updateGL();
    application.processEvents();
  }

  return 0;
}

set<string> readDirectory(string dir) {
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  std::set<std::string> filenames;
  dp = opendir(dir.c_str());
  if(dp == NULL) {
    return filenames;
  }
  
  while((dirp = readdir(dp))) {
    string filepath = dir + "/" + dirp->d_name;

    // If the file is a directory (or is in some way invalid) we'll skip it 
    if(stat(filepath.c_str(), &filestat)) 
      continue;
    if(S_ISDIR(filestat.st_mode))         
      continue;

    filenames.insert(filepath);
  }

  closedir(dp);
  
  return filenames;
}

void applySettings() {
  sensorOffset = Isometry3f::Identity();
  sensorOffset.translation() = Vector3f(0.0f, 0.0f, 0.0f);
  Quaternionf quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
  sensorOffset.linear() = quaternion.toRotationMatrix();
  sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  
  // Compute the reduced camera matrix and image size
  cameraMatrix << 
    fx, 0.0f, cx,
    0.0f, fy, cy,
    0.0f, 0.0f, 1.0f;  
  float scale = 1.0f / ng_scaleSpinBox->value();
  cameraMatrix = cameraMatrix * scale;
  cameraMatrix(2, 2) = 1.0f;
  if(pinholePointProjectorCheckBox->isChecked()) {
    fxSpinBox->setValue(cameraMatrix(0, 0));
    fySpinBox->setValue(cameraMatrix(1, 1));
    cxSpinBox->setValue(cameraMatrix(0, 2));
    cySpinBox->setValue(cameraMatrix(1, 2));
  }
  
  pinholePointProjector.setCameraMatrix(cameraMatrix);
  float angularResolution = imageColsSpinBox->value() / (2.0f * angularFOVSpinBox->value());
  cylindricalPointProjector.setAngularFov(angularFOVSpinBox->value());
  cylindricalPointProjector.setAngularResolution(angularResolution);
  cylindricalPointProjector.setVerticalCenter(imageRowsSpinBox->value() / 2.0f);
  cylindricalPointProjector.setVerticalFocalLenght(fySpinBox->value());
  
  statsCalculator.setWorldRadius(ng_minImageRadiusSpinBox->value());
  statsCalculator.setMinImageRadius(ng_maxImageRadiusSpinBox->value());
  statsCalculator.setMinImageRadius(ng_minPointsSpinBox->value());
  statsCalculator.setWorldRadius(ng_worldRadiusSpinBox->value());

  pointInformationMatrixCalculator.setCurvatureThreshold(ng_curvatureThresholdSpinBox->value());
  normalInformationMatrixCalculator.setCurvatureThreshold(ng_curvatureThresholdSpinBox->value());

  if(multiPointProjectorCheckBox->isChecked() == true)
    converter = DepthImageConverter(&multiPointProjector, &statsCalculator, 
				    &pointInformationMatrixCalculator, &normalInformationMatrixCalculator);
  else if(cylindricalPointProjectorCheckBox->isChecked() == true)
    converter = DepthImageConverter(&cylindricalPointProjector, &statsCalculator, 
				    &pointInformationMatrixCalculator, &normalInformationMatrixCalculator);  
  else
    converter = DepthImageConverter(&pinholePointProjector, &statsCalculator, 
				    &pointInformationMatrixCalculator, &normalInformationMatrixCalculator);

  converter._curvatureThreshold = ng_curvatureThresholdSpinBox->value();

  correspondenceFinder.setInlierDistanceThreshold(cf_inlierDistanceThresholdSpinBox->value());
  correspondenceFinder.setFlatCurvatureThreshold(cf_flatCurvatureThresholdSpinBox->value());  
  correspondenceFinder.setInlierCurvatureRatioThreshold(cf_inlierCurvatureRatioThresholdSpinBox->value());
  correspondenceFinder.setInlierNormalAngularThreshold(cosf(cf_inlierNormalAngularThresholdSpinBox->value()));
  correspondenceFinder.setSize(imageColsSpinBox->value(), imageRowsSpinBox->value());
  
  linearizer.setInlierMaxChi2(al_inlierMaxChi2SpinBox->value());

  if(multiPointProjectorCheckBox->isChecked() == true)
    aligner.setProjector(&multiPointProjector);
  else if(cylindricalPointProjectorCheckBox->isChecked() == true)
    aligner.setProjector(&cylindricalPointProjector);
  else
    aligner.setProjector(&pinholePointProjector);
  aligner.setLinearizer(&linearizer);
  linearizer.setAligner(&aligner);
  aligner.setCorrespondenceFinder(&correspondenceFinder);
  aligner.setInnerIterations(al_innerIterationsSpinBox->value());
  aligner.setOuterIterations(al_outerIterationsSpinBox->value());
}

void checkProjectorSelection() {
  if(pinholePointProjectorCheckBox->isChecked()) {
    projector = &pinholePointProjector;
    cylindricalPointProjectorCheckBox->setChecked(false);
    cylindricalPointProjectorCheckBox->setEnabled(false);
    multiPointProjectorCheckBox->setChecked(false);
    multiPointProjectorCheckBox->setEnabled(false);
  }
  else if(cylindricalPointProjectorCheckBox->isChecked()) {
    projector = &cylindricalPointProjector;
    pinholePointProjectorCheckBox->setChecked(false);
    pinholePointProjectorCheckBox->setEnabled(false);
    multiPointProjectorCheckBox->setChecked(false);
    multiPointProjectorCheckBox->setEnabled(false);
  }
  // else if(multiPointProjectorCheckBox.isChecked()) {
  //   projector = &multiPointProjector;
  //   pinholePointProjectorCheckBox->setChecked(false);
  //   pinholePointProjectorCheckBox->setEnabled(false);
  //   cylindricalPointProjectorCheckBox->setChecked(false);
  //   cylindricalPointProjectorCheckBox->setEnabled(false);
  // }
  else {
    projector = &pinholePointProjector;
    pinholePointProjectorCheckBox->setChecked(false);
    pinholePointProjectorCheckBox->setEnabled(true);
    cylindricalPointProjectorCheckBox->setChecked(false);
    cylindricalPointProjectorCheckBox->setEnabled(true);
    multiPointProjectorCheckBox->setChecked(false);
    multiPointProjectorCheckBox->setEnabled(false);
  }
}
