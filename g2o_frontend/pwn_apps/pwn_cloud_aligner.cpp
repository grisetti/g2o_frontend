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

#include "g2o_frontend/pwn2/cylindricalpointprojector.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/pwn2/frame.h"

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

using namespace std;
using namespace Eigen;
using namespace g2o;
using namespace pwn;

set<string> readDirectory(string dir);

int main(int argc, char **argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string working_directory;

  int ng_minImageRadius;
  int ng_maxImageRadius;
  int ng_minPoints;
  float ng_worldRadius;
  float ng_scale;
  float ng_curvatureThreshold;
  
  float cf_inlierNormalAngularThreshold;
  float cf_flatCurvatureThreshold;
  float cf_inlierCurvatureRatioThreshold;
  float cf_inlierDistanceThreshold;
  
  int al_innerIterations;
  int al_outerIterations;
  int al_minNumInliers; 
  float al_minError;
  float al_inlierMaxChi2;
  
  int vz_step;

  // Input parameters handling.
  g2o::CommandArgs arg;
  
  // Optional input parameters.
  arg.param("ng_minImageRadius", ng_minImageRadius, 10, "Specify the minimum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_maxImageRadius", ng_maxImageRadius, 30, "Specify the maximum number of pixels composing the square where to take points for a normal computation");
  arg.param("ng_minPoints", ng_minPoints, 50, "Specify the minimum number of points to be used to compute a normal");
  arg.param("ng_worldRadius", ng_worldRadius, 0.1f, "Specify the max distance for a point to be used to compute a normal");
  arg.param("ng_scale", ng_scale, 1.0f, "Specify the scaling factor to apply on the depth image");
  arg.param("ng_curvatureThreshold", ng_curvatureThreshold, 1.0f, "Specify the max surface curvature threshold for which normals are discarded");
  
  arg.param("cf_inlierNormalAngularThreshold", cf_inlierNormalAngularThreshold, M_PI / 6.0f, "Maximum angle between the normals of two points to regard them as iniliers");
  arg.param("cf_flatCurvatureThreshold", cf_flatCurvatureThreshold, 0.02f, "Maximum curvature value for a point to be used for data association");
  arg.param("cf_inlierCurvatureRatioThreshold", cf_inlierCurvatureRatioThreshold, 1.3f, "Maximum curvature ratio value between two points to regard them as iniliers");
  arg.param("cf_inlierDistanceThreshold", cf_inlierDistanceThreshold, 0.5f, "Maximum metric distance between two points to regard them as iniliers");

  arg.param("al_innerIterations", al_innerIterations, 1, "Specify the inner iterations");
  arg.param("al_outerIterations", al_outerIterations, 10, "Specify the outer iterations");
  arg.param("al_minNumInliers", al_minNumInliers, 10000, "Specify the minimum number of inliers to consider an alignment good");
  arg.param("al_minError", al_minError, 10.0f, "Specify the minimum error to consider an alignment good");
  arg.param("al_inlierMaxChi2", al_inlierMaxChi2, 9e3, "Max chi2 error value for the alignment step");
  
  arg.param("vz_step", vz_step, 1, "A graphic element is drawn each vz_step elements");

  // Last parameter has to be the working directory.
  arg.paramLeftOver("working_directory", working_directory, ".", "Path of the working directory", true);

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
  viewerLayout->addWidget(viewer);
  viewer->init();
  viewer->setAxisIsDrawn(true);

  QLabel *stepLabel = new QLabel("Step", mainWindow);
  QLabel *pointsLabel = new QLabel("Points", mainWindow);
  QLabel *normalsLabel = new QLabel("Normals", mainWindow);
  QLabel *covariancesLabel = new QLabel("Covariances", mainWindow);
  QLabel *correspondencesLabel = new QLabel("Correspondences", mainWindow);
  QCheckBox *stepByStepCheckBox = new QCheckBox("Step-By-Step", mainWindow);
  QSpinBox *stepSpinBox = new QSpinBox(mainWindow);
  stepSpinBox->setMinimum(0);
  stepSpinBox->setSingleStep(1);
  QDoubleSpinBox *pointsSpinBox = new QDoubleSpinBox(mainWindow);
  pointsSpinBox->setMinimum(0.0f);
  pointsSpinBox->setSingleStep(0.25f);
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
  while(mainWindow->isVisible()) {
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
