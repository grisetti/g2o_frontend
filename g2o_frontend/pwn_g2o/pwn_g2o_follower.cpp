#include <fstream>
#include <unistd.h>
#include <set>
#include <sys/stat.h>

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QListWidget>
#include <QPushButton>

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include "g2o_frontend/pwn_mapper/pwn_loop_closer_controller.h"
#include "g2o_frontend/pwn_viewer/pwn_qglviewer.h"
#include "g2o_frontend/pwn_viewer/drawable_frame.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_frame.h"
#include "g2o_frontend/pwn_viewer/drawable_trajectory.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_trajectory.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/statscalculatorintegralimage.h"
#include "g2o_frontend/pwn2/informationmatrixcalculator.h"
#include "g2o_frontend/pwn2/depthimageconverterintegralimage.h"

#include "g2o_frontend/pwn_utils/pwn_utils.h"

using namespace std;
using namespace Eigen;
using namespace pwn;
using namespace g2o;

// Variables for the input parameters.
int vz_pwn;
int vz_step;
float vz_pointSize;
float di_scale;
float di_scaleFactor;

Matrix3f cameraMatrix;
Isometry3f sensorOffset;
DepthImage depthImage, scaledDepthImage;  

bool extractRGBDData(G2OFrame *frame, DepthImageConverter *converter);

int main(int argc, char** argv) {
  /************************************************************************
   * Input Handling *
   ************************************************************************/
  string g2o_filename;  

  // Input parameters handling.
  g2o::CommandArgs arg;
  
  // Optional input parameters.
  arg.param("vz_pwn", vz_pwn, 0, "Choose to visualize rgbd data or pwn data");
  arg.param("vz_step", vz_step, 1, "A graphic element is drawn each vz_step elements");
  arg.param("vz_pointSize", vz_pointSize, 1.0f, "Dimension of the point composing the clouds");
  arg.param("di_scale", di_scale, 1, "Size scale factor to apply when loading a depth image");
  arg.param("di_scaleFactor", di_scaleFactor, 0.001f, "Depth image values scaling factor");
 
  // Last parameter has to be the working directory.
  arg.paramLeftOver("g2o_input_filename", g2o_filename, "", "g2o input filename", true);

  // Set parser input.
  arg.parseArgs(argc, argv);
  
  /************************************************************************
   * Window Creation *
   ************************************************************************/
  QApplication application(argc,argv);
  QWidget* mainWindow = new QWidget();
  mainWindow->setWindowTitle("Priscilla Catacombs 14/02/2013");
  QHBoxLayout* baseLayout = new QHBoxLayout();
  mainWindow->setLayout(baseLayout);
  QVBoxLayout* listWidgetLayout = new QVBoxLayout();
  baseLayout->addItem(listWidgetLayout);
  QVBoxLayout* qglviewerLayout = new QVBoxLayout();
  baseLayout->addItem(qglviewerLayout);
  baseLayout->setStretch(1.0f, 2.0f);

  QListWidget* listWidget = new QListWidget();
  listWidget->setSelectionMode(QAbstractItemView::MultiSelection);
  listWidgetLayout->addWidget(listWidget);
  PWNQGLViewer* viewer = new PWNQGLViewer(mainWindow);
  qglviewerLayout->addWidget(viewer);
  
  /************************************************************************
   * Loading Graph *
   ************************************************************************/
  SparseOptimizer *graph = new SparseOptimizer();
  graph->load(g2o_filename.c_str());
  vector<int> vertexIds(graph->vertices().size());
  int k = 0;
  std::cerr << "Checking verteces... ";
  for(OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
    vertexIds[k++] = (it->first);
  }
  std::cerr << " done." << std::endl;
  std::cerr << "Sorting verteces... ";
  sort(vertexIds.begin(), vertexIds.end());
  std::cerr << " done." << std::endl;
  std::cerr << "Found " << vertexIds.size() << " verteces." << std::endl;

  std::cerr << "Checking verteces types... ";
  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
    g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3*>(_v);
    if(!v)
      continue;
    OptimizableGraph::Data *d = v->userData();
    while(d) {
      if(vz_pwn) {
	PWNData *pwnData = dynamic_cast<PWNData*>(d);
	if(!pwnData) {
	  d = d->next();
	  continue;
	}
      }
      else {
	RGBDData *rgbdData = dynamic_cast<RGBDData*>(d);
	if(!rgbdData) {
	  d = d->next();
	  continue;
	}
      }
      
      char buff[1024];
      sprintf(buff, "%d", v->id());
      QString listItem(buff);
      listWidget->addItem(listItem);
      QListWidgetItem *lastItem = listWidget->item(listWidget->count() - 1);
      lastItem->setHidden(false);
      d = d->next();
    }
  }
  std::cerr << " done." << std::endl;

  /************************************************************************
   * Setting Variables *
   ************************************************************************/
  // Load depth image
  DepthImage depth, scaledDepth;

  // Initialize pwn objects
  cameraMatrix << 
    525.0f,   0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f,   0.0f,   1.0f;
  float minDistance = 0.5f;
  float maxDistance = 5.0f;
  PinholePointProjector projector;
  projector.setCameraMatrix(cameraMatrix);
  projector.setTransform(Isometry3f::Identity());
  projector.setMaxDistance(maxDistance);
  projector.setMinDistance(minDistance);
  projector.scale(1.0f/(float)di_scale);
  
  StatsCalculatorIntegralImage statsCalculatorIntegralImage;
  int minImageRadius = 10;
  int maxImageRadius = 30;
  int minPoints = 50;
  float worldRadius = 0.1f;
  float curvatureThreshold = 1.0f;
  statsCalculatorIntegralImage.setWorldRadius(worldRadius);
  statsCalculatorIntegralImage.setMinImageRadius(minImageRadius);
  statsCalculatorIntegralImage.setMaxImageRadius(maxImageRadius);
  statsCalculatorIntegralImage.setMinPoints(minPoints);
  statsCalculatorIntegralImage.setCurvatureThreshold(curvatureThreshold);
  
  PointInformationMatrixCalculator pointInformationMatrixCalculator;
  pointInformationMatrixCalculator.setCurvatureThreshold(curvatureThreshold);
  
  NormalInformationMatrixCalculator normalInformationMatrixCalculator;
  normalInformationMatrixCalculator.setCurvatureThreshold(curvatureThreshold);
  
  DepthImageConverterIntegralImage *converter = new DepthImageConverterIntegralImage(&projector, 
										     &statsCalculatorIntegralImage,
										     &pointInformationMatrixCalculator, 
										     &normalInformationMatrixCalculator);
  
  std::vector<Isometry3f> trajectory;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > trajectoryColors;
 
  sensorOffset = Isometry3f::Identity();
  sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
  Quaternionf quaternion;
  quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
  sensorOffset.linear() = quaternion.toRotationMatrix();
  sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  PWNLoopCloserController *controller = new PWNLoopCloserController(graph);
  controller->setSensorOffset(sensorOffset);
  
  viewer->init();
  viewer->setAxisIsDrawn(true);
  viewer->show();

  mainWindow->show();
  mainWindow->showMaximized();
  
  listWidget->show();

  GLParameterTrajectory *parameterTrajectory = new GLParameterTrajectory(0.03f, Vector4f(1.0f, 0.0f, 1.0f, 1.0f));
  DrawableTrajectory *drawableTrajectory = new DrawableTrajectory(Isometry3f::Identity(), parameterTrajectory, &trajectory, &trajectoryColors);
  viewer->addDrawable(drawableTrajectory);
  GLParameterFrame *parameterFrame = new GLParameterFrame(vz_step);

  /************************************************************************
   * Drawing Trajectory *
   ************************************************************************/
  trajectory.clear();
  trajectoryColors.clear();

  std::cerr << "Drawing trajectory... ";
  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
    g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3*>(_v);
    if(!v)
      continue;
    OptimizableGraph::Data *d = v->userData();
    bool foundPwnData = false;
    bool foundRGBDData = false;
    while(d) {
      if(vz_pwn) {
	PWNData *pwnData = dynamic_cast<PWNData*>(d);
	if(!pwnData) {
	  d = d->next();
	  continue;
	}
	foundPwnData = true;      
      }
      else {
	RGBDData *rgbdData = dynamic_cast<RGBDData*>(d);
	if(!rgbdData) {
	  d = d->next();
	  continue;
	}
	foundRGBDData = true;
      }
      
      d = d->next();
    }

    Isometry3f estimate;
    isometry3d2f(estimate, v->estimate());
    if(foundPwnData || foundRGBDData) {
      trajectory.push_back(estimate);
      trajectoryColors.push_back(Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.75f));
    }
    else {
      trajectory.push_back(estimate);
      trajectoryColors.push_back(Eigen::Vector4f(0.5f, 0.5f, 0.5f, 0.3f));
    }
  }
  std::cerr << " done." << std::endl;
  
  /************************************************************************
   * MAIN DRAWING LOOP *
   ************************************************************************/
  int i = 0;
  // int count = 0;
  Isometry3f previousEstimate = Isometry3f::Identity();
  previousEstimate.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
  while(viewer->isVisible()) {
    bool changed = false;
    
    // Read next vertex
    if(i < listWidget->count() - 1) {
      QListWidgetItem* item = listWidget->item(i);
      if(item) {
	string idString = item->text().toUtf8().constData();
	int index = atoi(idString.c_str());
	VertexSE3 *v = dynamic_cast<VertexSE3*>(graph->vertex(index));
	if(v) {
	  G2OFrame *currentFrame = new G2OFrame(v);
	  if(vz_pwn) {
	    controller->extractPWNData(currentFrame);
	  }
	  else {
	    extractRGBDData(currentFrame, converter);
	  }
	  Eigen::Isometry3f estimate = Isometry3f::Identity();
	  for (int c = 0; c < 4; c++)
	    for (int r = 0; r < 3; r++)
	      estimate.matrix()(r, c) = v->estimate().matrix()(r, c);
	  estimate.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
	  DrawableFrame *drawableFrame = new DrawableFrame(estimate, parameterFrame, currentFrame);
	  parameterFrame->parameterPoints()->setPointSize(vz_pointSize);
	  viewer->addDrawable(drawableFrame);	    
	  // Update camera
	  if(i == 0)
	    viewer->updateCameraPosition(estimate);
	  else
	    viewer->updateCameraPosition(previousEstimate);
	  previousEstimate = estimate;
	  changed = true;	
	}
      }
      i++;
    }

    if(viewer->drawableList().size() > 5) {
      Drawable *d = viewer->drawableList()[1];
      DrawableFrame *dFrame = dynamic_cast<DrawableFrame*>(d);
      if(dFrame) {
	Frame *f = dFrame->frame();
	viewer->erase(1);
	delete dFrame;
	delete f;
	changed = true;      
      }
    }

    if(changed)
      viewer->updateGL();
    application.processEvents();

    // char screenName[1024];
    // sprintf(screenName, "screen%06d.jpg", count);
    // viewer->setSnapshotFormat(QString("PNG"));
    // viewer->setSnapshotQuality(-1);
    // viewer->saveSnapshot(QString().sprintf(screenName, "screen%06d.jpg", count), true);    
    // count++;
    
    // if(i == 1) {
    //   int a;
    //   std::cin >> a;
    // }
  }

  return 0;
};

bool extractRGBDData(G2OFrame *frame, DepthImageConverter *converter) {
  // Get the list of data from the input vertex and check if one of them it's a RGBDData
  OptimizableGraph::Data *d = frame->vertex()->userData();
  RGBDData *rgbdData = 0;
  while(d && !rgbdData) {
    rgbdData = dynamic_cast<RGBDData*>(d);
    d = d->next();
  }
  if(rgbdData) {
    std::string filename = rgbdData->baseFilename() + "_depth.pgm";
    cerr << "loading  " << filename << endl;
    // Read the depth image
    if (!depthImage.load(filename.c_str(), true, di_scaleFactor)) {
      cerr << "Impossible to load image " << filename << endl;
      return false;
    }
    
    // Scale the depth image
    DepthImage::scale(scaledDepthImage, depthImage, di_scale);

    // Set frame parameters
    frame->cameraMatrix() = cameraMatrix;
    frame->sensorOffset() = sensorOffset;
    
    // Get the global transformation of the cloud
    Eigen::Isometry3f originPose = Eigen::Isometry3f::Identity();
    originPose.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    Eigen::Isometry3d T = frame->vertex()->estimate();    
    for(int c = 0; c < 4; c++)
      for(int r = 0; r < 3; r++)
	originPose.matrix()(r, c) = T.matrix()(r, c);
    
    frame->globalTransform() = originPose;
    frame->globalTransform().matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    frame->previousFrameTransform().setIdentity();
    frame->previousFrameTransform().matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
    frame->setPreviousFrame(0);
    
    // Compute the stats for the current cloud
    converter->compute(*frame, scaledDepthImage, sensorOffset);

    return true;
  }
  else
    return false;
}
