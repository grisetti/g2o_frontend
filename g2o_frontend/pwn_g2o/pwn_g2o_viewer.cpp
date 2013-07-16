#include "g2o/stuff/command_args.h"
#include "g2o/stuff/timeutil.h"

#include <iostream>
#include <fstream>
#include <set>
#include <sys/stat.h>

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QListWidget>

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "g2o_frontend/pwn_mapper/pwn_mapper_controller.h"
#include "g2o_frontend/pwn_viewer/pwn_qglviewer.h"
#include "g2o_frontend/pwn_viewer/drawable_frame.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_frame.h"

#include <unistd.h>

using namespace pwn;
using namespace std;
using namespace g2o;

int main(int argc, char** argv) {
  /************************************************************************
   *                           Input Handling                             *
   ************************************************************************/
  string g2o_filename;

  // Variables for the input parameters. 
  float al_scale;
  float al_curvatureThreshold;
  int al_innerIterations;
  int al_outerIterations;
  int al_minNumInliers;
  int al_imageRows;
  int al_imageCols;
  int startingVertex;
  float al_minError;
  int vz_step;
  int chunkStep;
  float chunkAngle;
  float chunkDistance;
  int pwnSaving;

  // Input parameters handling.
  g2o::CommandArgs arg;
  
  // Optional input parameters.
  arg.param("al_scale", al_scale, 1.0f, "Specify the scaling factor to apply on the depth image");
  arg.param("al_curvatureThreshold", al_curvatureThreshold, 0.2f, "Specify the max surface curvature threshold for which normals are discarded");
  arg.param("al_innerIterations", al_innerIterations, 1, "Specify the inner iterations");
  arg.param("al_outerIterations", al_outerIterations, 10, "Specify the outer iterations");
  arg.param("al_minNumInliers", al_minNumInliers, 10000, "Specify the minimum number of inliers to consider an alignment good");
  arg.param("al_minError", al_minError, 10.0f, "Specify the minimum error to consider an alignment good");
  arg.param("al_imageRows", al_imageRows, 480, "Specify the number of rows of the depth image associated to the pinhole point projector");
  arg.param("al_imageCols", al_imageCols, 640, "Specify the number of columns of the depth image associated to the pinhole point projector");
  arg.param("startingVertex", startingVertex, 0, "Specify the vertex id from which to start the process");
  arg.param("vz_step", vz_step, 5, "A graphic element is drawn each vz_step elements");
  arg.param("chunkStep", chunkStep, 1000000, "Reset the process every chunkStep images");
  arg.param("chunkAngle", chunkAngle, M_PI/4, "Reset the process each time the camera has rotated of chunkAngle radians from the first frame");
  arg.param("chunkDistance", chunkDistance, 0.5, "reset the process each time the camera has moved of chunkDistance meters from the first frame");
  arg.param("pwnSaving", pwnSaving, 0, "choose if you want to save or not the pwn clouds during the process");

  // Last parameter has to be the working directory.
  arg.paramLeftOver("g2o_input_filename", g2o_filename, "", "g2o input inputfilename", true);

  // Set parser input.
  arg.parseArgs(argc, argv);
  
  /************************************************************************
   *                           Window Creation                            *
   ************************************************************************/
  QApplication application(argc,argv);
  QWidget* mainWindow = new QWidget();
  mainWindow->setWindowTitle("pwn_simpleViewer");
  QHBoxLayout* baseLayout = new QHBoxLayout();
  mainWindow->setLayout(baseLayout);
  QVBoxLayout* listWidgetLayout = new QVBoxLayout();
  baseLayout->addItem(listWidgetLayout);
  QVBoxLayout* qglviewerLayout = new QVBoxLayout();
  baseLayout->addItem(qglviewerLayout);
  baseLayout->setStretch(1.0f, 1.0f);

  QListWidget* listWidget = new QListWidget(mainWindow);
  listWidget->setSelectionMode(QAbstractItemView::MultiSelection );
  listWidgetLayout->addWidget(listWidget);
  PWNQGLViewer* viewer = new PWNQGLViewer(mainWindow);
  qglviewerLayout->addWidget(viewer);

  /************************************************************************
   *                          Loading Graph                               *
   ************************************************************************/
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  SlamLinearSolver *linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton *solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);
  SparseOptimizer *graph = new SparseOptimizer();
  graph->setAlgorithm(solverGauss);
    
  graph->load(g2o_filename.c_str());
  
  vector<int> vertexIds(graph->vertices().size());
  int k = 0;
  for(OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
    vertexIds[k++] = (it->first);
  }
  
  sort(vertexIds.begin(), vertexIds.end());
  
  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
    g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3*>(_v);
    if(!v)
      continue;
    OptimizableGraph::Data *d = v->userData();
    while(d) {
      RGBDData *rgbdData = dynamic_cast<RGBDData*>(d);
      if(!rgbdData) {
	d = d->next();
	continue;
      }
      char buff[1024];
      sprintf(buff, "%d", v->id());
      QString listItem(buff);
      listWidget->addItem(listItem);
      d = d->next();
    }
  }
  
  Matrix3f cameraMatrix;
  cameraMatrix <<
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  Isometry3f sensorOffset = Isometry3f::Identity();
  sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
  Quaternionf quaternion = Quaternionf(0.5f, -0.5f, 0.5f, -0.5f);
  sensorOffset.linear() = quaternion.toRotationMatrix();
  sensorOffset.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  std::vector<DrawableFrame*> drawableFrameVector;
  GLParameterFrame *parameterFrame = new GLParameterFrame(vz_step);
  PWNMapperController *controller = new PWNMapperController(graph);
  controller->setChunkStep(chunkStep);
  controller->setChunkAngle(chunkAngle);
  controller->setChunkDistance(chunkDistance);
  controller->setMinNumInliers(al_minNumInliers);
  controller->setMinError(al_minError);
  controller->setCameraMatrix(cameraMatrix);
  controller->setSensorOffset(sensorOffset);
  controller->setReduction(al_scale);
  controller->setImageRows(al_imageCols);
  controller->setImageCols(al_imageRows);
  controller->setCurvatureThreshold(al_curvatureThreshold);
  controller->setPwnSaving(pwnSaving);
  viewer->init();
  viewer->setAxisIsDrawn(true);
  mainWindow->show();
  viewer->show();
  listWidget->show();
  int i = startingVertex;
  while(viewer->isVisible()) {
    if(i < listWidget->count() && i >= 0) {
      QListWidgetItem *listItem = listWidget->item(i);
      string idString = listItem->text().toUtf8().constData();
      int index = atoi(idString.c_str());
      VertexSE3 *v = dynamic_cast<VertexSE3*>(graph->vertex(index));
      if(v) {     
	if(!controller->addVertex(v))
	  continue;
	controller->alignIncrementally();
	Eigen::Isometry3f globalTransform = controller->framesDeque()->back()->globalTransform();
	globalTransform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
	DrawableFrame *drawableFrame = new DrawableFrame(globalTransform, 
							 parameterFrame, 
							 controller->framesDeque()->back());      
	viewer->addDrawable(drawableFrame);
	while(viewer->drawableList().size() > controller->maxDequeSize()) {
	  DrawableFrame *front = dynamic_cast<DrawableFrame*>(viewer->drawableList()[0]);
	  viewer->popFront();
	  delete front;
	}
      }      
      i++;
    }
    viewer->updateGL();
    application.processEvents();
  }
  
  graph->save("graphwithpwn.g2o");

  return 0;
};
