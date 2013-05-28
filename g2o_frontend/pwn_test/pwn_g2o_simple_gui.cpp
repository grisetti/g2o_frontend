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
  float ng_scale = 1.0f;
  float ng_curvatureThreshold = 1.0f;
  int al_innerIterations = 1;
  int al_outerIterations = 10;
  int vz_step = 5;

  // Input parameters handling.
  g2o::CommandArgs arg;
  
  // Optional input parameters.
  arg.param("ng_scale", ng_scale, 1.0f, "Specify the scaling factor to apply on the depth image. [float]");
  arg.param("ng_curvatureThreshold", ng_curvatureThreshold, 1.0f, "Specify the max surface curvature threshold for which normals are discarded. [float]");
  arg.param("al_innerIterations", al_innerIterations, 1, "Specify the inner iterations. [int]");
  arg.param("al_outerIterations", al_outerIterations, 10, "Specify the outer iterations. [int]");
  arg.param("vz_step", vz_step, 5, "A graphic element is drawn each vz_step elements. [int]");

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
  QHBoxLayout* hlayout = new QHBoxLayout();
  mainWindow->setLayout(hlayout);
  QVBoxLayout* vlayout = new QVBoxLayout();
  hlayout->addItem(vlayout);
  QVBoxLayout* vlayout2 = new QVBoxLayout();
  hlayout->addItem(vlayout2);
  hlayout->setStretch(1,1);

  QListWidget* listWidget = new QListWidget(mainWindow);
  listWidget->setSelectionMode(QAbstractItemView::MultiSelection );
  vlayout->addWidget(listWidget);
  PWNQGLViewer* viewer = new PWNQGLViewer(mainWindow);
  vlayout2->addWidget(viewer);

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
  
  size_t maxCount = 1000;
  for(size_t i = 0; i < vertexIds.size() &&  i < maxCount; ++i) {
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
  
  std::vector<DrawableFrame*> drawableFrameVector;
  GLParameterFrame *parameterFrame = new GLParameterFrame(vz_step);
  PWNMapperController *controller = new PWNMapperController();
  controller->init(graph);
  viewer->init();
  mainWindow->show();
  viewer->show();
  listWidget->show();
  size_t i = 0;
  while(i < maxCount-1) {
    QListWidgetItem *listItem = listWidget->item(i);
    string idString = listItem->text().toUtf8().constData();
    int index = atoi(idString.c_str());
    VertexSE3 *v = dynamic_cast<VertexSE3*>(graph->vertex(index));
    if(v) {
      
      if(!controller->addVertex(v))
    	continue;
      controller->alignIncrementally();
      
      Eigen::Isometry3f globalTransform = controller->lastFrame()->globalTransform();
      globalTransform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
      DrawableFrame *drawableFrame = new DrawableFrame(globalTransform, parameterFrame, controller->lastFrame());
      viewer->addDrawable(drawableFrame);
      while(viewer->drawableList().size() > controller->maxDequeSize()) {
       	DrawableFrame *front = dynamic_cast<DrawableFrame*>(viewer->drawableList()[0]);
       	viewer->popFront();
       	delete front;
      }
    }
    
    i++;
    viewer->updateGL();
    application.processEvents();
  }
  
  return 0;
};
