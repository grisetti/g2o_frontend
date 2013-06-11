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
#include "g2o_frontend/pwn_viewer/drawable_trajectory.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_trajectory.h"

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
  float ng_scale;
  float ng_curvatureThreshold;
  int al_innerIterations;
  int al_outerIterations;
  int vz_step;
  int chunkStep;
  float chunkAngle;
  float chunkDistance;

  // Input parameters handling.
  g2o::CommandArgs arg;
  
  // Optional input parameters.
  arg.param("ng_scale", ng_scale, 1.0f, "Specify the scaling factor to apply on the depth image");
  arg.param("ng_curvatureThreshold", ng_curvatureThreshold, 1.0f, "Specify the max surface curvature threshold for which normals are discarded");
  arg.param("al_innerIterations", al_innerIterations, 1, "Specify the inner iterations");
  arg.param("al_outerIterations", al_outerIterations, 10, "Specify the outer iterations");
  arg.param("vz_step", vz_step, 5, "A graphic element is drawn each vz_step elements");
  arg.param("chunkStep", chunkStep, 1000000, "Reset the process every chunkStep images");
  arg.param("chunkAngle", chunkAngle, M_PI/4, "Reset the process each time the camera has rotated of chunkAngle radians from the first frame");
  arg.param("chunkDistance", chunkDistance, 0.5, "reset the process each time the camera has moved of chunkDistance meters from the first frame");

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
      QListWidgetItem *lastItem = listWidget->item(listWidget->count() - 1);
      lastItem->setHidden(true);
      d = d->next();
    }
  }

  std::vector<Isometry3f> trajectory;
  std::vector<G2OFrame*> frames;
  frames.resize(listWidget->count());
  std::fill(frames.begin(), frames.end(), (G2OFrame*)0);
  PWNMapperController *controller = new PWNMapperController();
  controller->init(graph);
  controller->setChunkStep(chunkStep);
  controller->setChunkAngle(chunkAngle);
  controller->setChunkDistance(chunkDistance);
  viewer->init();
  viewer->setAxisIsDrawn(true);
  mainWindow->show();
  viewer->show();
  listWidget->show();
  int i = 0;
  GLParameterTrajectory *parameterTrajectory = new GLParameterTrajectory(0.03f, Vector4f(1.0f, 0.0f, 1.0f, 1.0f));
  DrawableTrajectory *drawableTrajectory = new DrawableTrajectory(Isometry3f::Identity(), parameterTrajectory, &trajectory);
  viewer->addDrawable(drawableTrajectory);
  //Isometry3f globalInitialGuess = Isometry3f::Identity();
  Isometry3f globalT = Isometry3f::Identity();
  GLParameterFrame *parameterFrame = new GLParameterFrame(vz_step); 
  while(viewer->isVisible()) {
    bool changed = false;
    if(i < listWidget->count()) {
      QListWidgetItem *listItem = listWidget->item(i);
      string idString = listItem->text().toUtf8().constData();
      int index = atoi(idString.c_str());
      VertexSE3 *v = dynamic_cast<VertexSE3*>(graph->vertex(index));
      if(v) {      
      	if(!controller->addVertex(v))
      	  continue;
      	controller->alignIncrementally();
      	//Eigen::Isometry3f initialGuess = controller->alInitialGuess();
      	//globalInitialGuess = globalInitialGuess * initialGuess;
	G2OFrame *frame = controller->lastFrame();
	Eigen::Isometry3f localT = frame->previousFrameTransform();
	globalT = globalT * localT;
	trajectory.push_back(globalT);
	listItem->setHidden(false);
	changed = true;
      }
      i++;
    }
    for(int k = 0; k < listWidget->count(); k++){
      QListWidgetItem* item = listWidget->item(k);
      if(item) {
	if(item->isSelected()) {
	  string idString = item->text().toUtf8().constData();
	  int index = atoi(idString.c_str());
	  VertexSE3 *v = dynamic_cast<VertexSE3*>(graph->vertex(index));
	  if(v && !frames[k]) {
	    G2OFrame *currentFrame = new G2OFrame(v);
	    controller->addVertex(*currentFrame);
	    DrawableFrame *drawableFrame = new DrawableFrame(trajectory[k], parameterFrame, frames[k]); 
	    viewer->addDrawable(drawableFrame);
	    changed = true;
	  }	
	}
	else {
	  if(frames[k]) {
	    for(size_t j = 0; j < viewer->drawableList().size(); j++) {
	      DrawableFrame *drawableFrame = dynamic_cast<DrawableFrame*>(viewer->drawableList()[j]);
	      if(drawableFrame) {
		G2OFrame *currentFrame = dynamic_cast<G2OFrame*>(drawableFrame->frame());
		if(currentFrame) {
		  if(currentFrame->vertex()->id() == frames[k]->vertex()->id()) {
		    viewer->erase(j);
		    delete drawableFrame;
		    delete frames[k]; 
		    frames[k] = 0;
		    changed = true;
		    break;
		  }
		}
	      }
	    }
	  }
	}
      }
    }
    if(changed)
      viewer->updateGL();
    application.processEvents();
  }

  return 0;
};
