#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QListWidget>

#include "g2o_frontend/pwn_viewer/pwn_qglviewer.h"
#include "g2o_frontend/pwn_viewer/drawable_points.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_points.h"
#include "g2o_frontend/pwn_viewer/drawable_normals.h"
#include "g2o_frontend/pwn_viewer/gl_parameter_normals.h"

#include "g2o/stuff/command_args.h"
#include "g2o_frontend/basemath/bm_se3.h"

#include "g2o_frontend/pwn_core/cloud.h"

using namespace Eigen;
using namespace g2o;
using namespace std;
using namespace pwn;
using namespace pwn_viewer;

int main (int argc, char** argv) {
  float pointSize;
  float pointStep;
  float normalLenght;
  float normalStep;
  float alpha;
  int applyTransform;
  int step;

  const int maxFiles = 1000;
  std::vector<string> filenames(maxFiles);

  
  g2o::CommandArgs arg;
  arg.param("pointSize",pointSize,1.0f,"size of the points") ;
  arg.param("normalLenght",normalLenght,0,"lenght of the normals") ;
  arg.param("transform",applyTransform,1,"apply transform") ;
  arg.param("step",step,1,"show one cloud each step cloud") ;
  arg.param("alpha",alpha,1.0f,"alpha channel for points") ;
  arg.param("pointStep",pointStep,1,"step of the points") ;
  arg.param("normalStep",normalStep,1,"step of the normals") ;
  arg.paramLeftOver("filenames", filenames[0], "", "filenames", true);
  for (int i=1; i<maxFiles; i++){
    arg.paramLeftOver("", filenames[i], "", "", true);
  }
  arg.parseArgs(argc, argv);
  
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
  Eigen::Isometry3f T;
  T.setIdentity();
  T.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

  viewer->init();
  mainWindow->show();
  viewer->show();
  listWidget->show();

  viewer->setAxisIsDrawn(true);

  // Manage viewer
  while (mainWindow->isVisible()) {
    if(viewer->drawableList().size() == 0) {
      for(int i = 0; i < maxFiles; i+=step) {
	if(filenames[i] == "")
	  break;

	pwn::Cloud *cloud = new pwn::Cloud();
	Isometry3f transform;
	if(!cloud->load(transform, filenames[i].c_str())) {
	  cerr << "unable to load points from file [" << filenames[i] << "]" << endl;
	  return 0;
	} else {
	  //cout << "Transform: " << t2v(transform).transpose() << endl;
	  listWidget->addItem(QString(filenames[i].c_str()));
	}
	transform.matrix().row(3) << 0.0f, 0.0f, 0.0f, 1.0f;
	if(applyTransform)
	  cloud->transformInPlace(transform);

	GLParameterPoints* pointsParams = new GLParameterPoints(pointSize, Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
	pointsParams->setStep(pointStep);
	DrawablePoints* drawablePoints = new DrawablePoints(T, pointsParams, &cloud->points(), &cloud->normals());
	viewer->addDrawable(drawablePoints);

	GLParameterNormals* normalParams = new GLParameterNormals(pointSize, Eigen::Vector4f(0.0f,0.0f,1.0f,alpha), normalLenght);
	DrawableNormals* drawableNormals = new DrawableNormals(T, normalParams, &cloud->points(), &cloud->normals());
	normalParams->setStep(normalStep);
	normalParams->setNormalLength(normalLenght);
	viewer->addDrawable(drawableNormals);
      }
    }
    
    bool selectionChanged= false;
    for (int i = 0; i<listWidget->count(); i++){
      QListWidgetItem* item = listWidget->item(i);
      int dpIndex = i*2;
      int dnIndex = dpIndex+1;
      Drawable* drawablePoints = viewer->drawableList().at(dpIndex);
      Drawable* drawableNormals = viewer->drawableList().at(dnIndex);
      if (item && item->isSelected()){
	if(!drawablePoints->parameter()->show())
	  selectionChanged = true;
	drawablePoints->parameter()->setShow(true);
	drawableNormals->parameter()->setShow(true);
      } else {
	if(drawablePoints->parameter()->show())
	  selectionChanged = true;
	drawablePoints->parameter()->setShow(false);
	drawableNormals->parameter()->setShow(false);
      }
    }
    if (selectionChanged)
      viewer->updateGL();

    application.processEvents();
  }
}
