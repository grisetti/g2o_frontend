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

#include "pwn_qglviewer.h"
#include "drawable_points.h"
#include "gl_parameter_points.h"
#include "drawable_normals.h"
#include "gl_parameter_normals.h"

#include "g2o/stuff/command_args.h"

#include "../pwn2/frame.h"

using namespace Eigen;
using namespace g2o;
using namespace std;
using namespace pwn;

int main (int argc, char** argv) {
  float pointSize;
  float pointStep;
  float normalLenght;
  float normalStep;
  float alpha;
  const int maxFiles = 1000;
  std::vector<string> filenames(maxFiles);

  
  g2o::CommandArgs arg;
  arg.param("pointSize",pointSize,1.0f,"size of the points") ;
  arg.param("normalLenght",normalLenght,0,"lenght of the normals") ;
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
  pwn::Frame *frame = new pwn::Frame();
  while (mainWindow->isVisible()) {
    if(viewer->drawableList().size() == 0) {
      for(int i = 0; i < maxFiles; i++) {
	if(filenames[i] == "")
	  break;

	if(!frame->load(filenames[i].c_str())) {
	  cerr << "unable to load points from file [" << filenames[i] << "]" << endl;
	  return 0;
	} else {
	  listWidget->addItem(QString(filenames[i].c_str()));
	}
      
	GLParameterPoints* pointsParams = new GLParameterPoints(pointSize, Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
	pointsParams->setStep(pointStep);
	DrawablePoints* drawablePoints = new DrawablePoints(T, pointsParams, &frame->points(), &frame->normals());
	viewer->addDrawable(drawablePoints);

	GLParameterNormals* normalParams = new GLParameterNormals(pointSize, Eigen::Vector4f(0.0f,0.0f,1.0f,alpha), normalLenght);
	DrawableNormals* drawableNormals = new DrawableNormals(T, normalParams, &frame->points(), &frame->normals());
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
