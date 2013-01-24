#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <qapplication.h>
#include "pwn_qglviewer.h"
#include "drawable_points.h"
#include "gl_parameter_points.h"
#include "g2o/stuff/command_args.h"

using namespace Eigen;
using namespace g2o;
using namespace std;

int main (int argc, char** argv) {
  float pointSize;
  float normalLenght;
  string filename;

  g2o::CommandArgs arg;
  arg.param("pointSize",pointSize,1,"size of the points") ;
  arg.param("normalLenght",normalLenght,0,"lenght of the normals") ;
  arg.paramLeftOver("filename", filename, "", "filename", true);
  arg.parseArgs(argc, argv);

  PointWithNormalVector points;
  if (! points.load(filename.c_str())){
    cerr << "unable to load points from file [" << filename << "]" << endl;
    return 0;
  }

  QApplication application(argc,argv);
  PWNQGLViewer* viewer = new PWNQGLViewer();
  viewer->setWindowTitle("pwn_simpleViewer");
  GLParameterPoints* pointsParams = new GLParameterPoints(pointSize,Eigen::Vector4f(0.5,0.5,0.1,1));
  pointsParams->setStep(1.0f);
  Eigen::Isometry3f T;
  T.setIdentity();
  DrawablePoints* drawablePoints = new DrawablePoints(T, pointsParams, &points);
  pointsParams->setStep(1);
  viewer->init();
  viewer->show();
  viewer->addDrawable(drawablePoints);
  cerr << "drawList " << viewer->drawableList().size() << endl;
  
  return application.exec();
}
