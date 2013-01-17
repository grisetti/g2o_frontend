/*
 * viewerGUI.h
 *
 *  Created on: Dic 05, 2012
 *      Author: Martina
 */

#include "leQGLViewer.h"
#include <fstream>

leQGLViewer::leQGLViewer(QWidget *parent): QGLViewer(parent), data(NULL), lineFound(false), lContainer(NULL)
{
	setAxisIsDrawn(false);
} 

void leQGLViewer::init()
{
	QGLViewer::init();
//	Light disabled
// 	glDisable(GL_LIGHTING);
// 	glDisable(GL_LIGHT0);
// 	glDisable(GL_LIGHT1);

	setBackgroundColor(QColor::fromRgb(51, 51, 51));

  // some default settings i like
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND); 
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  //glEnable(GL_CULL_FACE);
  glShadeModel(GL_FLAT);
  //glShadeModel(GL_SMOOTH);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	setAxisIsDrawn(false);

  // don't save state
  setStateFileName(QString::null);

  // mouse bindings
  setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
  setMouseBinding(Qt::MiddleButton, CAMERA, TRANSLATE);

  // keyboard shortcuts
  setShortcut(CAMERA_MODE, 0);
  setShortcut(EXIT_VIEWER, 0);
  //setShortcut(SAVE_SCREENSHOT, 0);

  // replace camera
  qglviewer::Camera* oldcam = camera();
  qglviewer::Camera* cam = new StandardCamera();
  setCamera(cam);
  cam->setPosition(qglviewer::Vec(0., 0., 75.));
  cam->setUpVector(qglviewer::Vec(0., 1., 0.));
  cam->lookAt(qglviewer::Vec(0., 0., 0.));
  delete oldcam;
}
#if 1
				ofstream osp2("points2.dat");
				ofstream os2("lines2.dat");
#endif

void leQGLViewer::draw()
{	
	drawAxis();
	// draw all the points
	if(!lineFound) 
	{
		
#if 0
				for (size_t j =0; j<data->size(); j++){
					osp2 << (*data)[j].transpose() << endl;
				}
				osp2.flush();
#endif
				
		for (size_t i=0; i<data->size(); i++)
		{				
			glPointSize(3.f);
			glBegin(GL_POINTS);
			glColor4f(0.f,1.f,0.f,0.5f);
			glVertex3f((*data)[i].x(), (*data)[i].y(), 0.f);
			glEnd();
		}
	}
	//draw the point belonging to the lines found
	else 
	{
#if 0
		for(int i=0; i<lContainer->size(); i++)
		{
			Vector2fVector line = (*lContainer)[i];
			os2 << line[0].x() << " " << line[0].y() << endl;
			os2 << line[1].x() << " " << line[1].y() << endl;
			os2 << endl;
			os2 << endl;
		}
			os2.flush();
#endif
		
		glLineWidth(3.f);
		glBegin(GL_LINES);
		
		//cout << "line found!" << endl;
		for(int i=0; i<lContainer->size(); i++)
		{
			glColor4f(1.f, 1.f, 0.f, 0.5f);
			Vector2fVector line = (*lContainer)[i];
			glVertex3f(line[0].x(), line[0].y(), 0.f);
			glVertex3f(line[1].x(), line[1].y(), 0.f);
		}
		glEnd();
	}
	glColor3f(1,1,1);
}

void leQGLViewer::setDataPointer(LaserRobotData::Vector2fVector* p)
{
	data = p;
}
