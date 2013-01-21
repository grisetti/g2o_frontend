/*
 * viewerGUI.h
 *
 *  Created on: Dic 05, 2012
 *      Author: Martina
 */

#include "leQGLViewer.h"
#include <fstream>
// #include <GL/glut.h>

void drawCircle(double radius, float x, float y, float z, int segments, int arc){
	float increment = (float)arc/(float)segments;
	float angle = 0;
	for(int i = 0; i<segments;i++) {
		glBegin(GL_TRIANGLES);
		glVertex3f(x, y, z);
		glVertex3f(x+radius*cos(angle), y+radius*sin(angle), z);
		glVertex3f(x+radius*cos(angle+increment), y+radius*sin(angle+increment), z);
		glEnd();
		angle+=increment;		
	}
}

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
	glEnable(GL_POINT_SMOOTH);
  glEnable(GL_BLEND); 
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  //glEnable(GL_CULL_FACE);
  glShadeModel(GL_FLAT);
  //glShadeModel(GL_SMOOTH);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
// 	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE );
	
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
{drawAxis();
	glNormal3f(0.f, 0.f, 1.f);
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
			glColor4f(1.f,0.f,0.f,0.5f);
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
		//cout << "line found!" << endl;
		for(int i=0; i<lContainer->size(); i++)
		{
			Vector2fVector line = (*lContainer)[i];
			
			glBegin(GL_LINES);
			glColor4f(0.f, 1.f, 0.f, 0.5f);			
			glVertex3f(line[0].x(), line[0].y(), 0.f);
			glVertex3f(line[1].x(), line[1].y(), 0.f);
			
// 			glPushMatrix();
// 			glTranslatef(line[0].x(),line[0].y(), 0.f);
// 			glColor4f(0.f, 0.f, 1.f, 0.5f);
// 			glutSolidSphere(0.5f,20, 20);
// 			glPopMatrix();
			
			glColor4f(0.f, 0.f, 1.f, 0.5f);
			drawCircle(0.08f,line[0].x(), line[0].y(), 0.f, 60, 360);
			glColor4f(1.f, 0.f, 0.f, 0.5f);
			drawCircle(0.08f,line[1].x(), line[1].y(), 0.f, 60, 360);
			glEnd();
		}
	}
	glColor3f(1,1,1);
}

void leQGLViewer::setDataPointer(LaserRobotData::Vector2fVector* p)
{
	data = p;
}
