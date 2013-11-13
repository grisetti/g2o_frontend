#ifndef LEDQGLVIEWER_H
#define LEDQGLVIEWER_H

#include <QGLViewer/qglviewer.h>
#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "line_extraction2d.h"

	/**
   * \brief helper for setting up a camera for qglviewer
   */
  class StandardCamera : public qglviewer::Camera
  {
    public:
      StandardCamera() : _standard(true) {};

      float zNear() const {
        if (_standard) 
          return 0.001f; 
        else 
          return Camera::zNear(); 
      }

      float zFar() const
      {  
        if (_standard) 
          return 10000.0f; 
        else 
          return Camera::zFar();
      }

      bool standard() const {return _standard;}
      void setStandard(bool s) { _standard = s;}

    private:
      bool _standard;
  };

typedef std::vector<Vector3fVector> lineContainer;


class leQGLViewer: public QGLViewer
{
	public:
		leQGLViewer(QWidget *parent);
		virtual void init();
		virtual void draw();
		void setDataPointer(LaserRobotData::Vector2fVector*);
		LaserRobotData::Vector2fVector* data;
		bool lineFound;
		lineContainer* lContainer;
};

#endif // LEDQGLVIEWER_H
