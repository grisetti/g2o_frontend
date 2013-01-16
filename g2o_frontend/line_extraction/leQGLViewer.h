#ifndef LEDQGLVIEWER_H
#define LEDQGLVIEWER_H

#include <QGLViewer/qglviewer.h>
#include "g2o_frontend/thesis/LaserRobotData.h"
#include "line_extraction2d.h"

// #include "opencv2/opencv.hpp"
// #include "opencv2/core/core.hpp"
//PCL
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/passthrough.h>


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

typedef std::vector<Vector2fVector> lineContainer;


class leQGLViewer: public QGLViewer
{
	public:
		leQGLViewer(QWidget *parent);
		virtual void init();
		virtual void draw();
		void setDataPointer(LaserRobotData::Vector2fVector*);
		LaserRobotData::Vector2fVector* data;
		bool lineFound;
// 		Line2DExtractor::IntLineMap* lineContainerSM;
		lineContainer* lContainer;
};

#endif // LEDQGLVIEWER_H
