#ifndef LEDQGLVIEWER_H
#define LEDQGLVIEWER_H

#include <QGLViewer/qglviewer.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "g2o_frontend/thesis/LaserRobotData.h"
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

#ifndef LINE3d
#define LINE3d
struct line3d
{
	//vector<vector2d> to represent start and end points belonging to the line found
	LaserRobotData::Point2DVector lineVertices;
	//random color for the points
 	cv::Vec3f rgbColors;
 	line3d() {
				this->rgbColors[0]= 0.f; //((float)rand())/RAND_MAX;
				this->rgbColors[1]= 1.f;
				this->rgbColors[2]= 0.f;
	}
};

typedef std::vector<line3d> l3dContainer;
#endif

class leQGLViewer: public QGLViewer
{
	public:
		leQGLViewer(QWidget *parent);
		virtual void init();
		virtual void draw();
// 		void setDataPointer(std::vector<Vec3f>*);
// 		std::vector<Vec3f>* dataptr;
		void setDataPointer(LaserRobotData::Point2DVector*);
		LaserRobotData::Point2DVector* data;
		bool lineFound;
		l3dContainer* lineContainer;
};

#endif // LEDQGLVIEWER_H
