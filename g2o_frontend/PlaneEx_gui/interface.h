/*
 * interface.h
 *
 *  Created on: Nov 27, 2012
 *      Author: malcom
 */

#ifndef INTERFACE_H_
#define INTERFACE_H_

#include "viewer_slider.h"
#include <qapplication.h>
#include "CloudUtils.h"
#include "Eigen/Core"
#include "opencv2/opencv.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

using namespace std;

#ifndef PLANE
#define PLANE
struct plane
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	Vec3f color;
	plane()
	{

		this->color[0]=((float)rand())/RAND_MAX;
		this->color[1]=((float)rand())/RAND_MAX;
		this->color[2]=((float)rand())/RAND_MAX;
	}
};


typedef std::vector<plane> planesContainer;
#endif

class ViewerInterface : public QMainWindow, public Ui::MainWindow
{
	 Q_OBJECT

public:
		ViewerInterface(CloudUtils * theUtils,std::vector<Vec3f> * theCloud,std::vector<Vec3f> * theFull, QWidget *parent=0);
		int slider1value;
		int slider2value;
		int slider3value;
		CloudUtils * utils;
		std::vector<Vec3f> * Cloud;
		std::vector<Vec3f> * Full;
		std::vector<Vec3f> near;
		planesContainer planes;

public slots:
	 void updateZmin(int val);
	 void updateZmax(int val);
	 void updateP(int val);
	 void showFull();
};

#endif
