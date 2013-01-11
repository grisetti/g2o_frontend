/*
 * viewerGUI.h
 *
 *  Created on: Dic 05, 2012
 *      Author: Martina
 */

#ifndef VIEWERGUI_H_
#define VIEWERGUI_H_

#include "viewer.h"
#include "IEdgesExtractor.h"
#include <Qt/qapplication.h>

//PCL
// #include <pcl/point_cloud.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/passthrough.h>

class ViewerGUI : public QMainWindow, public Ui::MainWindow
{
	 Q_OBJECT

	public:
		ViewerGUI(LaserRobotData* theLaserData, LaserRobotData::Point2DVector* theLinesPoints, LaserRobotData::Point2DVector* theOriginalPoints, QWidget *parent=0);
		int slider1value;
		int slider2value;
		int slider3value;
		
		std::string algotype;
		IEdgesExtractor* edgeExtr;
		LaserRobotData* laserData;
		LaserRobotData::Point2DVector* linesPoints;
		LaserRobotData::Point2DVector* originalPoints;
		l3dContainer lc;

	public slots:
	 void updateVal1(int val);
	 void updateVal2(int val);
	 void updateVal3(int val);
	 void showOriginal();
	 void lineExtraction();
	 void setAlgorithm();
};

#endif
