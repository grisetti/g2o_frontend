/*
 * viewerGUI.h
 *
 *  Created on: Dic 05, 2012
 *      Author: Martina
 */

#ifndef VIEWERGUI_H_
#define VIEWERGUI_H_

#include <fstream>
#include "viewer.h"
#include "IEdgesExtractor.h"
#include "line_extraction2d.h"
#include <Qt/qapplication.h>

#include "g2o_frontend/sensor_data/laser_robot_data.h"

//changing this..
// typedef std::pair<LaserRobotData*, LaserRobotData::Vector2fVector> LaserData;
// typedef std::vector<LaserData> LaserDataVector;
//..in this
typedef std::pair<g2o::VertexSE2*, LaserRobotData::Vector2fVector> VertexData;
typedef std::vector<VertexData> VertexDataVector;

/**for adjacent lines**/
struct line2DwithPoints{
    Line2D l;
    Eigen::Vector2f p0;
    Eigen::Vector2f p1;
};
typedef std::vector<line2DwithPoints/*, Eigen::aligned_allocator<Line2D>*/ > LinesAdjacent;
typedef std::vector<LinesAdjacent/*, Eigen::aligned_allocator<std::vector<Line2D> > */> LinesAdjacentVector;

// file .g2o to be read for the line extraction, it has to be modified with the line info founded
// std::string filename;
// file .g2o with the original graph plus the new lines info
std::string outfilename;



class ViewerGUI : public QMainWindow, public Ui::MainWindow
{
	 Q_OBJECT

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// 		ViewerGUI(LaserRobotData* theLaserData, Vector2fVector* theOriginalPoints, QWidget *parent=0);
// 		ViewerGUI(LaserDataVector* TheLdvector, QWidget *parent=0);
// TODO
		ViewerGUI(VertexDataVector* TheVLdvector, const Eigen::Isometry2d TheOffset, QWidget *parent=0);

		void linesInfoExtraction(Line2DExtractor::IntLineMap::const_iterator it_, const Line2DExtractor::IntLineMap& linesMap_, Vector2fVector& currentPoints_) const;
		void addingData(VertexDataVector::iterator it_);
		int slider1value;
		int slider2value;
		int slider3value;
		
		std::string algotype;
		IEdgesExtractor* edgeExtr;
		Line2DExtractor* lineExtractor;
		Point2DClusterer* clusterer;
// 		LaserRobotData* laserData;		
// 		Vector2fVector* originalPoints;
		
		//changing this..
// 		LaserDataVector* ldvector;
// 		LaserData ld;
		//in this..
		VertexDataVector* vldvector;
		VertexData vld;
		
		//TODO
		/*to calculate the offset of the laser data, taking into account the odometry of the robot*/
		Eigen::Isometry2d offset;
		
		lineContainer lc;
		int numIteration;
		/*vector of adajcent lines*/
		LinesAdjacentVector lAdjacentVector;

	public slots:
	 void updateVal1(int val);
	 void updateVal2(int val);
	 void updateVal3(int val);
	 void showOriginal();
	 void lineExtraction();
	 void setAlgorithm();
	 void setIdIteration();
	 void ComputeAll();
};

#endif
