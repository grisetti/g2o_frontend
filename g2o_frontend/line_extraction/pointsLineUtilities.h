/*
 * smExtraction.h
 *
 *  Created on: Dic 04, 2012
 *      Author: Martina
 */

#ifndef pointsLineUtilities_H
#define pointsLineUtilities_H

#include <fstream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "g2o_frontend/thesis/LaserRobotData.h"

 using namespace cv;

class pointsLineUtilities
{

public:
	pointsLineUtilities();
	virtual ~pointsLineUtilities();
	void linePopulation(LaserRobotData& ldata);
	//void cleanPoints();
	
	std::vector<Vec3f>* _pointsOriginal;
	std::vector<Vec3f>* _points;
	std::vector<float> _ranges;
	
	
};

#endif // pointsLineUtilities_H
