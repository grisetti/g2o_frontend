/*
 * OverridedViewer.h
 *
 *  Created on: Nov 27, 2012
 *      Author: malcom
 */

#ifndef OVERRIDEDVIEWER_H_
#define OVERRIDEDVIEWER_H_

#include <QGLViewer/qglviewer.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "CloudUtils.h"


//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>


using namespace cv;

#ifndef PLANE
#define PLANE
struct plane
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	Vec3f color;
    float a,b,c,d;
    Vec3f com;
	plane()
	{

		this->color[0]=((float)rand())/RAND_MAX;
		this->color[1]=((float)rand())/RAND_MAX;
		this->color[2]=((float)rand())/RAND_MAX;
        a=b=c=d=0;
	}
};

typedef std::vector<plane> planesContainer;
#endif

class OverridedViewer : public QGLViewer
{
public :
	OverridedViewer(QWidget *parent);
	virtual void draw();
	void getDataPointer(std::vector<Vec3f> * thePtr);
	std::vector<Vec3f> * dataptr;
	std::vector<Vec3f> * nearVector;
	int planes;
	planesContainer * planesContainerPTR;
};

#endif /* OVERRIDEDVIEWER_H_ */
