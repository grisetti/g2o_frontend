/*
 * CloudUtils.h
 *
 *  Created on: Nov 27, 2012
 *      Author: malcom
 */

#ifndef CLOUDUTILS_H_
#define CLOUDUTILS_H_

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

class CloudUtils
{
public:
	CloudUtils(std::vector<Vec3f>  * destination,std::vector<Vec3f>  * source);
	std::vector<Vec3f>  * dst;
	std::vector<Vec3f>  * src;
	void cleanCloud(int val1, int val2, int val3);
	void restore();
};

#endif /* CLOUDUTILS_H_ */
