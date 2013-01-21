/*
 * CloudUtils.cpp
 *
 *  Created on: Nov 27, 2012
 *      Author: malcom
 */

#include "CloudUtils.h"

CloudUtils::CloudUtils(std::vector<Vec3f>  * destination,std::vector<Vec3f>  * source) {
	src=source;
	dst=destination;
}



void CloudUtils::cleanCloud(int val1, int val2, int val3)
{
	float zmax=val2;
	float zmin=val1;
	float p=(float)val3/1000.0f;
	//float offset=((sqrt(p)*zmax)-zmin)/(sqrt(p)-1);

	//float k=1/((zmax-offset)*(zmax-offset));

	float offset=(p*zmax-zmin)/(p-1);

	float k=1./(zmax-offset);

	cout << "Off: "<<offset<<" K: "<<k<<endl;

	dst->clear();

	for (int i=0;i<(int)src->size();i++)
	{

		float z=(*src)[i][2];
		float P2=k*((z-offset)/**(z-offset)*/);
		double randomNum=((double) rand() / (RAND_MAX));


		if(randomNum<=P2 )
		{
			Vec3f tmp((*src)[i][0],(*src)[i][1],(*src)[i][2]);
			dst->push_back(tmp);
		}
		else
		{
			//(*dst)[i][2]=0;
			//NOP
		}
	}
	cout << "DST HAS SIZE OF "<<dst->size()<<endl;


}

void CloudUtils::restore()
{
	*dst=*src;
}
