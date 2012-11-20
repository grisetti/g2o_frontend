
#ifndef SENSOR_H_
#define SENSOR_H_

#include "g2o/core/parameter.h"
#include <list>

using namespace std;
/**
 *  Generic sensor class, purely abstract
 */
class Sensor {
public:
	/** 
	* Class constructor
	*/
	Sensor();

	/** 
	* Paramater Getter, purely virtual
	*/
	virtual g2o::Parameter* getParameter() = 0;

	/** 
	* Paramater setter, purely virtual
	*/
	virtual bool setParameter(g2o::Parameter* parameter_) = 0;

	virtual int getNum() = 0;
	virtual void setNum(int num_) = 0;
protected:

	
	g2o::Parameter* _parameter;
	int _num;
};

#endif /* SENSOR_H_ */
