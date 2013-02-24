
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
  virtual const g2o::Parameter* parameter() const { return _parameter;}

  virtual g2o::Parameter* parameter() { return _parameter;}

  /** 
   * Paramater setter, purely virtual
   */
  virtual bool setParameter(g2o::Parameter* parameter_) = 0;

  virtual int paramIndex() = 0;
  virtual void setNum(int num_) = 0;
protected:

	
  g2o::Parameter* _parameter;
  int _num;
};

#endif /* SENSOR_H_ */
