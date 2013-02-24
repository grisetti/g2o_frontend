/*
 * Stamped.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */


#include "sensor_data.h"
#include "sensor.h"
void Stamped::setTimeStamp(double ts) {
	_timeStamp = ts;
}


ParameterizedSensorData::~ParameterizedSensorData(){}


int ParameterizedSensorData::paramIndex() const {
  const Sensor* s = sensor();
  if (s && s->parameter()){
    return s->parameter()->id();
  }
  return _paramIndex;
}

void ParameterizedSensorData::setParamIndex(int paramIndex_) {
  if (sensor() && sensor()->parameter() && sensor()->parameter()->id()!=paramIndex_){
    assert(0 && "YOU CANT SET A PARAMETER TO A DATA, WHEN THE PARAMETER OF THE SENSOR HAS A DIFFERENT INDEX");
  }
  _paramIndex =paramIndex_;
}

const g2o::Parameter* ParameterizedSensorData::parameter() const {

  const g2o::HyperGraph::DataContainer* container = dataContainer();
  const g2o::OptimizableGraph::Vertex* v = dynamic_cast<const g2o::OptimizableGraph::Vertex*> (container);
  if (! v) {
    cerr << "dataContainer = " << container << endl;
    cerr << "dynamic cast failed" << endl;
    return 0;
  }
		
  const g2o::OptimizableGraph* g = v->graph();
  if (! g) {
    cerr << "no graph" << endl;
    return 0;
  }
		
  const g2o::Parameter* p = g->parameters().getParameter(_paramIndex);
  if (! p) {
    cerr << "no param, " << "paramIndex: " << _paramIndex << endl;
    return 0;
  }	
  return p;
}
