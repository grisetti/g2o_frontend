#ifndef SENSORHANDLER_H_
#define SENSORHANDLER_H_

#include "sensor.h"
#include "priority_data_queue.h"

class SensorHandler {
public:
   SensorHandler();
   virtual Sensor* sensor() { return _sensor;}
   virtual const Sensor* sensor() const { return _sensor;}
   virtual bool setQueue(PriorityDataQueue* queue_) = 0;
  virtual bool setSensor(Sensor* sensor_) {_sensor = sensor_; return true;}
   virtual void registerCallback() = 0;
   
protected:
   Sensor* _sensor;
   PriorityDataQueue* _queue;
 };

#endif //SENSORHANDLER_H_
