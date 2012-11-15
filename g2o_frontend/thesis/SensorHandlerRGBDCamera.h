
#ifndef SENSORHANDLERRGBDCAMERA_H
#define SENSORHANDLERRGBDCAMERA_H
#include "SensorHandler.h"
#include "SensorRGBDCamera.h"
#include "PrioritySynchronousDataQueue.h"

class SensorHandlerRGBDCamera : public SensorHandler
{

public:
	SensorHandlerRGBDCamera();
	virtual ~SensorHandlerRGBDCamera();
	bool setQueue(PriorityDataQueue* queue_);
	bool setSensor(Sensor* sensor_s);
	void registerCallback();
	void unregisterCallback();

};

#endif // SENSORHANDLERRGBDCAMERA_H
