
#include "SensorHandlerRGBDCamera.h"

SensorHandlerRGBDCamera::SensorHandlerRGBDCamera()
{

}

SensorHandlerRGBDCamera::~SensorHandlerRGBDCamera()
{

}

bool SensorHandlerRGBDCamera::setQueue(PriorityDataQueue* queue_)
{
	PrioritySynchronousDataQueue* prioritySyncQueue = dynamic_cast<PrioritySynchronousDataQueue*>(queue_);
	if (prioritySyncQueue == 0) return false;
	_queue = queue_;
	return true;
}


bool SensorHandlerRGBDCamera::setSensor(Sensor* sensor_)
{
	SensorRGBDCamera* rgbdCamera = dynamic_cast<SensorRGBDCamera*>(sensor_);
	if (rgbdCamera == 0) return false;
	_sensor = sensor_;
	return true;
}

void SensorHandlerRGBDCamera::registerCallback()
{
}

void SensorHandlerRGBDCamera::unregisterCallback()
{

}
